#!/usr/bin/env python3
"""
learning_node.py — Online learning coordinator.

Maintains a replay buffer of engagement outcomes, fine-tunes the LSTM and
GNN models every 50 new entries, and publishes a learning-status summary.

Replay buffer
-------------
  deque(maxlen=1000) of dicts:
    { 'features': [...6 floats...], 'intent_label': int,
      'tactic_label': int, 'outcome': 'NEUTRALIZED'|'BREACHED' }

Fine-tuning trigger
-------------------
  Every 50 new buffer entries:
    • LSTM: 5 epochs on last 200 entries (intent head only)
    • GNN : 5 epochs on last 100 entries (tactic head)
  Saves updated weights to ~/.ros/acsdg_ai/models/.

Subscribes
----------
  /sensors/fusion/targets     std_msgs/String   JSON FusedTarget array
  /interceptors/fleet_status  std_msgs/String   JSON fleet snapshot
  /ai/threat_predictions      acsdg_msgs/ThreatPrediction
  /ai/swarm_classification    acsdg_msgs/SwarmClassification

Publishes
---------
  /learning/status            std_msgs/String   JSON @ 1 Hz
"""

import json
import math
import pathlib
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from acsdg_msgs.msg import ThreatPrediction, SwarmClassification

import torch
import torch.nn as nn
import torch.optim as optim

# ── Shared model paths ────────────────────────────────────────────────────────

MODEL_DIR       = pathlib.Path.home() / '.ros' / 'acsdg_ai' / 'models'
# Must match threat_predictor_node.MODEL_PATH — otherwise fine-tuning
# silently updates a file nobody reads.
LSTM_MODEL_PATH = MODEL_DIR / 'lstm_intent_v2.pt'
GNN_MODEL_PATH  = MODEL_DIR / 'gnn_swarm.pt'

INTENT_LABELS = ['KAMIKAZE', 'RECON', 'DECOY']
TACTIC_LABELS = ['PINCER', 'SATURATION', 'FEINT', 'UNKNOWN']

FINETUNE_EVERY = 50   # new entries between fine-tuning runs
LSTM_WINDOW    = 200  # entries used for LSTM fine-tune
GNN_WINDOW     = 100  # entries used for GNN fine-tune
SEQ_LEN        = 20   # must match threat_predictor_node

# ── Inline lightweight model definitions (mirrors the other nodes) ─────────────

class _LSTMIntent(nn.Module):
    """Mirror of threat_predictor_node.LSTMIntentModel so load_state_dict
    round-trips the full file (including path_head). Fine-tune only touches
    the intent head; the path head is frozen and saved back unchanged."""
    def __init__(self) -> None:
        super().__init__()
        self.lstm        = nn.LSTM(6, 64, 2, batch_first=True, dropout=0.2)
        self.fc          = nn.Linear(64, 32)
        self.path_head   = nn.Linear(32, 5 * 3)
        self.intent_head = nn.Linear(32, 3)
        self.relu        = nn.ReLU()

    def forward(self, x):
        out, _ = self.lstm(x)
        h = self.relu(self.fc(out[:, -1, :]))
        return self.intent_head(h)   # logits (intent only)


# ── Node ──────────────────────────────────────────────────────────────────────

class LearningNode(Node):

    def __init__(self) -> None:
        super().__init__('learning_node')

        self._buffer: deque = deque(maxlen=1000)
        self._buffer_lock   = threading.Lock()
        self._new_since_ft  = 0   # entries added since last fine-tune

        # Latest snapshot of live targets and fleet
        self._targets: dict = {}       # id → JSON dict
        self._fleet:   dict = {}       # id → fleet entry
        self._predictions: dict = {}   # target_id → ThreatPrediction msg
        self._last_tactic: str  = 'UNKNOWN'
        self._last_tactic_conf: float = 0.0

        # Rolling per-target frame buffers — one deque of SEQ_LEN (x,y,z,vx,vy,vz)
        # samples per active target id. Without this the LSTM fine-tuner saw a
        # single stationary frame repeated 20 times and could not learn any
        # temporal pattern. Captured into the replay entry at outcome time.
        self._frame_buffers: dict = {}   # id → deque(maxlen=SEQ_LEN)

        # Stats
        self._total_finetune_runs = 0
        self._total_lstm_samples  = 0
        self._total_gnn_samples   = 0

        MODEL_DIR.mkdir(parents=True, exist_ok=True)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            String, '/sensors/fusion/targets', self._on_targets, 10)
        self.create_subscription(
            String, '/interceptors/fleet_status', self._on_fleet, 10)
        self.create_subscription(
            ThreatPrediction, '/ai/threat_predictions', self._on_prediction, 10)
        self.create_subscription(
            SwarmClassification, '/ai/swarm_classification', self._on_swarm, 10)
        # Ground-truth outcomes from interceptor controllers. Before this
        # existed we guessed from interceptor-to-target proximity — which
        # fed the fine-tuner noisy labels. Now it's event-driven.
        self.create_subscription(
            String, '/mission/engagement_ack', self._on_engagement_ack, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/learning/status', 10)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._on_status_timer)    # 1 Hz status
        # BREACHED detection stays heuristic (no signal from enemy_driver yet);
        # NEUTRALIZED/LOST now come via engagement_ack.
        self.create_timer(2.0, self._scan_for_breaches)

        self.get_logger().info('LearningNode started')

    # ── Subscriptions ─────────────────────────────────────────────────────

    def _on_targets(self, msg: String) -> None:
        try:
            targets = json.loads(msg.data)
            self._targets = {int(t['id']): t for t in targets}
        except Exception:
            return
        # Update per-target rolling frame buffers.
        live_ids = set()
        for t in targets:
            try:
                tid = int(t['id'])
            except Exception:
                continue
            live_ids.add(tid)
            p = t.get('position', {}); v = t.get('velocity', {})
            frame = [
                p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0),
                v.get('x', 0.0), v.get('y', 0.0), v.get('z', 0.0),
            ]
            if tid not in self._frame_buffers:
                self._frame_buffers[tid] = deque(maxlen=SEQ_LEN)
            self._frame_buffers[tid].append(frame)
        # Drop buffers for disappeared targets
        for stale_id in [k for k in self._frame_buffers if k not in live_ids]:
            self._frame_buffers.pop(stale_id, None)

    def _on_fleet(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._fleet = {int(u['id']): u for u in data.get('interceptors', [])}
        except Exception:
            pass

    def _on_prediction(self, msg: ThreatPrediction) -> None:
        self._predictions[int(msg.drone_id)] = msg

    def _on_swarm(self, msg: SwarmClassification) -> None:
        self._last_tactic      = msg.tactic
        self._last_tactic_conf = msg.confidence

    # ── Outcome capture ───────────────────────────────────────────────────

    def _on_engagement_ack(self, msg: String) -> None:
        """Ground-truth outcome from the interceptor flight controller."""
        try:
            data    = json.loads(msg.data)
            tid     = int(data.get('target_id', 0))
            outcome = str(data.get('outcome', ''))
        except Exception:
            return
        # LOST is not a training signal (interceptor gave up — tells us
        # nothing about the threat's intent or the swarm's tactic).
        if outcome in ('NEUTRALIZED',) and tid != 0:
            self._record_outcome(tid, outcome)

    def _scan_for_breaches(self) -> None:
        """BREACHED heuristic: target got within 20 m of origin with non-zero
        velocity. Would ideally be replaced by an explicit signal from
        enemy_driver_node when a drone reaches the base perimeter."""
        breached_ids = []
        for tid, tgt in self._targets.items():
            p = tgt.get('position', {})
            tx, ty, tz = p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)
            v = tgt.get('velocity', {})
            vx, vy, vz = v.get('x', 0.0), v.get('y', 0.0), v.get('z', 0.0)
            if math.sqrt(tx*tx + ty*ty + tz*tz) < 20.0 and \
               math.sqrt(vx*vx + vy*vy + vz*vz) > 0.5:
                breached_ids.append(tid)
        for tid in breached_ids:
            self._record_outcome(tid, 'BREACHED')

    def _record_outcome(self, tid: int, outcome: str) -> None:
        pred = self._predictions.get(tid)
        if pred is None:
            return

        # INTENT_LABELS has 3 entries; fallback to KAMIKAZE (0) rather than
        # the out-of-range 3 that crashed CrossEntropyLoss.
        intent_idx = (INTENT_LABELS.index(pred.intent)
                      if pred.intent in INTENT_LABELS else 0)
        tactic_idx = (TACTIC_LABELS.index(self._last_tactic)
                      if self._last_tactic in TACTIC_LABELS else 3)

        # Snapshot the target's rolling frame buffer. If short, the fine-tune
        # pads later. Empty → skip the entry (nothing to learn).
        buf = self._frame_buffers.get(tid)
        if not buf:
            return
        sequence = list(buf)

        # Snapshot the swarm state at outcome time: all active drones with
        # their 7-feature (x,y,z,vx,vy,vz,threat_score) rows. Used by the
        # GNN fine-tuner to build a real multi-node graph instead of the
        # isolated-node graphs the previous code was using.
        swarm_nodes = []
        for sid, stgt in self._targets.items():
            sp = stgt.get('position', {}); sv = stgt.get('velocity', {})
            swarm_nodes.append([
                sp.get('x', 0.0), sp.get('y', 0.0), sp.get('z', 0.0),
                sv.get('x', 0.0), sv.get('y', 0.0), sv.get('z', 0.0),
                float(stgt.get('threat_score', 0.5)),
            ])

        entry = {
            'sequence':      sequence,     # list of [x,y,z,vx,vy,vz] frames
            'swarm_nodes':   swarm_nodes,  # list of 7-feature rows
            'intent_label':  intent_idx,
            'tactic_label':  tactic_idx,
            'outcome':       outcome,
        }

        with self._buffer_lock:
            self._buffer.append(entry)
            self._new_since_ft += 1
            should_ft = self._new_since_ft >= FINETUNE_EVERY

        if should_ft:
            with self._buffer_lock:
                self._new_since_ft = 0
            t = threading.Thread(target=self._finetune, daemon=True)
            t.start()

    # ── Fine-tuning ────────────────────────────────────────────────────────

    def _finetune(self) -> None:
        with self._buffer_lock:
            entries = list(self._buffer)

        self._finetune_lstm(entries[-LSTM_WINDOW:])
        self._finetune_gnn(entries[-GNN_WINDOW:])
        self._total_finetune_runs += 1
        self.get_logger().info(
            f'Fine-tune run #{self._total_finetune_runs} complete '
            f'(buffer={len(entries)})')

    def _finetune_lstm(self, entries: list) -> None:
        if len(entries) < 4:
            return
        if not LSTM_MODEL_PATH.exists():
            return

        model = _LSTMIntent()
        try:
            model.load_state_dict(torch.load(str(LSTM_MODEL_PATH), map_location='cpu'))
        except Exception as exc:
            self.get_logger().warn(f'LSTM fine-tune: could not load weights: {exc}')
            return

        # Build (batch, SEQ_LEN, 6) tensors from the captured sequences.
        # Left-pad shorter buffers by repeating the first frame so the LSTM
        # still sees SEQ_LEN timesteps.
        X_list, Y_list = [], []
        for e in entries:
            seq = e.get('sequence') or []
            if not seq:
                continue
            if len(seq) < SEQ_LEN:
                seq = [seq[0]] * (SEQ_LEN - len(seq)) + list(seq)
            else:
                seq = list(seq[-SEQ_LEN:])
            X_list.append(seq)
            Y_list.append(e['intent_label'])

        if not X_list:
            return

        X = torch.tensor(X_list, dtype=torch.float32)
        Y = torch.tensor(Y_list, dtype=torch.long)

        optimizer = optim.Adam(model.parameters(), lr=5e-4)
        ce_loss   = nn.CrossEntropyLoss()

        model.train()
        for _ in range(5):
            optimizer.zero_grad()
            logits = model(X)
            loss   = ce_loss(logits, Y)
            loss.backward()
            optimizer.step()

        model.eval()
        torch.save(model.state_dict(), str(LSTM_MODEL_PATH))
        self._total_lstm_samples += len(entries)

    def _finetune_gnn(self, entries: list) -> None:
        """GNN fine-tuning is skipped if torch_geometric is unavailable."""
        try:
            from torch_geometric.nn import GCNConv, global_mean_pool
            from torch_geometric.data import Data, Batch
            import torch.nn.functional as F
        except ImportError:
            return

        if not GNN_MODEL_PATH.exists() or len(entries) < 4:
            return

        class _GNNSwarm(nn.Module):
            def __init__(self):
                super().__init__()
                self.conv1      = GCNConv(7, 32)
                self.conv2      = GCNConv(32, 16)
                self.classifier = nn.Linear(16, 4)
            def forward(self, x, edge_index, batch):
                x = F.relu(self.conv1(x, edge_index))
                x = F.relu(self.conv2(x, edge_index))
                x = global_mean_pool(x, batch)
                return self.classifier(x)   # logits

        model = _GNNSwarm()
        try:
            model.load_state_dict(torch.load(str(GNN_MODEL_PATH), map_location='cpu'))
        except Exception as exc:
            self.get_logger().warn(f'GNN fine-tune: could not load weights: {exc}')
            return

        # Build one graph per outcome using the swarm snapshot captured at
        # ACK time: nodes = all active drones (7 features each), edges =
        # bidirectional if ≤80 m apart (matching the inference-time rule
        # in swarm_classifier_node). Single-drone snapshots get an empty
        # edge set; they contribute only a trivial node-level signal.
        EDGE_THRESHOLD = 80.0
        data_list, labels = [], []
        for e in entries:
            nodes = e.get('swarm_nodes') or []
            if not nodes:
                continue
            node_x = torch.tensor(nodes, dtype=torch.float32)
            src, dst = [], []
            for i in range(len(nodes)):
                for j in range(len(nodes)):
                    if i == j:
                        continue
                    dx = nodes[i][0] - nodes[j][0]
                    dy = nodes[i][1] - nodes[j][1]
                    dz = nodes[i][2] - nodes[j][2]
                    if (dx*dx + dy*dy + dz*dz) ** 0.5 <= EDGE_THRESHOLD:
                        src.append(i); dst.append(j)
            if src:
                edge_index = torch.tensor([src, dst], dtype=torch.long)
            else:
                edge_index = torch.zeros((2, 0), dtype=torch.long)
            data_list.append(Data(x=node_x, edge_index=edge_index))
            labels.append(e['tactic_label'])

        if not data_list:
            return

        Y = torch.tensor(labels, dtype=torch.long)
        optimizer = optim.Adam(model.parameters(), lr=5e-4)
        ce_loss   = nn.CrossEntropyLoss()

        model.train()
        for _ in range(5):
            batch_data = Batch.from_data_list(data_list)
            optimizer.zero_grad()
            logits = model(batch_data.x, batch_data.edge_index, batch_data.batch)
            loss   = ce_loss(logits, Y)
            loss.backward()
            optimizer.step()

        model.eval()
        torch.save(model.state_dict(), str(GNN_MODEL_PATH))
        self._total_gnn_samples += len(entries)

    # ── 1 Hz status publisher ──────────────────────────────────────────────

    def _on_status_timer(self) -> None:
        with self._buffer_lock:
            buf_size       = len(self._buffer)
            new_since_ft   = self._new_since_ft

        status = {
            'buffer_size':         buf_size,
            'new_since_finetune':  new_since_ft,
            'finetune_runs':       self._total_finetune_runs,
            'lstm_samples_seen':   self._total_lstm_samples,
            'gnn_samples_seen':    self._total_gnn_samples,
            'next_finetune_in':    max(0, FINETUNE_EVERY - new_since_ft),
            'last_tactic':         self._last_tactic,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LearningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
