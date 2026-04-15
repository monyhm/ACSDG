#!/usr/bin/env python3
"""
swarm_classifier_node.py — GNN-based swarm tactic classifier.

Model (torch_geometric)
-----------------------
  GCNConv(7 → 32) → ReLU
  GCNConv(32 → 16) → ReLU
  global_mean_pool → Linear(16 → 4) → Softmax
  Classes: PINCER / SATURATION / FEINT / UNKNOWN

Node features (7): x, y, z, vx, vy, vz, threat_score
Edges: bidirectional between drones within 80 m.
Pre-training: 200 synthetic examples per class × 20 epochs (daemon thread).
Weights saved to ~/.ros/acsdg_ai/models/gnn_swarm.pt.

Falls back to a rule-based classifier if torch_geometric is unavailable
or inference fails.

Subscribes
----------
  /sensors/fusion/targets   std_msgs/String   JSON FusedTarget array
  /c2/threat_scores         std_msgs/String   JSON [{id, score, state}, ...]

Publishes
---------
  /ai/swarm_classification  acsdg_msgs/SwarmClassification  @ 2 Hz
"""

import json
import math
import pathlib
import random
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from acsdg_msgs.msg import SwarmClassification

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

# Optional torch_geometric
try:
    from torch_geometric.nn import GCNConv, global_mean_pool
    from torch_geometric.data import Data, Batch
    TORCH_GEOMETRIC_AVAILABLE = True
except ImportError:
    TORCH_GEOMETRIC_AVAILABLE = False

# ── Constants ─────────────────────────────────────────────────────────────────

TACTIC_LABELS  = ['PINCER', 'SATURATION', 'FEINT', 'UNKNOWN']
EDGE_THRESHOLD = 80.0   # metres
MIN_DRONES     = 3

MODEL_DIR  = pathlib.Path.home() / '.ros' / 'acsdg_ai' / 'models'
MODEL_PATH = MODEL_DIR / 'gnn_swarm.pt'

# ── GNN Model ─────────────────────────────────────────────────────────────────

if TORCH_GEOMETRIC_AVAILABLE:
    class GNNSwarmModel(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.conv1      = GCNConv(7, 32)
            self.conv2      = GCNConv(32, 16)
            self.classifier = nn.Linear(16, 4)

        def forward(self, x, edge_index, batch):
            x = F.relu(self.conv1(x, edge_index))
            x = F.relu(self.conv2(x, edge_index))
            x = global_mean_pool(x, batch)
            return F.softmax(self.classifier(x), dim=-1)


# ── Synthetic data generation ─────────────────────────────────────────────────

def _random_swarm(n: int, cx: float, cy: float, spread: float) -> list:
    """Return list of (x, y, z, vx, vy, vz, score) tuples."""
    drones = []
    for _ in range(n):
        x = cx + random.gauss(0, spread)
        y = cy + random.gauss(0, spread)
        z = random.uniform(20, 80)
        vx = random.gauss(0, 4); vy = random.gauss(0, 4)
        score = random.uniform(0.3, 0.9)
        drones.append((x, y, z, vx, vy, 0.0, score))
    return drones


def _gen_pincer(n: int = 200) -> tuple:
    """Two groups approaching from opposite sides."""
    graphs, labels = [], []
    for _ in range(n):
        half = random.randint(2, 5)
        angle = random.uniform(0, math.pi)
        r = random.uniform(100, 200)
        group_a = []
        for _ in range(half):
            x = r * math.cos(angle) + random.gauss(0, 15)
            y = r * math.sin(angle) + random.gauss(0, 15)
            vx = -math.cos(angle) * random.uniform(5, 15)
            vy = -math.sin(angle) * random.uniform(5, 15)
            group_a.append((x, y, random.uniform(20, 80), vx, vy, 0.0, random.uniform(0.5, 0.9)))
        group_b = []
        for _ in range(half):
            x = -r * math.cos(angle) + random.gauss(0, 15)
            y = -r * math.sin(angle) + random.gauss(0, 15)
            vx = math.cos(angle) * random.uniform(5, 15)
            vy = math.sin(angle) * random.uniform(5, 15)
            group_b.append((x, y, random.uniform(20, 80), vx, vy, 0.0, random.uniform(0.5, 0.9)))
        graphs.append(group_a + group_b); labels.append(0)
    return graphs, labels


def _gen_saturation(n: int = 200) -> tuple:
    """Many drones in a tight cluster all heading toward origin."""
    graphs, labels = [], []
    for _ in range(n):
        count = random.randint(5, 12)
        cx = random.uniform(-200, 200); cy = random.uniform(-200, 200)
        dist = math.sqrt(cx*cx + cy*cy) + 1e-6
        drones = []
        for _ in range(count):
            x = cx + random.gauss(0, 20)
            y = cy + random.gauss(0, 20)
            spd = random.uniform(8, 18)
            vx = -cx / dist * spd + random.gauss(0, 1)
            vy = -cy / dist * spd + random.gauss(0, 1)
            drones.append((x, y, random.uniform(20, 80), vx, vy, 0.0, random.uniform(0.6, 1.0)))
        graphs.append(drones); labels.append(1)
    return graphs, labels


def _gen_feint(n: int = 200) -> tuple:
    """Mixed directions — some inbound, some diverging."""
    graphs, labels = [], []
    for _ in range(n):
        count = random.randint(3, 8)
        drones = []
        for _ in range(count):
            x = random.uniform(-200, 200); y = random.uniform(-200, 200)
            # Randomly inbound or outbound
            dist = math.sqrt(x*x + y*y) + 1e-6
            spd  = random.uniform(4, 14)
            sign = 1 if random.random() < 0.5 else -1
            vx = sign * (-x / dist) * spd + random.gauss(0, 2)
            vy = sign * (-y / dist) * spd + random.gauss(0, 2)
            drones.append((x, y, random.uniform(20, 80), vx, vy, 0.0, random.uniform(0.2, 0.7)))
        graphs.append(drones); labels.append(2)
    return graphs, labels


def _gen_unknown(n: int = 200) -> tuple:
    """Scattered, slow, low-threat drones."""
    graphs, labels = [], []
    for _ in range(n):
        count = random.randint(3, 6)
        drones = []
        for _ in range(count):
            x = random.uniform(-300, 300); y = random.uniform(-300, 300)
            vx = random.gauss(0, 2); vy = random.gauss(0, 2)
            drones.append((x, y, random.uniform(20, 80), vx, vy, 0.0, random.uniform(0.0, 0.3)))
        graphs.append(drones); labels.append(3)
    return graphs, labels


def _drones_to_pyg(drones: list):
    """Convert a list of (x,y,z,vx,vy,vz,score) to a torch_geometric Data object."""
    n = len(drones)
    x = torch.tensor(drones, dtype=torch.float32)  # (n, 7)

    # Build edges: bidirectional within EDGE_THRESHOLD
    src, dst = [], []
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            dx = drones[i][0] - drones[j][0]
            dy = drones[i][1] - drones[j][1]
            dz = drones[i][2] - drones[j][2]
            if math.sqrt(dx*dx + dy*dy + dz*dz) <= EDGE_THRESHOLD:
                src.append(i); dst.append(j)

    if src:
        edge_index = torch.tensor([src, dst], dtype=torch.long)
    else:
        edge_index = torch.zeros((2, 0), dtype=torch.long)

    return Data(x=x, edge_index=edge_index)


# ── Node ──────────────────────────────────────────────────────────────────────

class SwarmClassifierNode(Node):

    def __init__(self) -> None:
        super().__init__('swarm_classifier_node')

        self._model       = GNNSwarmModel() if TORCH_GEOMETRIC_AVAILABLE else None
        self._model_ready = False
        self._model_lock  = threading.Lock()

        self._targets: dict = {}      # id → JSON dict
        self._scores:  dict = {}      # id → float threat score

        # ── Publishers / Subscriptions ────────────────────────────────────
        self._cls_pub = self.create_publisher(
            SwarmClassification, '/ai/swarm_classification', 10)

        self.create_subscription(
            String, '/sensors/fusion/targets', self._on_targets, 10)
        self.create_subscription(
            String, '/c2/threat_scores', self._on_scores, 10)

        # ── Timer: 2 Hz ───────────────────────────────────────────────────
        self.create_timer(0.5, self._on_timer)

        # ── Pre-training ──────────────────────────────────────────────────
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        if TORCH_GEOMETRIC_AVAILABLE:
            t = threading.Thread(target=self._pretrain, daemon=True)
            t.start()
            self.get_logger().info(
                'SwarmClassifierNode started (GNN pre-training in background)')
        else:
            self.get_logger().warn(
                'torch_geometric not available — using rule-based fallback')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_targets(self, msg: String) -> None:
        try:
            targets = json.loads(msg.data)
            self._targets = {int(t['id']): t for t in targets}
        except Exception:
            pass

    def _on_scores(self, msg: String) -> None:
        try:
            scores = json.loads(msg.data)
            self._scores = {int(s['id']): float(s['score']) for s in scores}
        except Exception:
            pass

    # ── 2 Hz classify ─────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        active = {tid: t for tid, t in self._targets.items()
                  if t.get('state') in ('DETECTED', 'TARGETED')}

        if len(active) < MIN_DRONES:
            return

        drones = []
        for tid, tgt in active.items():
            p = tgt.get('position', {}); v = tgt.get('velocity', {})
            score = self._scores.get(tid, 0.5)
            drones.append((
                p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0),
                v.get('x', 0.0), v.get('y', 0.0), v.get('z', 0.0),
                score,
            ))

        with self._model_lock:
            ready = self._model_ready

        if ready and TORCH_GEOMETRIC_AVAILABLE:
            tactic, confidence = self._gnn_classify(drones)
        else:
            tactic, confidence = self._rule_classify(drones)

        msg = SwarmClassification()
        msg.tactic      = tactic
        msg.confidence  = confidence
        # decoy_ids: ids of low-inbound-score drones (heuristic)
        msg.decoy_ids   = [
            tid for tid, d in zip(active.keys(), drones)
            if math.sqrt(d[3]**2 + d[4]**2) < 3.0
        ]
        self._cls_pub.publish(msg)

    # ── GNN inference ─────────────────────────────────────────────────────

    def _gnn_classify(self, drones: list) -> tuple:
        try:
            data  = _drones_to_pyg(drones)
            batch = torch.zeros(len(drones), dtype=torch.long)
            with self._model_lock:
                with torch.no_grad():
                    probs = self._model(data.x, data.edge_index, batch)
            probs_np = probs[0].numpy()
            cls = int(probs_np.argmax())
            return TACTIC_LABELS[cls], float(probs_np[cls])
        except Exception as exc:
            self.get_logger().warn(f'GNN inference error: {exc}')
            return self._rule_classify(drones)

    # ── Rule-based fallback ────────────────────────────────────────────────

    def _rule_classify(self, drones: list) -> tuple:
        n = len(drones)
        if n == 0:
            return 'UNKNOWN', 0.4

        # Spread (std dev of positions)
        xs = [d[0] for d in drones]; ys = [d[1] for d in drones]
        mx = sum(xs) / n; my = sum(ys) / n
        spread = math.sqrt(sum((x-mx)**2 + (y-my)**2 for x, y in zip(xs, ys)) / n)

        # Average inbound-ness
        inbound_scores = []
        for x, y, z, vx, vy, vz, _ in drones:
            dist = math.sqrt(x*x + y*y) + 1e-6
            spd  = math.sqrt(vx*vx + vy*vy) + 1e-6
            inbound_scores.append(-(x*vx + y*vy) / (dist * spd))
        avg_inbound = sum(inbound_scores) / n

        if spread < 60 and avg_inbound > 0.5:
            return 'SATURATION', 0.65
        elif spread > 120 and avg_inbound > 0.4:
            return 'PINCER', 0.60
        elif avg_inbound < 0.1:
            return 'FEINT', 0.55
        else:
            return 'UNKNOWN', 0.45

    # ── Pre-training ──────────────────────────────────────────────────────

    def _pretrain(self) -> None:
        if MODEL_PATH.exists():
            try:
                state = torch.load(str(MODEL_PATH), map_location='cpu')
                with self._model_lock:
                    self._model.load_state_dict(state)
                    self._model.eval()
                    self._model_ready = True
                self.get_logger().info('GNN: loaded saved weights')
                return
            except Exception as exc:
                self.get_logger().warn(f'GNN: could not load weights ({exc}), retraining')

        self.get_logger().info('GNN: generating synthetic graphs and pre-training...')

        all_graphs, all_labels = [], []
        for gen in (_gen_pincer, _gen_saturation, _gen_feint, _gen_unknown):
            g, l = gen(200)
            all_graphs.extend(g); all_labels.extend(l)

        optimizer = optim.Adam(self._model.parameters(), lr=1e-3)
        ce_loss   = nn.CrossEntropyLoss()

        N      = len(all_labels)
        EPOCHS = 20
        BATCH  = 16

        for epoch in range(EPOCHS):
            idx_list = list(range(N))
            random.shuffle(idx_list)
            total_loss = 0.0; batches = 0

            for start in range(0, N, BATCH):
                batch_idx = idx_list[start:start + BATCH]
                data_list = [_drones_to_pyg(all_graphs[i]) for i in batch_idx]
                labels_t  = torch.tensor([all_labels[i] for i in batch_idx], dtype=torch.long)

                try:
                    batch_data = Batch.from_data_list(data_list)
                except Exception:
                    continue

                optimizer.zero_grad()
                with self._model_lock:
                    probs = self._model(batch_data.x, batch_data.edge_index, batch_data.batch)
                loss = ce_loss(probs, labels_t)
                loss.backward()
                optimizer.step()
                total_loss += loss.item(); batches += 1

            if (epoch + 1) % 5 == 0 and batches > 0:
                self.get_logger().info(
                    f'GNN pre-train epoch {epoch+1}/{EPOCHS}  '
                    f'loss={total_loss/batches:.4f}')

        with self._model_lock:
            self._model.eval()
            self._model_ready = True
            torch.save(self._model.state_dict(), str(MODEL_PATH))

        self.get_logger().info(f'GNN: pre-training complete, weights saved to {MODEL_PATH}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
