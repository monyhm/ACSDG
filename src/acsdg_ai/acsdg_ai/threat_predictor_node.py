#!/usr/bin/env python3
"""
threat_predictor_node.py — LSTM-based threat trajectory and intent predictor.

Model
-----
  LSTM(input=6, hidden=64, layers=2, dropout=0.2)
    → fc(64→32) → ReLU
    → path_head(32→15)  reshaped to (5, 3) waypoints
    → intent_head(32→3) softmax → [KAMIKAZE, RECON, DECOY]

Pre-training
------------
  500 synthetic examples per class × 30 epochs (daemon thread, non-blocking).
  Weights saved to ~/.ros/acsdg_ai/models/lstm_intent.pt.

Subscribes
----------
  /sensors/fusion/targets   std_msgs/String   JSON FusedTarget array

Publishes
---------
  /ai/threat_predictions    acsdg_msgs/ThreatPrediction   @ 5 Hz
"""

import json
import math
import os
import pathlib
import random
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from acsdg_msgs.msg import ThreatPrediction

import torch
import torch.nn as nn
import torch.optim as optim
from geometry_msgs.msg import Point


def _make_point(x: float, y: float, z: float) -> Point:
    p = Point(); p.x = x; p.y = y; p.z = z
    return p


# ── Model ─────────────────────────────────────────────────────────────────────

INTENT_LABELS = ['KAMIKAZE', 'RECON', 'DECOY']
SEQ_LEN  = 20   # frames kept per target
IN_FEATS = 6    # x, y, z, vx, vy, vz

MODEL_DIR = pathlib.Path.home() / '.ros' / 'acsdg_ai' / 'models'
MODEL_PATH = MODEL_DIR / 'lstm_intent.pt'


class LSTMIntentModel(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.lstm = nn.LSTM(
            input_size=IN_FEATS, hidden_size=64, num_layers=2,
            batch_first=True, dropout=0.2)
        self.fc          = nn.Linear(64, 32)
        self.path_head   = nn.Linear(32, 5 * 3)   # 5 waypoints × (x,y,z)
        self.intent_head = nn.Linear(32, 3)
        self.relu    = nn.ReLU()
        self.softmax = nn.Softmax(dim=-1)

    def forward(self, x: torch.Tensor):
        """x: (batch, SEQ_LEN, 6)"""
        out, _ = self.lstm(x)
        h = self.relu(self.fc(out[:, -1, :]))
        path   = self.path_head(h).view(-1, 5, 3)
        intent = self.softmax(self.intent_head(h))
        return path, intent


# ── Synthetic data generation ─────────────────────────────────────────────────

def _gen_kamikaze(n: int = 500) -> list:
    """Straight-line inbound trajectories."""
    seqs, labels = [], []
    for _ in range(n):
        dist  = random.uniform(150, 400)
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(10, 20)
        vx = -math.cos(angle) * speed / dist * 100
        vy = -math.sin(angle) * speed / dist * 100
        vz = random.uniform(-0.5, 0.5)
        seq = []
        x, y, z = dist * math.cos(angle), dist * math.sin(angle), random.uniform(20, 80)
        for _ in range(SEQ_LEN):
            seq.append([x, y, z, vx * dist / 100, vy * dist / 100, vz])
            x += vx * 0.2; y += vy * 0.2; z += vz * 0.2
        seqs.append(seq); labels.append(0)
    return seqs, labels


def _gen_recon(n: int = 500) -> list:
    """Circular patrol patterns."""
    seqs, labels = [], []
    for _ in range(n):
        cx = random.uniform(-100, 100)
        cy = random.uniform(-100, 100)
        cz = random.uniform(30, 100)
        r  = random.uniform(50, 150)
        omega = random.uniform(0.05, 0.15)  # rad / step
        phase = random.uniform(0, 2 * math.pi)
        speed = r * omega
        seq = []
        for i in range(SEQ_LEN):
            theta = phase + omega * i
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)
            vx = -r * omega * math.sin(theta)
            vy =  r * omega * math.cos(theta)
            seq.append([x, y, cz, vx, vy, 0.0])
        seqs.append(seq); labels.append(1)
    return seqs, labels


def _gen_decoy(n: int = 500) -> list:
    """Erratic, direction-changing trajectories."""
    seqs, labels = [], []
    for _ in range(n):
        x, y, z = (random.uniform(-200, 200), random.uniform(-200, 200),
                   random.uniform(20, 120))
        vx, vy, vz = (random.uniform(-8, 8), random.uniform(-8, 8),
                      random.uniform(-1, 1))
        seq = []
        for _ in range(SEQ_LEN):
            vx += random.gauss(0, 1.5); vy += random.gauss(0, 1.5)
            spd = math.sqrt(vx*vx + vy*vy) + 1e-6
            if spd > 12:
                vx *= 12 / spd; vy *= 12 / spd
            x += vx * 0.2; y += vy * 0.2; z += vz * 0.2
            z = max(5.0, min(150.0, z))
            seq.append([x, y, z, vx, vy, vz])
        seqs.append(seq); labels.append(2)
    return seqs, labels


def _build_dataset() -> tuple:
    seqs, labels = [], []
    for gen in (_gen_kamikaze, _gen_recon, _gen_decoy):
        s, l = gen(500)
        seqs.extend(s); labels.extend(l)
    X = torch.tensor(seqs,   dtype=torch.float32)
    Y = torch.tensor(labels, dtype=torch.long)
    idx = torch.randperm(len(Y))
    return X[idx], Y[idx]


# ── Node ──────────────────────────────────────────────────────────────────────

class ThreatPredictorNode(Node):

    def __init__(self) -> None:
        super().__init__('threat_predictor_node')

        self._model   = LSTMIntentModel()
        self._model.eval()
        self._model_ready = False
        self._model_lock  = threading.Lock()

        # Per-target rolling frame buffer: id → deque(maxlen=SEQ_LEN)
        self._buffers: dict = {}
        self._targets: dict = {}  # id → latest JSON dict

        # ── Publishers / Subscriptions ────────────────────────────────────
        self._pred_pub = self.create_publisher(
            ThreatPrediction, '/ai/threat_predictions', 10)

        self.create_subscription(
            String, '/sensors/fusion/targets',
            self._on_targets, 10)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(0.2, self._on_timer)   # 5 Hz

        # ── Pre-training (daemon thread) ──────────────────────────────────
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        t = threading.Thread(target=self._pretrain, daemon=True)
        t.start()

        self.get_logger().info('ThreatPredictorNode started (pre-training in background)')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_targets(self, msg: String) -> None:
        try:
            targets = json.loads(msg.data)
        except Exception:
            return
        for tgt in targets:
            tid = int(tgt['id'])
            self._targets[tid] = tgt
            if tid not in self._buffers:
                self._buffers[tid] = deque(maxlen=SEQ_LEN)
            p = tgt.get('position', {}); v = tgt.get('velocity', {})
            self._buffers[tid].append([
                p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0),
                v.get('x', 0.0), v.get('y', 0.0), v.get('z', 0.0),
            ])

        # Prune stale target buffers
        live = {int(t['id']) for t in targets}
        for old_id in list(self._buffers.keys()):
            if old_id not in live:
                del self._buffers[old_id]
                self._targets.pop(old_id, None)

    # ── 5 Hz publish ──────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        with self._model_lock:
            ready = self._model_ready

        for tid, buf in list(self._buffers.items()):
            if len(buf) < 1:
                continue  # need at least one frame

            # Pad or use available frames
            frames = list(buf)
            if len(frames) < SEQ_LEN:
                frames = [frames[0]] * (SEQ_LEN - len(frames)) + frames

            msg = ThreatPrediction()
            msg.drone_id = tid

            if ready:
                try:
                    x = torch.tensor([frames], dtype=torch.float32)
                    with self._model_lock:
                        with torch.no_grad():
                            path, intent = self._model(x)
                    intent_np = intent[0].numpy()
                    cls = int(intent_np.argmax())
                    msg.intent      = INTENT_LABELS[cls]
                    msg.confidence  = float(intent_np[cls])
                    wp = path[0].numpy()
                    msg.predicted_path = [
                        _make_point(float(wp[i, 0]), float(wp[i, 1]), float(wp[i, 2]))
                        for i in range(5)]
                except Exception as exc:
                    self.get_logger().warn(f'Inference error tid={tid}: {exc}')
                    self._fallback(msg, frames)
            else:
                self._fallback(msg, frames)

            self._pred_pub.publish(msg)

    def _fallback(self, msg: ThreatPrediction, frames: list) -> None:
        """Simple rule-based fallback while model is training."""
        last = frames[-1]
        vx, vy = last[3], last[4]
        px, py = last[0], last[1]
        speed = math.sqrt(vx*vx + vy*vy)
        dist = math.sqrt(px*px + py*py) + 1e-6
        inbound = -(px * vx + py * vy) / (dist * (speed + 1e-6))

        if inbound > 0.7:
            msg.intent = 'KAMIKAZE'; msg.confidence = float(inbound)
        elif speed < 5.0:
            msg.intent = 'RECON';    msg.confidence = 0.5
        else:
            msg.intent = 'DECOY';    msg.confidence = 0.4

        x, y, z = last[0], last[1], last[2]
        msg.predicted_path = [
            _make_point(x + vx * (i+1) * 1.0,
                        y + vy * (i+1) * 1.0,
                        z + last[5] * (i+1) * 1.0)
            for i in range(5)]

    # ── Pre-training ──────────────────────────────────────────────────────

    def _pretrain(self) -> None:
        # Try to load saved weights first
        if MODEL_PATH.exists():
            try:
                with self._model_lock:
                    self._model.load_state_dict(torch.load(str(MODEL_PATH), map_location='cpu'))
                    self._model.eval()
                    self._model_ready = True
                self.get_logger().info('LSTM: loaded saved weights')
                return
            except Exception as exc:
                self.get_logger().warn(f'LSTM: could not load weights ({exc}), retraining')

        self.get_logger().info('LSTM: generating synthetic data and pre-training...')
        X, Y = _build_dataset()

        optimizer = optim.Adam(self._model.parameters(), lr=1e-3)
        ce_loss   = nn.CrossEntropyLoss()
        mse_loss  = nn.MSELoss()

        EPOCHS = 30
        BATCH  = 64
        N      = len(Y)

        for epoch in range(EPOCHS):
            perm = torch.randperm(N)
            total_loss = 0.0
            batches = 0
            for start in range(0, N, BATCH):
                idx = perm[start:start + BATCH]
                xb  = X[idx]; yb = Y[idx]
                optimizer.zero_grad()
                with self._model_lock:
                    path_pred, intent_pred = self._model(xb)
                # Dummy path target: repeat last position 5 times
                path_tgt = xb[:, -1, :3].unsqueeze(1).expand(-1, 5, 3)
                loss = ce_loss(intent_pred, yb) + 0.1 * mse_loss(path_pred, path_tgt)
                loss.backward()
                optimizer.step()
                total_loss += loss.item(); batches += 1

            if (epoch + 1) % 10 == 0:
                self.get_logger().info(
                    f'LSTM pre-train epoch {epoch+1}/{EPOCHS}  '
                    f'loss={total_loss/batches:.4f}')

        with self._model_lock:
            self._model.eval()
            self._model_ready = True
            torch.save(self._model.state_dict(), str(MODEL_PATH))

        self.get_logger().info(f'LSTM: pre-training complete, weights saved to {MODEL_PATH}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThreatPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
