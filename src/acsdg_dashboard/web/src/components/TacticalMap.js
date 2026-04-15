/**
 * TacticalMap.js — Canvas-based tactical map.
 *
 * World coordinates: origin = base center, +x = East, +y = North, meters.
 * Map scale: 1 px = 1 m at BASE_SCALE, canvas is 800×600.
 */

import React, { useRef, useEffect, useState, useCallback } from 'react';
import { subscribe } from '../ros/RosConnection';

const W = 800, H = 600;
const SCALE = 0.9;                 // px per metre
const CX = W / 2, CY = H / 2;    // canvas centre = world origin

// ── World → canvas ────────────────────────────────────────────────────────────
function wx(x) { return CX + x * SCALE; }
function wy(y) { return CY - y * SCALE; }

// ── Hex sensor positions (r=200 m, 6 nodes) ──────────────────────────────────
const SENSOR_NODES = Array.from({ length: 6 }, (_, i) => {
  const a = (i * Math.PI) / 3;
  return { x: 200 * Math.cos(a), y: 200 * Math.sin(a) };
});

// ── Defense post positions (corners, 300 m) ───────────────────────────────────
const DEFENSE_POSTS = [
  { x:  220, y:  220 }, { x: -220, y:  220 },
  { x:  220, y: -220 }, { x: -220, y: -220 },
];

const INTENT_COLORS = {
  KAMIKAZE: '#ef233c',
  RECON:    '#f4a261',
  DECOY:    '#8d99ae',
  UNKNOWN:  '#ffffff',
};

const TACTIC_COLORS = {
  SATURATION: '#ef233c',
  PINCER:     '#f4a261',
  FEINT:      '#ffd166',
  UNKNOWN:    '#8d99ae',
};

// ── Drawing helpers ───────────────────────────────────────────────────────────

function drawGrid(ctx) {
  ctx.save();
  ctx.strokeStyle = 'rgba(0, 180, 216, 0.06)';
  ctx.lineWidth = 0.5;
  const step = 50 * SCALE;
  for (let x = CX % step; x < W; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
  }
  for (let y = CY % step; y < H; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  }
  ctx.restore();
}

function drawBase(ctx) {
  ctx.save();
  const s = 40;
  ctx.strokeStyle = '#00b4d8';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(wx(-s), wy(s), s * 2 * SCALE, s * 2 * SCALE);
  // Inner compound
  const s2 = 18;
  ctx.strokeStyle = '#00b4d8aa';
  ctx.lineWidth = 1;
  ctx.strokeRect(wx(-s2), wy(s2), s2 * 2 * SCALE, s2 * 2 * SCALE);
  // Cross marker
  ctx.beginPath();
  ctx.moveTo(wx(-8), wy(0)); ctx.lineTo(wx(8), wy(0));
  ctx.moveTo(wx(0), wy(-8)); ctx.lineTo(wx(0), wy(8));
  ctx.strokeStyle = '#00b4d8';
  ctx.stroke();
  // Label
  ctx.fillStyle = '#00b4d8';
  ctx.font = '600 9px IBM Plex Sans Condensed';
  ctx.textAlign = 'center';
  ctx.fillText('BASE', wx(0), wy(-s - 5));
  ctx.restore();
}

function drawSensors(ctx, sweepAngle) {
  SENSOR_NODES.forEach(({ x, y }, i) => {
    const px = wx(x), py = wy(y);
    // Hexagon
    ctx.save();
    ctx.beginPath();
    for (let k = 0; k < 6; k++) {
      const a = (k * Math.PI) / 3;
      const hx = px + 7 * Math.cos(a);
      const hy = py + 7 * Math.sin(a);
      k === 0 ? ctx.moveTo(hx, hy) : ctx.lineTo(hx, hy);
    }
    ctx.closePath();
    ctx.strokeStyle = '#00b4d8';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = 'rgba(0,180,216,0.15)';
    ctx.fill();

    // Radar sweep (each sensor offset by 60°)
    const sweepR = 80;
    const angle = sweepAngle + (i * Math.PI) / 3;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px + sweepR * Math.cos(angle), py + sweepR * Math.sin(angle));
    ctx.strokeStyle = 'rgba(0,180,216,0.35)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Sweep arc (fading)
    const arcSpan = Math.PI / 6;
    const grad = ctx.createConicalGradient
      ? null
      : ctx.createRadialGradient(px, py, 0, px, py, sweepR);
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.arc(px, py, sweepR, angle - arcSpan, angle);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0,180,216,0.07)';
    ctx.fill();

    ctx.restore();
  });
}

function drawDefensePosts(ctx) {
  DEFENSE_POSTS.forEach(({ x, y }) => {
    const px = wx(x), py = wy(y);
    ctx.save();
    ctx.strokeStyle = '#06d6a0';
    ctx.lineWidth = 1;
    const s = 6;
    ctx.beginPath();
    ctx.moveTo(px, py - s); ctx.lineTo(px + s, py);
    ctx.lineTo(px, py + s); ctx.lineTo(px - s, py);
    ctx.closePath();
    ctx.stroke();
    ctx.fillStyle = 'rgba(6,214,160,0.1)';
    ctx.fill();
    ctx.restore();
  });
}

function drawDrone(ctx, dx, dy, color) {
  ctx.save();
  ctx.translate(dx, dy);
  ctx.beginPath();
  ctx.moveTo(0, -8);
  ctx.lineTo(6, 6);
  ctx.lineTo(0, 3);
  ctx.lineTo(-6, 6);
  ctx.closePath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.fillStyle = color + '44';
  ctx.fill();
  ctx.restore();
}

function drawCompass(ctx) {
  const cx = W - 40, cy = H - 40, r = 22;
  ctx.save();
  ctx.strokeStyle = '#1e3a5f';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, 2 * Math.PI);
  ctx.stroke();
  // N arrow
  ctx.beginPath();
  ctx.moveTo(cx, cy - r + 4);
  ctx.lineTo(cx + 4, cy + 4);
  ctx.lineTo(cx, cy);
  ctx.closePath();
  ctx.fillStyle = '#ef233c';
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(cx, cy - r + 4);
  ctx.lineTo(cx - 4, cy + 4);
  ctx.lineTo(cx, cy);
  ctx.closePath();
  ctx.fillStyle = '#c9d6e3';
  ctx.fill();
  ctx.fillStyle = '#00b4d8';
  ctx.font = '700 9px IBM Plex Sans Condensed';
  ctx.textAlign = 'center';
  ctx.fillText('N', cx, cy - r - 3);
  ctx.restore();
}

// ── Component ─────────────────────────────────────────────────────────────────

export default function TacticalMap() {
  const canvasRef = useRef(null);
  const stateRef  = useRef({
    targets:      [],
    predictions:  {},
    fleet:        [],
    swarm:        null,
    scores:       {},
    sweepAngle:   0,
    animFrame:    null,
  });

  const [swarmBanner, setSwarmBanner] = useState(null);

  // ── ROS subscriptions ────────────────────────────────────────────────────

  useEffect(() => {
    const unsubs = [];

    unsubs.push(subscribe('/sensors/fusion/targets', 'std_msgs/String', (msg) => {
      try {
        stateRef.current.targets = JSON.parse(msg.data);
      } catch (_) {}
    }));

    unsubs.push(subscribe('/c2/threat_scores', 'std_msgs/String', (msg) => {
      try {
        const scores = JSON.parse(msg.data);
        stateRef.current.scores = Object.fromEntries(scores.map(s => [s.id, s.score]));
      } catch (_) {}
    }));

    unsubs.push(subscribe('/ai/threat_predictions', 'acsdg_msgs/ThreatPrediction', (msg) => {
      stateRef.current.predictions[msg.drone_id] = msg;
    }));

    unsubs.push(subscribe('/interceptors/fleet_status', 'std_msgs/String', (msg) => {
      try {
        const data = JSON.parse(msg.data);
        stateRef.current.fleet = data.interceptors || [];
      } catch (_) {}
    }));

    unsubs.push(subscribe('/ai/swarm_classification', 'acsdg_msgs/SwarmClassification', (msg) => {
      stateRef.current.swarm = msg;
      if (msg.confidence > 0.6) {
        setSwarmBanner({ tactic: msg.tactic, confidence: msg.confidence });
      } else {
        setSwarmBanner(null);
      }
    }));

    return () => unsubs.forEach(u => u());
  }, []);

  // ── Animation loop ───────────────────────────────────────────────────────

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const s = stateRef.current;

    s.sweepAngle = (s.sweepAngle + 0.02) % (2 * Math.PI);

    ctx.fillStyle = '#060b14';
    ctx.fillRect(0, 0, W, H);

    drawGrid(ctx);
    drawBase(ctx);
    drawSensors(ctx, s.sweepAngle);
    drawDefensePosts(ctx);

    // Range rings
    [100, 200, 300, 400].forEach(r => {
      ctx.save();
      ctx.beginPath();
      ctx.arc(CX, CY, r * SCALE, 0, 2 * Math.PI);
      ctx.strokeStyle = 'rgba(0,180,216,0.08)';
      ctx.lineWidth = 0.5;
      ctx.setLineDash([4, 8]);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(0,180,216,0.25)';
      ctx.font = '8px IBM Plex Mono';
      ctx.textAlign = 'left';
      ctx.fillText(`${r}m`, CX + r * SCALE + 3, CY + 3);
      ctx.restore();
    });

    // ── Interceptors ───────────────────────────────────────────────────────
    s.fleet.forEach(unit => {
      const ux = wx(unit.position.x), uy = wy(unit.position.y);
      const color = unit.status === 'IDLE'      ? '#06d6a0'
                  : unit.status === 'PURSUING'  ? '#00b4d8'
                  : '#f4a261';

      // Diamond shape
      ctx.save();
      ctx.translate(ux, uy);
      ctx.beginPath();
      ctx.moveTo(0, -9); ctx.lineTo(6, 0); ctx.lineTo(0, 9); ctx.lineTo(-6, 0);
      ctx.closePath();
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.fillStyle = color + '33'; ctx.fill();
      ctx.restore();

      // Label
      ctx.save();
      ctx.fillStyle = color;
      ctx.font = '600 8px IBM Plex Mono';
      ctx.textAlign = 'center';
      ctx.fillText(`I${unit.id}`, ux, uy - 12);
      ctx.restore();

      // Line to target if pursuing
      if (unit.status === 'PURSUING' && unit.target_id) {
        const tgt = s.targets.find(t => t.id === unit.target_id);
        if (tgt) {
          ctx.save();
          ctx.beginPath();
          ctx.moveTo(ux, uy);
          ctx.lineTo(wx(tgt.position.x), wy(tgt.position.y));
          ctx.strokeStyle = '#00b4d855';
          ctx.lineWidth = 1;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.restore();
        }
      }
    });

    // ── Drones ─────────────────────────────────────────────────────────────
    s.targets.forEach(tgt => {
      if (!['DETECTED', 'TARGETED'].includes(tgt.state)) return;

      const pred = s.predictions[tgt.id];
      const intent = pred ? pred.intent : 'UNKNOWN';
      const color  = INTENT_COLORS[intent] || INTENT_COLORS.UNKNOWN;
      const dx = wx(tgt.position.x), dy = wy(tgt.position.y);

      // Predicted path
      if (pred && pred.predicted_path && pred.predicted_path.length > 0) {
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(dx, dy);
        pred.predicted_path.forEach(pt => {
          ctx.lineTo(wx(pt.x), wy(pt.y));
        });
        ctx.strokeStyle = color + '55';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
        // Waypoint dots
        pred.predicted_path.forEach(pt => {
          ctx.beginPath();
          ctx.arc(wx(pt.x), wy(pt.y), 2, 0, 2 * Math.PI);
          ctx.fillStyle = color + '77';
          ctx.fill();
        });
        ctx.restore();
      }

      drawDrone(ctx, dx, dy, color);

      // Score label
      const score = s.scores[tgt.id];
      ctx.save();
      ctx.fillStyle = color;
      ctx.font = '500 8px IBM Plex Mono';
      ctx.textAlign = 'left';
      if (score !== undefined) {
        ctx.fillText(score.toFixed(2), dx + 10, dy - 5);
      }
      ctx.fillText(`T${tgt.id}`, dx + 10, dy + 6);
      ctx.restore();
    });

    drawCompass(ctx);

    s.animFrame = requestAnimationFrame(draw);
  }, []);

  useEffect(() => {
    stateRef.current.animFrame = requestAnimationFrame(draw);
    return () => {
      if (stateRef.current.animFrame) {
        cancelAnimationFrame(stateRef.current.animFrame);
      }
    };
  }, [draw]);

  // ── Render ───────────────────────────────────────────────────────────────

  return (
    <div style={{ position: 'relative', width: W, height: H, flexShrink: 0 }}>
      <canvas ref={canvasRef} width={W} height={H} style={{ display: 'block' }} />

      {/* Swarm tactic banner */}
      {swarmBanner && (
        <div style={{
          position: 'absolute',
          top: 8,
          left: '50%',
          transform: 'translateX(-50%)',
          background: '#0a0f1add',
          border: `1px solid ${TACTIC_COLORS[swarmBanner.tactic] || '#8d99ae'}`,
          padding: '4px 18px',
          color: TACTIC_COLORS[swarmBanner.tactic] || '#8d99ae',
          fontFamily: "'IBM Plex Mono', monospace",
          fontSize: 11,
          fontWeight: 600,
          letterSpacing: 2,
          whiteSpace: 'nowrap',
        }}>
          SWARM: {swarmBanner.tactic} ({(swarmBanner.confidence * 100).toFixed(0)}%)
        </div>
      )}

      {/* Corner label */}
      <div style={{
        position: 'absolute', top: 6, left: 8,
        color: '#1e3a5f', fontFamily: "'IBM Plex Mono',monospace",
        fontSize: 9, letterSpacing: 1,
      }}>
        TACTICAL DISPLAY · ENU FRAME · SCALE 1m:1px
      </div>
    </div>
  );
}
