/**
 * SwarmPanel.js — Swarm tactic classification + recommended response.
 */

import React, { useState, useEffect } from 'react';
import { subscribe } from '../ros/RosConnection';

const TACTIC_CONFIG = {
  SATURATION: {
    color: '#ef233c',
    bg: '#ef233c11',
    response: 'ACTIVATE ALL INTERCEPTORS — PRIORITIZE PROXIMITY',
  },
  PINCER: {
    color: '#f4a261',
    bg: '#f4a26111',
    response: 'SPLIT FLEET — COVER BOTH APPROACH VECTORS',
  },
  FEINT: {
    color: '#ffd166',
    bg: '#ffd16611',
    response: 'IGNORE DECOY CLUSTER — FOCUS STRIKE GROUP',
  },
  UNKNOWN: {
    color: '#8d99ae',
    bg: '#8d99ae11',
    response: 'STANDARD ASSIGNMENT PROTOCOL',
  },
};

const PANEL = {
  background: '#080d18',
  border: '1px solid #1e3a5f',
  fontFamily: "'IBM Plex Mono', monospace",
  color: '#c9d6e3',
};

export default function SwarmPanel() {
  const [swarm, setSwarm] = useState({ tactic: 'UNKNOWN', confidence: 0, drone_count: 0 });

  useEffect(() => {
    const unsub = subscribe(
      '/ai/swarm_classification',
      'acsdg_msgs/SwarmClassification',
      (msg) => setSwarm(msg),
    );
    return unsub;
  }, []);

  const cfg = TACTIC_CONFIG[swarm.tactic] || TACTIC_CONFIG.UNKNOWN;

  return (
    <div style={{ ...PANEL, display: 'flex', flexDirection: 'column' }}>
      {/* Header */}
      <div style={{
        padding: '5px 10px', borderBottom: '1px solid #1e3a5f',
        display: 'flex', alignItems: 'center',
      }}>
        <span style={{ fontSize: 10, fontWeight: 700, color: '#00b4d8', letterSpacing: 2 }}>
          SWARM ANALYSIS
        </span>
      </div>

      <div style={{ padding: '8px 12px', background: cfg.bg }}>
        {/* Tactic name */}
        <div style={{
          fontSize: 24, fontWeight: 700, color: cfg.color,
          letterSpacing: 3, lineHeight: 1,
        }}>
          {swarm.tactic}
        </div>

        {/* Confidence bar */}
        <div style={{ marginTop: 6, marginBottom: 8 }}>
          <div style={{ fontSize: 8, color: '#3a6186', marginBottom: 3, letterSpacing: 1 }}>
            CONFIDENCE {(swarm.confidence * 100).toFixed(0)}%
          </div>
          <div style={{ height: 3, background: '#0e2240', position: 'relative' }}>
            <div style={{
              position: 'absolute', left: 0, top: 0, bottom: 0,
              width: `${swarm.confidence * 100}%`,
              background: cfg.color,
              transition: 'width 0.4s',
            }} />
          </div>
        </div>

        {/* Response strategy */}
        <div style={{
          padding: '5px 8px',
          border: `1px solid ${cfg.color}44`,
          fontSize: 9,
          color: cfg.color,
          letterSpacing: 1,
          lineHeight: 1.5,
        }}>
          ▶ {cfg.response}
        </div>
      </div>
    </div>
  );
}
