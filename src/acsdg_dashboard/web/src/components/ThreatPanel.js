/**
 * ThreatPanel.js — Live threat list with scores, intent badges, and state.
 */

import React, { useState, useEffect } from 'react';
import { subscribe } from '../ros/RosConnection';

const PANEL = {
  background: '#080d18',
  border: '1px solid #1e3a5f',
  color: '#c9d6e3',
  fontFamily: "'IBM Plex Mono', monospace",
};

const INTENT_STYLE = {
  KAMIKAZE: { bg: '#ef233c22', border: '#ef233c', color: '#ef233c' },
  RECON:    { bg: '#f4a26122', border: '#f4a261', color: '#f4a261' },
  DECOY:    { bg: '#8d99ae22', border: '#8d99ae', color: '#8d99ae' },
  UNKNOWN:  { bg: '#ffffff11', border: '#ffffff44', color: '#ffffff88' },
};

function scoreColor(s) {
  if (s >= 0.7) return '#ef233c';
  if (s >= 0.4) return '#f4a261';
  return '#06d6a0';
}

function distToBase(pos) {
  if (!pos) return 0;
  return Math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2);
}

export default function ThreatPanel() {
  const [targets, setTargets]     = useState([]);
  const [predictions, setPreds]   = useState({});
  const [scores, setScores]       = useState({});

  useEffect(() => {
    const unsubs = [];

    unsubs.push(subscribe('/sensors/fusion/targets', 'std_msgs/String', (msg) => {
      try { setTargets(JSON.parse(msg.data)); } catch (_) {}
    }));

    unsubs.push(subscribe('/ai/threat_predictions', 'acsdg_msgs/ThreatPrediction', (msg) => {
      setPreds(prev => ({ ...prev, [msg.drone_id]: msg }));
    }));

    unsubs.push(subscribe('/c2/threat_scores', 'std_msgs/String', (msg) => {
      try {
        const arr = JSON.parse(msg.data);
        setScores(Object.fromEntries(arr.map(s => [s.id, s.score])));
      } catch (_) {}
    }));

    return () => unsubs.forEach(u => u());
  }, []);

  const rows = targets
    .filter(t => ['DETECTED', 'TARGETED'].includes(t.state))
    .map(t => ({
      ...t,
      score:  scores[t.id] ?? 0,
      intent: predictions[t.id]?.intent ?? 'UNKNOWN',
      conf:   predictions[t.id]?.confidence ?? 0,
      dist:   Math.round(distToBase(t.position)),
    }))
    .sort((a, b) => b.score - a.score);

  return (
    <div style={{ ...PANEL, display: 'flex', flexDirection: 'column', height: '100%' }}>
      {/* Header */}
      <div style={{
        padding: '5px 10px',
        borderBottom: '1px solid #1e3a5f',
        display: 'flex',
        alignItems: 'center',
        gap: 8,
      }}>
        <span style={{ color: '#00b4d8', fontSize: 10, fontWeight: 700, letterSpacing: 2 }}>
          THREAT TRACK
        </span>
        <span style={{
          marginLeft: 'auto',
          background: rows.length > 0 ? '#ef233c22' : '#06d6a022',
          border: `1px solid ${rows.length > 0 ? '#ef233c' : '#06d6a0'}`,
          color: rows.length > 0 ? '#ef233c' : '#06d6a0',
          fontSize: 9, padding: '1px 6px', letterSpacing: 1,
        }}>
          {rows.length} ACTIVE
        </span>
      </div>

      {/* Column headers */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: '30px 70px 1fr 60px 50px',
        padding: '3px 8px',
        borderBottom: '1px solid #0e2240',
        fontSize: 8,
        color: '#3a6186',
        letterSpacing: 1,
      }}>
        <span>ID</span>
        <span>INTENT</span>
        <span>SCORE</span>
        <span>STATE</span>
        <span>DIST</span>
      </div>

      {/* Rows */}
      <div style={{ flex: 1, overflowY: 'auto' }}>
        {rows.length === 0 && (
          <div style={{
            textAlign: 'center', padding: '20px 0',
            color: '#1e3a5f', fontSize: 10, letterSpacing: 1,
          }}>
            NO CONTACTS
          </div>
        )}
        {rows.map(row => {
          const is = INTENT_STYLE[row.intent] || INTENT_STYLE.UNKNOWN;
          const sc = scoreColor(row.score);
          return (
            <div key={row.id} style={{
              display: 'grid',
              gridTemplateColumns: '30px 70px 1fr 60px 50px',
              padding: '4px 8px',
              borderBottom: '1px solid #0a1628',
              alignItems: 'center',
              fontSize: 9,
            }}>
              <span style={{ color: '#00b4d8', fontWeight: 600 }}>T{row.id}</span>
              <span style={{
                background: is.bg, border: `1px solid ${is.border}`,
                color: is.color, padding: '1px 4px',
                fontSize: 8, letterSpacing: 1, whiteSpace: 'nowrap',
              }}>
                {row.intent}
              </span>
              <div style={{ padding: '0 6px' }}>
                <div style={{
                  height: 4, background: '#0e2240',
                  position: 'relative', width: '100%',
                }}>
                  <div style={{
                    position: 'absolute', left: 0, top: 0, bottom: 0,
                    width: `${row.score * 100}%`,
                    background: sc, transition: 'width 0.3s',
                  }} />
                </div>
                <div style={{ color: sc, fontSize: 8, marginTop: 1 }}>
                  {row.score.toFixed(3)}
                </div>
              </div>
              <span style={{ color: '#8d99ae', fontSize: 8 }}>{row.state}</span>
              <span style={{ color: '#c9d6e3', textAlign: 'right' }}>{row.dist}m</span>
            </div>
          );
        })}
      </div>
    </div>
  );
}
