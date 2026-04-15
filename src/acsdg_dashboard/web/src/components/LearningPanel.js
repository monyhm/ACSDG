/**
 * LearningPanel.js — AI learning status bar (full width, 40px).
 */

import React, { useState, useEffect } from 'react';
import { subscribe } from '../ros/RosConnection';

export default function LearningPanel() {
  const [status, setStatus] = useState(null);
  const [lastUpdate, setLastUpdate] = useState(null);
  const [pulse, setPulse] = useState(false);

  useEffect(() => {
    const unsub = subscribe('/learning/status', 'std_msgs/String', (msg) => {
      try {
        const data = JSON.parse(msg.data);
        setStatus(data);
        setLastUpdate(new Date());
        // Pulse when fine-tune just ran
        if (data.next_finetune_in === 50) {
          setPulse(true);
          setTimeout(() => setPulse(false), 2000);
        }
      } catch (_) {}
    });
    return unsub;
  }, []);

  const mono = { fontFamily: "'IBM Plex Mono', monospace" };

  return (
    <div style={{
      height: 40,
      background: '#060b14',
      borderTop: '1px solid #1e3a5f',
      display: 'flex',
      alignItems: 'center',
      padding: '0 12px',
      gap: 20,
      fontSize: 9,
      color: '#3a6186',
      ...mono,
    }}>
      {/* Pulse indicator */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 5 }}>
        <div style={{
          width: 6, height: 6,
          background: pulse ? '#00b4d8' : '#1e3a5f',
          boxShadow: pulse ? '0 0 6px #00b4d8' : 'none',
          transition: 'all 0.3s',
        }} />
        <span style={{ color: pulse ? '#00b4d8' : '#3a6186', letterSpacing: 1 }}>
          {pulse ? 'FINE-TUNING' : 'LEARNING IDLE'}
        </span>
      </div>

      <div style={{ width: 1, height: 20, background: '#1e3a5f' }} />

      {status ? (
        <>
          <span>BUF <span style={{ color: '#c9d6e3' }}>{status.buffer_size}</span>/1000</span>
          <span>RUNS <span style={{ color: '#c9d6e3' }}>{status.finetune_runs}</span></span>
          <span>LSTM <span style={{ color: '#c9d6e3' }}>{status.lstm_samples_seen}</span> samples</span>
          <span>GNN <span style={{ color: '#c9d6e3' }}>{status.gnn_samples_seen}</span> samples</span>
          <span>NEXT FT in <span style={{ color: '#f4a261' }}>{status.next_finetune_in}</span></span>
          <span>TACTIC <span style={{ color: '#00b4d8' }}>{status.last_tactic}</span></span>
          {lastUpdate && (
            <span style={{ marginLeft: 'auto' }}>
              UPDATED {lastUpdate.toLocaleTimeString()}
            </span>
          )}
        </>
      ) : (
        <span style={{ color: '#1e3a5f' }}>LEARNING NODE OFFLINE</span>
      )}
    </div>
  );
}
