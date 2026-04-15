/**
 * InterceptorPanel.js — Fleet status: 4 interceptors with status badges.
 */

import React, { useState, useEffect, useRef } from 'react';
import { subscribe } from '../ros/RosConnection';

const STATUS_STYLE = {
  IDLE:      { color: '#06d6a0', bg: '#06d6a022', border: '#06d6a0' },
  PURSUING:  { color: '#00b4d8', bg: '#00b4d822', border: '#00b4d8' },
  RETURNING: { color: '#f4a261', bg: '#f4a26122', border: '#f4a261' },
};

const PANEL = {
  background: '#080d18',
  border: '1px solid #1e3a5f',
  fontFamily: "'IBM Plex Mono', monospace",
  color: '#c9d6e3',
};

// Simulate battery drain for display purposes
function useBattery(id, status) {
  const ref = useRef(85 + (id * 7) % 15);
  useEffect(() => {
    const id_ = setInterval(() => {
      if (status === 'PURSUING') {
        ref.current = Math.max(0, ref.current - 0.05);
      } else if (status === 'IDLE') {
        ref.current = Math.min(100, ref.current + 0.01);
      }
    }, 1000);
    return () => clearInterval(id_);
  }, [status]);
  return ref.current;
}

function InterceptorRow({ unit }) {
  const st = STATUS_STYLE[unit.status] || STATUS_STYLE.IDLE;
  const bat = useBattery(unit.id, unit.status);
  const batColor = bat > 50 ? '#06d6a0' : bat > 20 ? '#f4a261' : '#ef233c';

  return (
    <div style={{
      display: 'grid',
      gridTemplateColumns: '28px 80px 50px 1fr 36px',
      padding: '5px 10px',
      borderBottom: '1px solid #0a1628',
      alignItems: 'center',
      gap: 6,
    }}>
      {/* ID */}
      <span style={{ color: '#00b4d8', fontWeight: 700, fontSize: 11 }}>
        #{unit.id}
      </span>

      {/* Status badge */}
      <span style={{
        background: st.bg,
        border: `1px solid ${st.border}`,
        color: st.color,
        fontSize: 8, padding: '1px 5px',
        letterSpacing: 1, textAlign: 'center',
      }}>
        {unit.status}
      </span>

      {/* Target */}
      <span style={{ fontSize: 9, color: unit.target_id ? '#f4a261' : '#1e3a5f' }}>
        {unit.target_id ? `→ T${unit.target_id}` : '—'}
      </span>

      {/* Battery bar */}
      <div style={{ position: 'relative', height: 4, background: '#0e2240' }}>
        <div style={{
          position: 'absolute', left: 0, top: 0, bottom: 0,
          width: `${bat.toFixed(0)}%`,
          background: batColor,
          transition: 'width 1s',
        }} />
      </div>

      {/* Battery % */}
      <span style={{ fontSize: 8, color: batColor, textAlign: 'right' }}>
        {bat.toFixed(0)}%
      </span>
    </div>
  );
}

export default function InterceptorPanel() {
  const [fleet, setFleet] = useState([]);

  useEffect(() => {
    const unsub = subscribe('/interceptors/fleet_status', 'std_msgs/String', (msg) => {
      try {
        const data = JSON.parse(msg.data);
        setFleet(data.interceptors || []);
      } catch (_) {}
    });
    return unsub;
  }, []);

  // Always show 4 slots, fill from fleet data
  const slots = [1, 2, 3, 4].map(id =>
    fleet.find(u => u.id === id) || { id, status: 'IDLE', target_id: 0, position: { x: 0, y: 0, z: 0 } }
  );

  return (
    <div style={{ ...PANEL, display: 'flex', flexDirection: 'column' }}>
      {/* Header */}
      <div style={{
        padding: '5px 10px', borderBottom: '1px solid #1e3a5f',
        display: 'flex', alignItems: 'center', gap: 8,
      }}>
        <span style={{ fontSize: 10, fontWeight: 700, color: '#00b4d8', letterSpacing: 2 }}>
          INTERCEPTOR FLEET
        </span>
        <span style={{ marginLeft: 'auto', fontSize: 8, color: '#3a6186' }}>
          {fleet.filter(u => u.status === 'IDLE').length}/4 AVAILABLE
        </span>
      </div>

      {/* Column headers */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: '28px 80px 50px 1fr 36px',
        padding: '3px 10px',
        borderBottom: '1px solid #0e2240',
        fontSize: 8, color: '#3a6186', letterSpacing: 1, gap: 6,
      }}>
        <span>ID</span><span>STATUS</span><span>TGT</span>
        <span>BATTERY</span><span></span>
      </div>

      {slots.map(unit => <InterceptorRow key={unit.id} unit={unit} />)}
    </div>
  );
}
