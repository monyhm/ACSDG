/**
 * MetricsPanel.js — Mission KPIs: cost saved, response time, wave, counters.
 */

import React, { useState, useEffect, useRef } from 'react';
import { subscribe } from '../ros/RosConnection';

const PANEL = {
  background: '#080d18',
  border: '1px solid #1e3a5f',
  fontFamily: "'IBM Plex Mono', monospace",
  color: '#c9d6e3',
};

function MetricBox({ label, value, color, size = 22 }) {
  return (
    <div style={{ flex: 1, padding: '6px 10px', borderRight: '1px solid #0e2240' }}>
      <div style={{ fontSize: 8, color: '#3a6186', letterSpacing: 1, marginBottom: 2 }}>
        {label}
      </div>
      <div style={{ fontSize: size, fontWeight: 600, color, lineHeight: 1 }}>
        {value}
      </div>
    </div>
  );
}

export default function MetricsPanel() {
  const [status, setStatus] = useState({
    wave_number: 0,
    drones_detected: 0,
    drones_neutralized: 0,
    drones_breached: 0,
    cost_saved: 0,
    avg_response_time_ms: 0,
  });

  // Animated cost counter
  const displayedCostRef = useRef(0);
  const [displayedCost, setDisplayedCost] = useState(0);

  useEffect(() => {
    const unsub = subscribe('/mission/status', 'acsdg_msgs/MissionStatus', (msg) => {
      setStatus({
        wave_number:         msg.wave_number,
        drones_detected:     msg.drones_detected,
        drones_neutralized:  msg.drones_neutralized,
        drones_breached:     msg.drones_breached,
        cost_saved:          msg.cost_saved,
        avg_response_time_ms: msg.avg_response_time_ms,
      });
    });
    return unsub;
  }, []);

  // Smooth count-up for cost_saved
  useEffect(() => {
    const target = status.cost_saved;
    const current = displayedCostRef.current;
    if (target <= current) { displayedCostRef.current = target; setDisplayedCost(target); return; }
    const diff = target - current;
    const step = Math.max(1000, diff / 30);
    const id = setInterval(() => {
      displayedCostRef.current = Math.min(displayedCostRef.current + step, target);
      setDisplayedCost(displayedCostRef.current);
      if (displayedCostRef.current >= target) clearInterval(id);
    }, 50);
    return () => clearInterval(id);
  }, [status.cost_saved]);

  const rt = status.avg_response_time_ms;
  const rtColor = rt === 0 ? '#8d99ae' : rt < 100 ? '#06d6a0' : rt < 300 ? '#f4a261' : '#ef233c';
  const costStr = '$' + Math.round(displayedCost).toLocaleString();

  return (
    <div style={{ ...PANEL, display: 'flex', flexDirection: 'column' }}>
      {/* Header */}
      <div style={{
        padding: '5px 10px', borderBottom: '1px solid #1e3a5f',
        fontSize: 10, fontWeight: 700, color: '#00b4d8', letterSpacing: 2,
      }}>
        MISSION METRICS
      </div>

      {/* Cost saved — large prominent */}
      <div style={{
        padding: '8px 12px',
        borderBottom: '1px solid #0e2240',
        background: '#06d6a008',
      }}>
        <div style={{ fontSize: 8, color: '#3a6186', letterSpacing: 1, marginBottom: 3 }}>
          COST SAVED VS. KINETIC STRIKE
        </div>
        <div style={{ fontSize: 30, fontWeight: 700, color: '#06d6a0', lineHeight: 1 }}>
          {costStr}
        </div>
        <div style={{ fontSize: 8, color: '#3a6186', marginTop: 2 }}>
          @ $950K saved per intercept
        </div>
      </div>

      {/* Counters row */}
      <div style={{ display: 'flex', borderBottom: '1px solid #0e2240' }}>
        <MetricBox label="WAVE"         value={status.wave_number}        color="#00b4d8" />
        <MetricBox label="DETECTED"     value={status.drones_detected}    color="#c9d6e3" />
        <MetricBox label="NEUTRALIZED"  value={status.drones_neutralized} color="#06d6a0" />
        <MetricBox
          label="BREACHED"
          value={status.drones_breached}
          color={status.drones_breached > 0 ? '#ef233c' : '#3a6186'}
        />
      </div>

      {/* Response time */}
      <div style={{ padding: '6px 12px', display: 'flex', alignItems: 'center', gap: 12 }}>
        <div>
          <div style={{ fontSize: 8, color: '#3a6186', letterSpacing: 1 }}>AVG RESPONSE</div>
          <span style={{ fontSize: 18, fontWeight: 600, color: rtColor }}>
            {rt > 0 ? rt.toFixed(1) : '—'}
          </span>
          <span style={{ fontSize: 9, color: '#3a6186', marginLeft: 3 }}>ms</span>
        </div>
        <div style={{
          flex: 1, height: 4, background: '#0e2240', position: 'relative',
        }}>
          {rt > 0 && (
            <div style={{
              position: 'absolute', left: 0, top: 0, bottom: 0,
              width: `${Math.min(100, rt / 5)}%`,
              background: rtColor,
            }} />
          )}
        </div>
        <div style={{ fontSize: 8, color: '#3a6186' }}>TARGET &lt;100ms</div>
      </div>
    </div>
  );
}
