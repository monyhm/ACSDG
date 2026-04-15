/**
 * App.js — ACSDG C4ISR Platform layout.
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │                      TOP BAR (36px)                         │
 * ├──────────────────────────┬───────────────────────────────────┤
 * │                          │  ThreatPanel                      │
 * │     TacticalMap          │  MetricsPanel                     │
 * │      (60% width)         │  SwarmPanel                       │
 * │                          │  InterceptorPanel                 │
 * ├──────────────────────────┴───────────────────────────────────┤
 * │                  LearningPanel (40px)                       │
 * └──────────────────────────────────────────────────────────────┘
 */

import React, { useState, useEffect } from 'react';
import TacticalMap      from './components/TacticalMap';
import ThreatPanel      from './components/ThreatPanel';
import MetricsPanel     from './components/MetricsPanel';
import SwarmPanel       from './components/SwarmPanel';
import InterceptorPanel from './components/InterceptorPanel';
import LearningPanel    from './components/LearningPanel';
import { onStatusChange } from './ros/RosConnection';

const TOP_H    = 36;
const BOTTOM_H = 40;
const BORDER   = '#1e3a5f';
const MONO     = "'IBM Plex Mono', monospace";
const SANS     = "'IBM Plex Sans Condensed', sans-serif";

function TopBar({ rosStatus }) {
  const [time, setTime] = useState(new Date());
  useEffect(() => {
    const id = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(id);
  }, []);

  const statusColor = rosStatus === 'CONNECTED'    ? '#06d6a0'
                    : rosStatus === 'ERROR'         ? '#ef233c'
                    : '#f4a261';

  return (
    <div style={{
      height: TOP_H,
      background: '#040810',
      borderBottom: `1px solid ${BORDER}`,
      display: 'flex',
      alignItems: 'center',
      padding: '0 14px',
      gap: 20,
      flexShrink: 0,
    }}>
      {/* Logo / title */}
      <div style={{ display: 'flex', alignItems: 'baseline', gap: 10 }}>
        <span style={{
          fontFamily: MONO, fontSize: 13, fontWeight: 700,
          color: '#00b4d8', letterSpacing: 3,
        }}>
          ACSDG
        </span>
        <span style={{
          fontFamily: SANS, fontSize: 11, fontWeight: 600,
          color: '#8d99ae', letterSpacing: 2,
        }}>
          C4ISR PLATFORM
        </span>
      </div>

      <div style={{ width: 1, height: 20, background: BORDER }} />

      <span style={{
        fontFamily: SANS, fontSize: 10, color: '#3a6186', letterSpacing: 1,
      }}>
        FACILITY: ALPHA-7 · OPERATIONAL
      </span>

      {/* Spacer */}
      <div style={{ flex: 1 }} />

      {/* ROS status */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
        <div style={{
          width: 6, height: 6,
          background: statusColor,
          boxShadow: `0 0 5px ${statusColor}`,
        }} />
        <span style={{
          fontFamily: MONO, fontSize: 9, color: statusColor, letterSpacing: 1,
        }}>
          ROS {rosStatus}
        </span>
      </div>

      <div style={{ width: 1, height: 20, background: BORDER }} />

      {/* Clock */}
      <span style={{
        fontFamily: MONO, fontSize: 11, color: '#00b4d8',
        letterSpacing: 1, minWidth: 80, textAlign: 'right',
      }}>
        {time.toLocaleTimeString('en-GB')}
      </span>
    </div>
  );
}

export default function App() {
  const [rosStatus, setRosStatus] = useState('DISCONNECTED');

  useEffect(() => {
    const unreg = onStatusChange(setRosStatus);
    return unreg;
  }, []);

  const contentH = `calc(100vh - ${TOP_H + BOTTOM_H}px)`;

  return (
    <div style={{
      display: 'flex', flexDirection: 'column',
      width: '100vw', height: '100vh',
      background: '#0a0f1a', overflow: 'hidden',
    }}>
      <TopBar rosStatus={rosStatus} />

      {/* Main content */}
      <div style={{
        display: 'flex', flex: 1, height: contentH, overflow: 'hidden',
      }}>
        {/* Left: Tactical map */}
        <div style={{
          width: '60%', minWidth: 0,
          borderRight: `1px solid ${BORDER}`,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          background: '#060b14', overflow: 'hidden',
        }}>
          <TacticalMap />
        </div>

        {/* Right column */}
        <div style={{
          width: '40%', display: 'flex', flexDirection: 'column',
          overflow: 'hidden',
        }}>
          {/* Threat panel — top third */}
          <div style={{
            flex: 3,
            borderBottom: `1px solid ${BORDER}`,
            overflow: 'hidden', display: 'flex', flexDirection: 'column',
          }}>
            <ThreatPanel />
          </div>

          {/* Metrics panel */}
          <div style={{
            flex: 2,
            borderBottom: `1px solid ${BORDER}`,
          }}>
            <MetricsPanel />
          </div>

          {/* Swarm panel */}
          <div style={{
            flex: 2,
            borderBottom: `1px solid ${BORDER}`,
          }}>
            <SwarmPanel />
          </div>

          {/* Interceptor panel */}
          <div style={{ flex: 3, overflow: 'hidden' }}>
            <InterceptorPanel />
          </div>
        </div>
      </div>

      {/* Bottom learning bar */}
      <LearningPanel />
    </div>
  );
}
