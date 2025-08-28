import React, { useContext, useState, useEffect } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import { FaBroadcastTower, FaPlane, FaCarBattery, FaLock, FaLockOpen } from 'react-icons/fa';

const StatusBar = () => {
  const { ros, isConnected } = useContext(RosContext);
  // Simplified state, as we get individual topics now
  const [isArmed, setIsArmed] = useState(false);
  const [flightMode, setFlightMode] = useState('UNKNOWN');
  const [batteryState, setBatteryState] = useState({ voltage: 0, percentage: 0 });

  useEffect(() => {
    // Reset state on disconnect
    if (!isConnected || !ros) {
        setIsArmed(false);
        setFlightMode('UNKNOWN');
        setBatteryState({ voltage: 0, percentage: 0 });
        return;
    }

    const listeners = [];

    // --- Subscriber for Armed Status ---
    const armedListener = new ROSLIB.Topic({
      ros: ros,
      name: '/fc/armed_status', // Changed topic
      messageType: 'std_msgs/Bool',
    });
    armedListener.subscribe((message) => {
      setIsArmed(message.data);
    });
    listeners.push(armedListener);

    // --- Subscriber for Flight Mode ---
    const modeListener = new ROSLIB.Topic({
      ros: ros,
      name: '/fc/flight_mode', // Changed topic
      messageType: 'std_msgs/String',
    });
    modeListener.subscribe((message) => {
      setFlightMode(message.data);
    });
    listeners.push(modeListener);


    // --- Subscriber for Battery State ---
    const batteryListener = new ROSLIB.Topic({
      ros: ros,
      name: '/fc/battery', // Changed topic
      messageType: 'sensor_msgs/BatteryState',
    });
    batteryListener.subscribe((message) => {
      setBatteryState({
        voltage: message.voltage,
        percentage: message.percentage * 100,
      });
    });
    listeners.push(batteryListener);

    return () => {
      listeners.forEach(listener => listener.unsubscribe());
    };
  }, [isConnected, ros]);

  const getBatteryColor = (percentage) => {
    if (percentage > 50) return '#4ade80';
    if (percentage > 20) return '#facc15';
    return '#f87171';
  };

  return (
    <div style={{
      display: 'flex',
      flexWrap: 'wrap',
      alignItems: 'center',
      justifyContent: 'center',
      padding: '0.75rem 1.5rem',
      background: 'rgba(10, 12, 24, 0.6)',
      backdropFilter: 'blur(8px)',
      borderBottom: '1.5px solid rgba(255,255,255,0.07)',
      color: '#e6e9f0',
      fontSize: '0.95rem',
      position: 'sticky',
      top: '95px',
      zIndex: 90,
      gap: '1.5rem'
    }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.7rem' }}>
        <FaBroadcastTower style={{ color: isConnected ? '#4ade80' : '#f87171', fontSize: '1.2em' }}/>
        <strong style={{color: '#9ca3af'}}>ROS:</strong>
        <span style={{height: '12px', width: '12px', backgroundColor: isConnected ? '#4ade80' : '#f87171', borderRadius: '50%', display: 'inline-block', marginRight: '8px', border: '1px solid rgba(255,255,255,0.2)'}} />
        <span>{isConnected ? 'Connected' : 'Disconnected'}</span>
      </div>

      <div style={{width: '1px', height: '24px', background: 'rgba(255,255,255,0.1)'}}></div>
      
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.7rem' }}>
        {isArmed ? <FaLockOpen style={{color: '#4ade80', fontSize: '1.2em'}}/> : <FaLock style={{color: '#f87171', fontSize: '1.2em'}}/>}
        <strong style={{color: '#9ca3af'}}>Status:</strong>
        <span style={{ fontWeight: 'bold', color: isArmed ? '#4ade80' : '#f87171' }}>
          {isArmed ? 'ARMED' : 'DISARMED'}
        </span>
      </div>
      
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.7rem' }}>
        <strong style={{color: '#9ca3af'}}>Mode:</strong>
        <span style={{ background: 'rgba(255,255,255,0.1)', padding: '2px 10px', borderRadius: '6px', fontWeight: 'bold', fontFamily: 'monospace' }}>
            {flightMode}
        </span>
      </div>
      
      <div style={{width: '1px', height: '24px', background: 'rgba(255,255,255,0.1)'}}></div>

      <div style={{ display: 'flex', alignItems: 'center', gap: '0.7rem' }}>
        <FaCarBattery style={{ color: getBatteryColor(batteryState.percentage), fontSize: '1.2em' }}/>
        <strong style={{color: '#9ca3af'}}>Battery:</strong>
        <span style={{ fontWeight: 'bold', fontSize: '1.1em', color: getBatteryColor(batteryState.percentage) }}>
            {batteryState.percentage > 0 ? `${batteryState.percentage.toFixed(1)}%` : 'N/A'}
        </span>
        <span style={{ color: '#9ca3af', fontSize: '0.9rem' }}>
            {batteryState.voltage > 0 ? `(${batteryState.voltage.toFixed(2)}V)`: ''}
        </span>
      </div>
    </div>
  );
};

export default StatusBar;
