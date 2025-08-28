// src/components/DroneState.jsx
import React, { useContext, useState, useEffect } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';

const DroneState = () => {
  const { ros, isConnected } = useContext(RosContext);
  const [battery, setBattery] = useState(null);
  // Add more states as needed: altitude, flight mode, etc.

  useEffect(() => {
    if (!isConnected || !ros) return;

    // --- Subscriber for Battery State ---
    const batteryListener = new ROSLIB.Topic({
      ros: ros,
      name: '/mavros/battery', // Change to your battery topic
      messageType: 'sensor_msgs/BatteryState',
    });

    batteryListener.subscribe((message) => {
      setBattery(message.percentage * 100);
    });

    return () => {
      batteryListener.unsubscribe();
    };
  }, [isConnected, ros]);

  if (!isConnected) {
    return <div>Connecting to Drone...</div>;
  }

  return (
    <div className="drone-state-panel">
      <h3>Drone Status</h3>
      <p>Battery: {battery ? `${battery.toFixed(2)}%` : 'N/A'}</p>
      {/* Add more state displays here */}
    </div>
  );
};

export default DroneState;