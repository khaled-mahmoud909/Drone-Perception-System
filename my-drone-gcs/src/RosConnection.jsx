// src/RosConnection.js
import ROSLIB from 'roslib';
import React, { createContext, useState, useEffect } from 'react';

// Create a context to provide the ROS object to all components
export const RosContext = createContext();

export const RosProvider = ({ children }) => {
  const [ros, setRos] = useState(null);
  const [isConnected, setIsConnected] = useState(false);

  // You can make this configurable
  const ROSBRIDGE_URL = 'ws://192.168.100.234:9090';

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: ROSBRIDGE_URL,
    });

    rosInstance.on('connection', () => {
      console.log('Connected to websocket server.');
      setIsConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      setIsConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to websocket server closed.');
      setIsConnected(false);
      // Optional: try to reconnect
      setTimeout(() => {
        try {
          rosInstance.connect(ROSBRIDGE_URL);
        } catch (e) {
          console.error('Reconnect failed', e);
        }
      }, 1000);
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  return (
    <RosContext.Provider value={{ ros, isConnected }}>
      {children}
    </RosContext.Provider>
  );
};