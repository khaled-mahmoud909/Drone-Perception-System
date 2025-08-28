'use client';
import React from 'react';
import { useContext, useState, useEffect } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import { FaImage } from 'react-icons/fa'; // Import an icon for the placeholder

// We keep the `title` prop for the `alt` attribute, but won't render it as a header.
const ImageViewer = ({ topicName, title }) => {
  const { ros, isConnected } = useContext(RosContext);
  const [imageData, setImageData] = useState(null);

  useEffect(() => {
    // Reset image data if connection or topic changes, ensures a clean state.
    setImageData(null); 
    if (!isConnected || !ros) return;

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: 'sensor_msgs/CompressedImage',
      // Optional: Throttle messages at the bridge level if needed for performance.
      // throttle_rate: 100 // (in milliseconds)
    });

    let lastUpdateTime = 0;
    // Prevent state updates from choking the browser on high-frequency topics.
    // 33ms is ~30fps, a good balance for visual updates without UI lag.
    const updateInterval = 33; 

    listener.subscribe((message) => {
      const now = Date.now();
      const arrivalTime = new Date().getTime() / 1000.0; // Time in seconds
      const sentTime = message.header.stamp.secs + message.header.stamp.nsecs / 1e9;
      const latency = arrivalTime - sentTime;
      console.log(`Image Latency: ${latency.toFixed(4)}s`);
      if (now - lastUpdateTime < updateInterval) {
        return; // Skip this frame
      }
      lastUpdateTime = now;

      let imageDataString = message.data;

      // This logic is correct for the common `compressedDepth` format which
      // has a 12-byte header (16 characters in base64).
      if (message.format.includes('compressedDepth')) {
        imageDataString = message.data.substring(16);
      }
      
      const imageFormat = message.format.includes('jpeg') ? 'jpeg' : 'png';
      setImageData(`data:image/${imageFormat};base64,${imageDataString}`);
    });

    return () => {
      listener.unsubscribe();
    };
  }, [isConnected, ros, topicName]);

  // Styles for the main container
  const containerStyle = {
    width: '100%',
    height: '100%',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    background: 'rgba(0, 0, 0, 0.2)',
    overflow: 'hidden', // Crucial for `object-fit` to work as expected
    padding: '0.5rem', // Add a little padding inside the panel
    boxSizing: 'border-box'
  };

  // Styles for the image to make it responsive
  const imageStyle = {
    width: '100%',
    height: '100%',
    objectFit: 'contain', // This is the key: keeps aspect ratio, fits within the box
    display: 'block',     // Removes potential bottom space under the image
    borderRadius: '8px'   // A little visual polish
  };
  
  // Styles for the placeholder when no image is available
  const placeholderStyle = {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    color: '#9ca3af',
    textAlign: 'center',
    fontSize: '0.9rem',
  };

  return (
    <div style={containerStyle}>
      {imageData ? (
        <img src={imageData} alt={title} style={imageStyle} />
      ) : (
        <div style={placeholderStyle}>
          <FaImage style={{ fontSize: '3rem', marginBottom: '1rem', opacity: 0.5 }} />
          <p style={{margin: 0}}>Waiting for topic: <br/> <code style={{background: 'rgba(255,255,255,0.1)', padding: '2px 4px', borderRadius: '4px'}}>{topicName}</code></p>
        </div>
      )}
    </div>
  );
};

export default ImageViewer;