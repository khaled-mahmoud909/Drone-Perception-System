import React, { useContext, useState, useEffect, useRef, useCallback } from 'react';
import { FaImage, FaEye, FaPalette, FaTimes, FaCog, FaExclamationTriangle } from 'react-icons/fa';

import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';

const DepthImageViewer = ({ topicName, title }) => {
  const { ros, isConnected } = useContext(RosContext);
  const [rawImageData, setRawImageData] = useState(null);
  const [processedImageData, setProcessedImageData] = useState(null);
  const [depthStats, setDepthStats] = useState({ min: 0, max: 0, mean: 0 });
  const [colormap, setColormap] = useState('jet');
  const [showSettings, setShowSettings] = useState(false);
  const [minRange, setMinRange] = useState(0);
  const [maxRange, setMaxRange] = useState(10); // meters
  const [mousePos, setMousePos] = useState({ x: 0, y: 0, depth: 0 });
  const [showCrosshair, setShowCrosshair] = useState(true);
  const [imageFormat, setImageFormat] = useState('png');
  const [error, setError] = useState(null);
  const [debugInfo, setDebugInfo] = useState('');
  
  const canvasRef = useRef(null);
  const imageRef = useRef(null);
  const depthDataRef = useRef(null);

  // Colormap definitions
  const colormaps = {
    jet: (normalized) => {
      const r = Math.max(0, Math.min(255, 255 * Math.min(4 * normalized - 1.5, -4 * normalized + 4.5)));
      const g = Math.max(0, Math.min(255, 255 * Math.min(4 * normalized - 0.5, -4 * normalized + 3.5)));
      const b = Math.max(0, Math.min(255, 255 * Math.min(4 * normalized + 0.5, -4 * normalized + 2.5)));
      return [r, g, b];
    },
    viridis: (normalized) => {
      const r = 255 * (0.267 + 0.392 * normalized - 0.685 * normalized * normalized + 0.826 * Math.pow(normalized, 3));
      const g = 255 * (0.004 + 1.423 * normalized - 1.154 * normalized * normalized + 0.327 * Math.pow(normalized, 3));
      const b = 255 * (0.329 + 1.508 * normalized - 3.815 * normalized * normalized + 2.478 * Math.pow(normalized, 3));
      return [Math.max(0, Math.min(255, r)), Math.max(0, Math.min(255, g)), Math.max(0, Math.min(255, b))];
    },
    grayscale: (normalized) => {
      const val = 255 * normalized;
      return [val, val, val];
    },
    hot: (normalized) => {
      const r = Math.min(255, 255 * 3 * normalized);
      const g = Math.max(0, Math.min(255, 255 * (3 * normalized - 1)));
      const b = Math.max(0, Math.min(255, 255 * (3 * normalized - 2)));
      return [r, g, b];
    }
  };

  const processDepthImage = useCallback((imageData, format) => {
    console.log('Starting depth image processing:', { format, imageDataLength: imageData.length });
    setError(null);
    
    if (!imageData) {
      console.error('Image data not available');
      setError('Image data not available');
      return;
    }

    const canvas = canvasRef.current;
    if (!canvas) {
      console.error('Canvas ref not available');
      setError('Canvas not available');
      return;
    }
    
    const ctx = canvas.getContext('2d');
    
    const tempImg = new Image();
    
    tempImg.onerror = (error) => {
      console.error('Failed to load depth image:', error);
      setError(`Failed to load image: ${error.message || 'Unknown error'}`);
    };
    
    tempImg.onload = () => {
      try {
        // Don't set fixed canvas dimensions - let it resize with the actual image
        canvas.width = tempImg.width;
        canvas.height = tempImg.height;
        
        // Draw the original image to get pixel data
        ctx.drawImage(tempImg, 0, 0);
        const imageDataObj = ctx.getImageData(0, 0, canvas.width, canvas.height);
        const data = imageDataObj.data;
        
        console.log('Image data extracted:', { 
          width: canvas.width, 
          height: canvas.height, 
          dataLength: data.length 
        });
        
        // Process depth values based on format
        const depthValues = [];
        const width = canvas.width;
        const height = canvas.height;
        
        if (format === 'compressedDepth') {
          console.log('Processing compressed depth format');
          
          // For compressedDepth format, depth values are encoded in 16-bit format
          // The encoding depends on the compression method used
          for (let i = 0; i < data.length; i += 4) {
            const r = data[i];     // Red channel
            const g = data[i + 1]; // Green channel
            const b = data[i + 2]; // Blue channel
            const a = data[i + 3]; // Alpha channel
            
            // Standard 16-bit reconstruction for compressed depth
            let depth16 = (r << 8) | g;
            
            // Convert to meters - adjust scaling based on your camera
            let depthMeters;
            
            if (depth16 === 0 || a === 0) {
              depthMeters = 0; // Invalid/no depth
            } else {
              // For most depth cameras, raw value is in millimeters
              depthMeters = depth16 / 1000.0; // Millimeters to meters
              
              // Clamp unrealistic values
              if (depthMeters > 50 || depthMeters < 0.01) {
                depthMeters = 0; // Treat as invalid
              }
            }
            
            depthValues.push(depthMeters);
          }
        } else {
          console.log('Processing regular depth format');
          // For regular grayscale depth images
          for (let i = 0; i < data.length; i += 4) {
            const grayValue = data[i]; // Use red channel
            const depthMeters = (grayValue / 255.0) * maxRange;
            depthValues.push(depthMeters);
          }
        }
        
        console.log('Depth values processed:', {
          totalPixels: depthValues.length,
          sampleValues: depthValues.slice(0, 10),
          nonZeroCount: depthValues.filter(d => d > 0).length
        });
        
        // Store depth data for mouse hover
        depthDataRef.current = {
          values: depthValues,
          width: width,
          height: height
        };
        
        // Calculate statistics
        const validDepths = depthValues.filter(d => d > 0 && d < 50);
        console.log('Valid depths found:', validDepths.length);
        
        if (validDepths.length === 0) {
          console.warn('No valid depth data found');
          setError('No valid depth data found. Check depth encoding format.');
          setDebugInfo(`Total pixels: ${depthValues.length}, Non-zero: ${depthValues.filter(d => d > 0).length}`);
          return;
        }
        
        const min = Math.min(...validDepths);
        const max = Math.max(...validDepths);
        const mean = validDepths.reduce((a, b) => a + b, 0) / validDepths.length;
        
        console.log('Depth statistics:', { min, max, mean });
        setDepthStats({ min, max, mean });
        
        // Apply colormap with better handling for missing pixels
        const newImageData = ctx.createImageData(canvas.width, canvas.height);
        const newData = newImageData.data;
        
        let coloredPixels = 0;
        for (let i = 0; i < data.length; i += 4) {
          const pixelIndex = i / 4;
          const depth = depthValues[pixelIndex];
          
          let normalized = 0;
          let isValidPixel = false;
          
          if (depth > 0 && depth >= minRange && depth <= maxRange) {
            normalized = (depth - minRange) / (maxRange - minRange);
            normalized = Math.max(0, Math.min(1, normalized));
            isValidPixel = true;
            coloredPixels++;
          }
          
          if (isValidPixel) {
            const [r, g, b] = colormaps[colormap](normalized);
            newData[i] = r;     // Red
            newData[i + 1] = g; // Green
            newData[i + 2] = b; // Blue
            newData[i + 3] = 255; // Alpha
          } else {
            // For invalid pixels, use a subtle background color
            newData[i] = 20;     // Dark red
            newData[i + 1] = 20; // Dark green  
            newData[i + 2] = 30; // Slightly blue
            newData[i + 3] = 100; // Semi-transparent
          }
        }
        
        console.log('Colored pixels:', coloredPixels);
        
        ctx.putImageData(newImageData, 0, 0);
        const processedDataUrl = canvas.toDataURL();
        setProcessedImageData(processedDataUrl);
        
        setDebugInfo(`Processed ${coloredPixels}/${depthValues.length} pixels`);
        console.log('Depth image processing completed successfully');
        
      } catch (err) {
        console.error('Error processing depth image:', err);
        setError(`Processing error: ${err.message}`);
      }
    };
    
    tempImg.src = imageData;
  }, [colormap, minRange, maxRange]);

  // Handle mouse movement for depth reading - Fixed coordinate mapping
  const handleMouseMove = useCallback((e) => {
    if (!imageRef.current || !depthDataRef.current) return;
    
    const imgRect = imageRef.current.getBoundingClientRect();
    const { values, width, height } = depthDataRef.current;
    
    // Get mouse position relative to the image container
    const containerRect = e.currentTarget.getBoundingClientRect();
    const mouseX = e.clientX - containerRect.left;
    const mouseY = e.clientY - containerRect.top;
    
    // Calculate position relative to the actual image (accounting for object-fit: contain)
    const imgNaturalWidth = width;
    const imgNaturalHeight = height;
    const containerWidth = containerRect.width;
    const containerHeight = containerRect.height;
    
    // Calculate the scale factor for object-fit: contain
    const scaleX = containerWidth / imgNaturalWidth;
    const scaleY = containerHeight / imgNaturalHeight;
    const scale = Math.min(scaleX, scaleY);
    
    // Calculate the actual displayed image dimensions
    const displayedWidth = imgNaturalWidth * scale;
    const displayedHeight = imgNaturalHeight * scale;
    
    // Calculate the offset to center the image
    const offsetX = (containerWidth - displayedWidth) / 2;
    const offsetY = (containerHeight - displayedHeight) / 2;
    
    // Map mouse coordinates to image coordinates
    const imageX = (mouseX - offsetX) / scale;
    const imageY = (mouseY - offsetY) / scale;
    
    // Convert to pixel coordinates
    const x = Math.floor(Math.max(0, Math.min(imageX, imgNaturalWidth - 1)));
    const y = Math.floor(Math.max(0, Math.min(imageY, imgNaturalHeight - 1)));
    
    if (x >= 0 && x < width && y >= 0 && y < height) {
      const pixelIndex = y * width + x;
      const depthValue = values[pixelIndex] || 0;
      
      setMousePos({ 
        x: mouseX, 
        y: mouseY, 
        depth: depthValue.toFixed(3) 
      });
    }
  }, []);

  useEffect(() => {
    setRawImageData(null);
    setProcessedImageData(null);
    setError(null);
    setDebugInfo('');
    depthDataRef.current = null;
    
    if (!isConnected || !ros) {
      console.log('ROS not connected');
      return;
    }

    console.log('Subscribing to depth topic:', topicName);

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: 'sensor_msgs/CompressedImage',
    });

    let lastUpdateTime = 0;
    const updateInterval = 100;

    listener.subscribe((message) => {
      console.log('Received depth message:', {
        format: message.format,
        dataLength: message.data.length,
        header: message.header
      });

      const now = Date.now();
      if (now - lastUpdateTime < updateInterval) {
        return;
      }
      lastUpdateTime = now;

      let imageDataString = message.data;
      let format = 'regular';

      // Handle compressedDepth format
      if (message.format.includes('compressedDepth')) {
        console.log('Detected compressedDepth format');
        format = 'compressedDepth';
        
        console.log('Original data length:', imageDataString.length);
        console.log('First 32 chars of data:', imageDataString.substring(0, 32));
        
        // Remove the 12-byte header (16 base64 characters)
        // The ROS message shows 12 zero bytes followed by PNG data
        if (imageDataString.startsWith('AAAAAAAAAAAAAAAA')) {
          console.log('Removing 12-byte header (16 base64 chars)');
          imageDataString = imageDataString.substring(16);
          console.log('After header removal - first 32 chars:', imageDataString.substring(0, 32));
        }
      }
      
      const imageFormat = message.format.includes('jpeg') ? 'jpeg' : 'png';
      const dataUrl = `data:image/${imageFormat};base64,${imageDataString}`;
      
      console.log('Setting raw image data:', {
        format: imageFormat,
        dataUrlLength: dataUrl.length,
        processingFormat: format
      });
      
      setImageFormat(format);
      setRawImageData(dataUrl);
    });

    return () => {
      console.log('Unsubscribing from depth topic');
      listener.unsubscribe();
    };
  }, [isConnected, ros, topicName]);

  useEffect(() => {
    if (rawImageData && canvasRef.current) {
      console.log('Processing depth image with format:', imageFormat, 'Canvas available:', !!canvasRef.current);
      processDepthImage(rawImageData, imageFormat);
    } else if (rawImageData) {
      console.log('Raw image data available but canvas not ready yet');
    }
  }, [rawImageData, processDepthImage, imageFormat]);

  // Trigger processing when canvas becomes available
  useEffect(() => {
    if (canvasRef.current && rawImageData) {
      console.log('Canvas ref is now available, processing pending image data');
      processDepthImage(rawImageData, imageFormat);
    }
  }, [canvasRef.current, rawImageData, imageFormat, processDepthImage]);

  const containerStyle = {
    width: '100%',
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
    background: 'rgba(0, 0, 0, 0.2)',
    overflow: 'hidden',
    position: 'relative'
  };

  const controlsStyle = {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '0.5rem',
    background: 'rgba(0, 0, 0, 0.3)',
    borderBottom: '1px solid rgba(255,255,255,0.1)',
    fontSize: '0.8rem',
    flexShrink: 0
  };

  const imageContainerStyle = {
    flex: 1,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    position: 'relative',
    overflow: 'hidden'
  };

  const imageStyle = {
    width: '100%',
    height: '100%',
    objectFit: 'contain',
    display: 'block',
    borderRadius: '4px'
  };

  const settingsStyle = {
    position: 'absolute',
    top: '10px',
    right: '10px',
    background: 'rgba(30, 34, 54, 0.95)',
    borderRadius: '8px',
    padding: '1rem',
    minWidth: '200px',
    border: '1px solid rgba(255,255,255,0.1)',
    zIndex: 10
  };

  const crosshairStyle = {
    position: 'absolute',
    pointerEvents: 'none',
    zIndex: 5
  };

  const placeholderStyle = {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    color: '#9ca3af',
    textAlign: 'center',
    fontSize: '0.9rem',
    padding: '1rem'
  };

  const errorStyle = {
    ...placeholderStyle,
    color: '#ff5c7c'
  };

  return (
    <div style={containerStyle}>
      {/* Controls */}
      <div style={controlsStyle}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
          <div>
            Min: {depthStats.min.toFixed(2)}m | 
            Max: {depthStats.max.toFixed(2)}m | 
            Avg: {depthStats.mean.toFixed(2)}m
          </div>
          {processedImageData && (
            <div style={{ color: '#a855f7' }}>
              Depth: {mousePos.depth}m
            </div>
          )}
          {debugInfo && (
            <div style={{ color: '#fbbf24', fontSize: '0.7rem' }}>
              {debugInfo}
            </div>
          )}
        </div>
        
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <button
            onClick={() => setShowCrosshair(!showCrosshair)}
            style={{
              background: showCrosshair ? 'rgba(124, 58, 237, 0.3)' : 'transparent',
              border: '1px solid rgba(255,255,255,0.2)',
              borderRadius: '4px',
              padding: '4px 8px',
              color: '#e6e9f0',
              cursor: 'pointer',
              fontSize: '0.8rem'
            }}
            title="Toggle crosshair"
          >
            <FaEye />
          </button>
          
          <button
            onClick={() => setShowSettings(!showSettings)}
            style={{
              background: showSettings ? 'rgba(124, 58, 237, 0.3)' : 'transparent',
              border: '1px solid rgba(255,255,255,0.2)',
              borderRadius: '4px',
              padding: '4px 8px',
              color: '#e6e9f0',
              cursor: 'pointer',
              fontSize: '0.8rem'
            }}
            title="Settings"
          >
            <FaCog />
          </button>
        </div>
      </div>

      {/* Settings Panel */}
      {showSettings && (
        <div style={settingsStyle}>
          <div style={{ 
            display: 'flex', 
            justifyContent: 'space-between', 
            alignItems: 'center',
            marginBottom: '1rem' 
          }}>
            <div style={{ fontWeight: 'bold', color: '#e6e9f0' }}>
              <FaPalette style={{ marginRight: '0.5rem' }} />
              Depth Visualization
            </div>
            <button
              onClick={() => setShowSettings(false)}
              style={{
                background: 'none',
                border: 'none',
                color: '#b0b3c6',
                fontSize: '1.2rem',
                cursor: 'pointer',
                padding: '0.2rem'
              }}
              title="Close settings"
            >
              <FaTimes />
            </button>
          </div>
          
          <div style={{ marginBottom: '0.8rem' }}>
            <label style={{ display: 'block', marginBottom: '0.3rem', fontSize: '0.85rem', color: '#b0b3c6' }}>
              Colormap:
            </label>
            <select
              value={colormap}
              onChange={(e) => setColormap(e.target.value)}
              style={{
                width: '100%',
                padding: '0.3rem',
                background: 'rgba(0,0,0,0.3)',
                border: '1px solid rgba(255,255,255,0.2)',
                borderRadius: '4px',
                color: '#e6e9f0',
                fontSize: '0.85rem'
              }}
            >
              <option value="jet">Jet</option>
              <option value="viridis">Viridis</option>
              <option value="grayscale">Grayscale</option>
              <option value="hot">Hot</option>
            </select>
          </div>

          <div style={{ marginBottom: '0.8rem' }}>
            <label style={{ display: 'block', marginBottom: '0.3rem', fontSize: '0.85rem', color: '#b0b3c6' }}>
              Min Range (m): {minRange}
            </label>
            <input
              type="range"
              min="0"
              max="5"
              step="0.1"
              value={minRange}
              onChange={(e) => setMinRange(parseFloat(e.target.value))}
              style={{ width: '100%' }}
            />
          </div>

          <div>
            <label style={{ display: 'block', marginBottom: '0.3rem', fontSize: '0.85rem', color: '#b0b3c6' }}>
              Max Range (m): {maxRange}
            </label>
            <input
              type="range"
              min="1"
              max="50"
              step="0.5"
              value={maxRange}
              onChange={(e) => setMaxRange(parseFloat(e.target.value))}
              style={{ width: '100%' }}
            />
          </div>
        </div>
      )}

      {/* Image Display */}
      <div style={imageContainerStyle}>
        {/* Hidden canvas for processing - always render */}
        <canvas 
          ref={canvasRef} 
          style={{ display: 'none' }}
          width={640}
          height={480}
        />
        
        {error ? (
          <div style={errorStyle}>
            <FaExclamationTriangle style={{ fontSize: '3rem', marginBottom: '1rem', opacity: 0.5 }} />
            <p style={{ margin: 0, marginBottom: '0.5rem' }}>
              Error processing depth image:
            </p>
            <p style={{ margin: 0, fontSize: '0.8rem', color: '#fbbf24' }}>
              {error}
            </p>
            <p style={{ margin: 0, marginTop: '1rem', fontSize: '0.7rem', opacity: 0.7 }}>
              Topic: <code>{topicName}</code>
            </p>
          </div>
        ) : processedImageData ? (
          <div 
            style={{ 
              position: 'relative', 
              width: '100%', 
              height: '100%',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center'
            }}
            onMouseMove={handleMouseMove}
          >
            <img 
              ref={imageRef}
              src={processedImageData} 
              alt={title} 
              style={imageStyle}
            />
            
            {/* Crosshair */}
            {showCrosshair && (
              <div style={{ 
                ...crosshairStyle, 
                left: mousePos.x, 
                top: mousePos.y,
                transform: 'translate(-50%, -50%)'
              }}>
                <div style={{
                  position: 'absolute',
                  width: '20px',
                  height: '1px',
                  background: '#ff5c7c',
                  left: '-10px',
                  top: '0px'
                }} />
                <div style={{
                  position: 'absolute',
                  width: '1px',
                  height: '20px',
                  background: '#ff5c7c',
                  left: '0px',
                  top: '-10px'
                }} />
              </div>
            )}
          </div>
        ) : (
          <div style={placeholderStyle}>
            <FaImage style={{ fontSize: '3rem', marginBottom: '1rem', opacity: 0.5 }} />
            <p style={{ margin: 0 }}>
              Processing depth data from: <br/>
              <code style={{ background: 'rgba(255,255,255,0.1)', padding: '2px 4px', borderRadius: '4px' }}>
                {topicName}
              </code>
            </p>
            {debugInfo && (
              <p style={{ margin: '0.5rem 0 0 0', fontSize: '0.7rem', color: '#fbbf24' }}>
                {debugInfo}
              </p>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default DepthImageViewer;