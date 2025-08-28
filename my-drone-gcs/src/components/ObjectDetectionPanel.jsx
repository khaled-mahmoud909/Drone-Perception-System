import React, { useState, useEffect, useRef, useContext, useCallback } from 'react';
import { FaSearchLocation, FaCog, FaTimes, FaPlay, FaPause, FaRuler, FaCube, FaEye, FaBullseye } from 'react-icons/fa';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import * as tf from '@tensorflow/tfjs';
import * as cocoSsd from '@tensorflow-models/coco-ssd';

// Helper function to check if two detections likely refer to the same object.
const calculateIoU = (box1, box2) => {
  const [x1, y1, w1, h1] = box1;
  const [x2, y2, w2, h2] = box2;
  const xi1 = Math.max(x1, x2), yi1 = Math.max(y1, y2);
  const xi2 = Math.min(x1 + w1, x2 + w2), yi2 = Math.min(y1 + h1, y2 + h2);
  const interWidth = Math.max(0, xi2 - xi1), interHeight = Math.max(0, yi2 - yi1);
  const interArea = interWidth * interHeight;
  const box1Area = w1 * h1, box2Area = w2 * h2;
  const unionArea = box1Area + box2Area - interArea;
  return unionArea > 0 ? interArea / unionArea : 0;
};

const ObjectDetectionPanel = () => {
  const { ros, isConnected } = useContext(RosContext);
  // --- MODIFIED: This is now the single source of truth for all persistent detections.
  const [persistentDetections, setPersistentDetections] = useState([]);
  const [isProcessing, setIsProcessing] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [confidence, setConfidence] = useState(0.5);
  const [maxObjects, setMaxObjects] = useState(10);
  const [model, setModel] = useState(null);
  const [modelLoading, setModelLoading] = useState(false);
  const [modelError, setModelError] = useState(null);
  const [rgbImage, setRgbImage] = useState(null);
  const [depthData, setDepthData] = useState(null);
  const [processingEnabled, setProcessingEnabled] = useState(false);
  const [fps, setFps] = useState(0);
  const [sortBy, setSortBy] = useState('confidence');
  const [filterClass, setFilterClass] = useState('all');
  
  const rgbCanvasRef = useRef(null);
  const depthCanvasRef = useRef(null);
  const animationFrameRef = useRef(null);
  const fpsCounterRef = useRef({ frames: 0, lastTime: Date.now() });

  // --- NEW: Function to manually remove a detection ---
  const handleRemoveDetection = useCallback((idToRemove) => {
    setPersistentDetections(prev => prev.filter(d => d.id !== idToRemove));
  }, []);

  // --- SIMPLIFIED: Load only the COCO-SSD model ---
  const loadModel = useCallback(async () => {
    setModelLoading(true);
    setModelError(null);
    setModel(null);
    try {
      await tf.ready();
      console.log(`TensorFlow.js backend: ${tf.getBackend()}`);
      const loadedModel = await cocoSsd.load({ base: 'mobilenet_v1' });
      setModel(loadedModel);
      console.log(`COCO-SSD model loaded successfully`);
    } catch (error) {
      console.error('Error loading model:', error);
      setModelError(error.message);
    } finally {
      setModelLoading(false);
    }
  }, []);

  useEffect(() => {
    loadModel();
  }, [loadModel]);

  useEffect(() => {
    if (!isConnected || !ros) return;
    const rgbListener = new ROSLIB.Topic({ ros, name: '/oak/rgb/preview/image_raw/compressed', messageType: 'sensor_msgs/CompressedImage' });
    rgbListener.subscribe((message) => setRgbImage(`data:image/jpeg;base64,${message.data}`));
    return () => rgbListener.unsubscribe();
  }, [isConnected, ros]);

  useEffect(() => {
    if (!isConnected || !ros) return;
    const depthListener = new ROSLIB.Topic({ ros, name: '/oak/stereo/image_raw/compressedDepth', messageType: 'sensor_msgs/CompressedImage' });
    depthListener.subscribe((message) => {
      let imageDataString = message.data;
      if (message.format.includes('compressedDepth') && imageDataString.startsWith('AAAAAAAAAAAAAAAA')) imageDataString = imageDataString.substring(16);
      setDepthData(`data:image/png;base64,${imageDataString}`);
    });
    return () => depthListener.unsubscribe();
  }, [isConnected, ros]);

  const getDepthAtPixel = useCallback((x, y, canvas) => {
    if (!canvas) return 0;
    const ctx = canvas.getContext('2d');
    const data = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
    const pixelIndex = (Math.floor(y) * canvas.width + Math.floor(x)) * 4;
    if (pixelIndex >= 0 && pixelIndex < data.length) {
      const r = data[pixelIndex], g = data[pixelIndex + 1];
      const depth16 = (r << 8) | g;
      if (depth16 === 0) return 0;
      const depthMeters = depth16 / 1000.0;
      return (depthMeters > 50 || depthMeters < 0.01) ? 0 : depthMeters;
    }
    return 0;
  }, []);

  // --- REWRITTEN: processDetection now updates a persistent list ---
  const processDetection = useCallback(async () => {
    if (!model || !rgbImage || !processingEnabled || isProcessing) return;

    setIsProcessing(true);
    const MATCH_IOU_THRESHOLD = 0.5;

    try {
      const img = new Image();
      img.crossOrigin = 'anonymous';
      img.onload = async () => {
        const canvas = rgbCanvasRef.current;
        if (!canvas) { setIsProcessing(false); return; }
        
        canvas.width = img.width;
        canvas.height = img.height;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(img, 0, 0);

        let predictions = [];
        try {
          predictions = await model.detect(canvas, maxObjects);
        } catch (modelError) {
          console.error('Model inference error:', modelError);
          setIsProcessing(false);
          return;
        }

        const now = Date.now();
        let updatedDetections = [...persistentDetections];
        const matchedPredictionIndices = new Set();

        // 1. Try to update existing detections with new predictions
        updatedDetections.forEach(existing => {
          let bestMatchIndex = -1;
          let bestIoU = MATCH_IOU_THRESHOLD;
          
          for (let i = 0; i < predictions.length; i++) {
            if (matchedPredictionIndices.has(i)) continue;
            const pred = predictions[i];
            if (pred.class === existing.class) {
              const iou = calculateIoU(existing.bbox, pred.bbox);
              if (iou > bestIoU) {
                bestIoU = iou;
                bestMatchIndex = i;
              }
            }
          }
          
          if (bestMatchIndex !== -1) {
            const pred = predictions[bestMatchIndex];
            existing.bbox = pred.bbox;
            existing.score = pred.score;
            existing.timestamp = new Date().toLocaleTimeString();
            matchedPredictionIndices.add(bestMatchIndex);
          }
        });

        // 2. Add new, unmatched predictions to the list
        for (let i = 0; i < predictions.length; i++) {
          if (matchedPredictionIndices.has(i)) continue;
          const pred = predictions[i];
          if (pred.score >= confidence) {
            updatedDetections.push({
              ...pred,
              id: `${pred.class}_${now}_${Math.random()}`,
              timestamp: new Date().toLocaleTimeString(),
              color: `hsl(${(updatedDetections.length * 137.5) % 360}, 70%, 50%)`
            });
          }
        }
        
        // 3. Update display properties (like distance) for all items in the list
        const finalProcessedDetections = await Promise.all(
            updatedDetections.map(async (det) => {
                const [x, y, w, h] = det.bbox;
                const centerX = x + w / 2, centerY = y + h / 2;
                const distance = depthData && depthCanvasRef.current ? getDepthAtPixel(centerX, centerY, depthCanvasRef.current) : 0;
                return {
                    ...det,
                    distance: distance || det.distance, // Keep old distance if new one is 0
                    centerX: Math.round(centerX),
                    centerY: Math.round(centerY),
                    width: Math.round(w),
                    height: Math.round(h),
                };
            })
        );
        
        setPersistentDetections(finalProcessedDetections);

        // Update FPS counter
        fpsCounterRef.current.frames++;
        if (now - fpsCounterRef.current.lastTime >= 1000) {
            setFps(fpsCounterRef.current.frames);
            fpsCounterRef.current.frames = 0;
            fpsCounterRef.current.lastTime = now;
        }
        setIsProcessing(false);
      };
      img.onerror = () => setIsProcessing(false);
      img.src = rgbImage;
    } catch (error) {
      console.error('Detection error:', error);
      setIsProcessing(false);
    }
  }, [model, rgbImage, depthData, confidence, maxObjects, processingEnabled, isProcessing, persistentDetections, getDepthAtPixel]);

  useEffect(() => {
    if (depthData && depthCanvasRef.current) {
      const img = new Image();
      img.onload = () => {
        const canvas = depthCanvasRef.current;
        canvas.width = img.width; canvas.height = img.height;
        canvas.getContext('2d').drawImage(img, 0, 0);
      };
      img.src = depthData;
    }
  }, [depthData]);

  useEffect(() => {
    if (processingEnabled) {
      const processFrame = () => {
        if (!isProcessing) processDetection();
        animationFrameRef.current = requestAnimationFrame(processFrame);
      };
      animationFrameRef.current = requestAnimationFrame(processFrame);
    }
    return () => { if (animationFrameRef.current) cancelAnimationFrame(animationFrameRef.current); };
  }, [processDetection, processingEnabled, isProcessing]);

  // Derive the list to be rendered from the persistent state
  const uniqueClasses = [...new Set(persistentDetections.map(d => d.class))];
  const filteredDetections = persistentDetections
    .filter(detection => filterClass === 'all' || detection.class === filterClass)
    .sort((a, b) => {
      switch (sortBy) {
        case 'confidence': return b.score - a.score;
        case 'distance': return (a.distance || Infinity) - (b.distance || Infinity);
        case 'class': return a.class.localeCompare(b.class);
        default: return 0;
      }
    });

  const containerStyle = { width: '100%', height: '100%', display: 'flex', flexDirection: 'column', background: 'rgba(0, 0, 0, 0.2)', overflow: 'hidden', position: 'relative' };
  const controlsStyle = { display: 'flex', alignItems: 'center', justifyContent: 'space-between', padding: '0.5rem', background: 'rgba(0, 0, 0, 0.3)', borderBottom: '1px solid rgba(255,255,255,0.1)', fontSize: '0.8rem', flexShrink: 0 };
  const contentStyle = { flex: 1, display: 'flex', flexDirection: 'column', overflow: 'hidden' };
  const filtersStyle = { display: 'flex', alignItems: 'center', gap: '1rem', padding: '0.5rem', background: 'rgba(0, 0, 0, 0.2)', borderBottom: '1px solid rgba(255,255,255,0.1)', fontSize: '0.8rem', flexShrink: 0 };
  const detectionListStyle = { flex: 1, overflowY: 'auto', padding: '0.5rem' };
  const detectionItemStyle = {
    background: 'rgba(255, 255, 255, 0.05)',
    borderRadius: '8px',
    padding: '0.75rem',
    paddingBottom: '2rem', // More space at the bottom for the timestamp
    marginBottom: '0.5rem',
    border: '1px solid rgba(255,255,255,0.1)',
    transition: 'all 0.2s ease',
    position: 'relative' // Needed for absolute positioning of children
  };
  const settingsStyle = { position: 'absolute', top: '10px', right: '10px', background: 'rgba(30, 34, 54, 0.95)', borderRadius: '8px', padding: '1rem', minWidth: '250px', border: '1px solid rgba(255,255,255,0.1)', zIndex: 10, fontSize: '0.85rem' };
  
  return (
    <div style={containerStyle}>
      <canvas ref={rgbCanvasRef} style={{ display: 'none' }} />
      <canvas ref={depthCanvasRef} style={{ display: 'none' }} />
      <div style={controlsStyle}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
          <div style={{ color: processingEnabled ? '#4ade80' : '#9ca3af' }}><FaSearchLocation /> {persistentDetections.length} Objects</div>
          <div style={{ color: '#a855f7' }}>{fps} FPS</div>
          {modelLoading && <div style={{ color: '#fbbf24' }}>Loading model...</div>}
          {modelError && <div style={{ color: '#ff5c7c', fontSize: '0.7rem' }}>Error: {modelError}</div>}
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <button onClick={() => setProcessingEnabled(!processingEnabled)} disabled={!model || modelLoading} style={{ background: processingEnabled ? 'rgba(34, 197, 94, 0.3)' : 'rgba(239, 68, 68, 0.3)', border: '1px solid rgba(255,255,255,0.2)', borderRadius: '4px', padding: '4px 8px', color: '#e6e9f0', cursor: model && !modelLoading ? 'pointer' : 'not-allowed', fontSize: '0.8rem', opacity: model && !modelLoading ? 1 : 0.5 }} title={processingEnabled ? 'Stop detection' : 'Start detection'}>
            {processingEnabled ? <FaPause /> : <FaPlay />}
          </button>
          <button onClick={() => setShowSettings(!showSettings)} style={{ background: showSettings ? 'rgba(124, 58, 237, 0.3)' : 'transparent', border: '1px solid rgba(255,255,255,0.2)', borderRadius: '4px', padding: '4px 8px', color: '#e6e9f0', cursor: 'pointer', fontSize: '0.8rem' }} title="Settings"><FaCog /></button>
        </div>
      </div>
      <div style={filtersStyle}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <label style={{ color: '#b0b3c6' }}>Sort by:</label>
          <select value={sortBy} onChange={(e) => setSortBy(e.target.value)} style={{ padding: '0.2rem', background: 'rgba(0,0,0,0.3)', border: '1px solid rgba(255,255,255,0.2)', borderRadius: '4px', color: '#e6e9f0', fontSize: '0.8rem' }}><option value="confidence">Confidence</option><option value="distance">Distance</option><option value="class">Class</option></select>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <label style={{ color: '#b0b3c6' }}>Filter:</label>
          <select value={filterClass} onChange={(e) => setFilterClass(e.target.value)} style={{ padding: '0.2rem', background: 'rgba(0,0,0,0.3)', border: '1px solid rgba(255,255,255,0.2)', borderRadius: '4px', color: '#e6e9f0', fontSize: '0.8rem' }}>
            <option value="all">All Classes</option>
            {uniqueClasses.map(className => (<option key={className} value={className}>{className}</option>))}
          </select>
        </div>
        <button onClick={() => setPersistentDetections([])} style={{ background: 'rgba(239, 68, 68, 0.3)', border: '1px solid rgba(255,255,255,0.2)', borderRadius: '4px', padding: '4px 8px', color: '#e6e9f0', cursor: 'pointer', fontSize: '0.8rem' }} title="Clear all detections">Clear All</button>
      </div>
      {showSettings && (
        <div style={settingsStyle}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}><div style={{ fontWeight: 'bold', color: '#e6e9f0' }}>Detection Settings</div><button onClick={() => setShowSettings(false)} style={{ background: 'none', border: 'none', color: '#b0b3c6', fontSize: '1.2rem', cursor: 'pointer', padding: '0.2rem' }}><FaTimes /></button></div>
          <div style={{ marginBottom: '1rem', color: '#b0b3c6', fontSize: '0.9rem' }}>Model: COCO-SSD</div>
          <div style={{ marginBottom: '1rem' }}>
            <label style={{ display: 'block', marginBottom: '0.3rem', color: '#b0b3c6' }}>Confidence: {Math.round(confidence * 100)}%</label>
            <input type="range" min="0.1" max="1" step="0.05" value={confidence} onChange={(e) => setConfidence(parseFloat(e.target.value))} style={{ width: '100%' }} />
          </div>
          <div>
            <label style={{ display: 'block', marginBottom: '0.3rem', color: '#b0b3c6' }}>Max Objects: {maxObjects}</label>
            <input type="range" min="1" max="50" step="1" value={maxObjects} onChange={(e) => setMaxObjects(parseInt(e.target.value))} style={{ width: '100%' }} />
          </div>
        </div>
      )}
      <div style={contentStyle}>
        <div style={detectionListStyle}>
          {filteredDetections.length === 0 ? (
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', color: '#9ca3af', textAlign: 'center', padding: '2rem', height: '100%' }}>
              <FaSearchLocation style={{ fontSize: '3rem', marginBottom: '1rem', opacity: 0.5 }} />
              <p style={{ margin: 0, marginBottom: '0.5rem' }}>{!processingEnabled ? 'Click play to start detecting objects' : 'No objects detected'}</p>
            </div>
          ) : (
            filteredDetections.map((detection) => (
              <div key={detection.id} style={{ ...detectionItemStyle, borderLeft: `4px solid ${detection.color}` }}>
                <button onClick={() => handleRemoveDetection(detection.id)} style={{ position: 'absolute', top: '8px', right: '8px', background: 'rgba(255,255,255,0.1)', border: 'none', borderRadius: '50%', color: '#e6e9f0', cursor: 'pointer', width: '24px', height: '24px', display: 'flex', alignItems: 'center', justifyContent: 'center', fontSize: '14px', transition: 'background 0.2s' }} onMouseOver={(e) => e.currentTarget.style.background = 'rgba(239, 68, 68, 0.5)'} onMouseOut={(e) => e.currentTarget.style.background = 'rgba(255,255,255,0.1)'} title="Remove Detection"><FaTimes /></button>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', marginBottom: '0.5rem' }}>
                  <div style={{ fontSize: '1rem', fontWeight: 'bold', color: '#e6e9f0', textTransform: 'capitalize' }}>
                    <FaCube style={{ marginRight: '0.5rem', color: detection.color }} />
                    {detection.class}
                  </div>
                </div>

                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(120px, 1fr))', gap: '0.5rem', fontSize: '0.85rem' }}>
                  <div><div style={{ color: '#9ca3af', fontSize: '0.7rem' }}>CONFIDENCE</div><div style={{ color: '#a855f7', fontWeight: 'bold' }}>{Math.round(detection.score * 100)}%</div></div>
                  {detection.distance > 0 && <div><div style={{ color: '#9ca3af', fontSize: '0.7rem' }}>DISTANCE</div><div style={{ color: '#4ade80', fontWeight: 'bold' }}><FaRuler style={{ marginRight: '0.3rem' }} />{detection.distance.toFixed(2)}m</div></div>}
                  <div><div style={{ color: '#9ca3af', fontSize: '0.7rem' }}>SIZE</div><div style={{ color: '#fbbf24' }}>{Math.round(detection.width)} × {Math.round(detection.height)}</div></div>
                  <div><div style={{ color: '#9ca3af', fontSize: '0.7rem' }}>CENTER</div><div style={{ color: '#06b6d4' }}><FaBullseye style={{ marginRight: '0.3rem' }} />({Math.round(detection.centerX)}, {Math.round(detection.centerY)})</div></div>
                </div>
                
                {/* --- NEW: Timestamp positioned at the bottom-right --- */}
                <div style={{ position: 'absolute', bottom: '8px', right: '12px', fontSize: '0.75rem', color: '#9ca3af', opacity: 0.8 }}>
                  {detection.timestamp}
                </div>
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
};

export default ObjectDetectionPanel;