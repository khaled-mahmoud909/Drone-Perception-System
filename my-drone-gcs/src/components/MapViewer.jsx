import React, { useContext, useState, useEffect, useRef, useCallback } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import { FaMap, FaSyncAlt } from 'react-icons/fa';

const MapView = () => {
  const { ros, isConnected } = useContext(RosContext);
  const canvasRef = useRef(null);
  const mapCanvasRef = useRef(document.createElement('canvas')); // Off-screen canvas for the map
  const animationFrameId = useRef(null);

  const [mapData, setMapData] = useState(null);
  const [odomPose, setOdomPose] = useState(null);
  const [globalPath, setGlobalPath] = useState([]);
  const [finalTrajectory, setFinalTrajectory] = useState([]);
  
  const [scale, setScale] = useState(1.0);
  const [pan, setPan] = useState({ x: 0, y: 0 });

  const isPanning = useRef(false);
  const lastMousePos = useRef({ x: 0, y: 0 });

  // --- THE CORE FIX: A UNIFIED COORDINATE TRANSFORMATION FUNCTION ---
  // This function converts world coordinates (from odometry, paths, etc.)
  // into pixel coordinates on the map's grid.
  const worldToMapPixels = useCallback((worldX, worldY) => {
    if (!mapData) return { x: 0, y: 0 };
    const { info } = mapData;
    const { resolution, width, height, origin } = info;
    
    const pixelX = (worldX - origin.position.x) / resolution;
    // The Y-axis is inverted in canvas coordinates vs. ROS maps
    const pixelY = height - ((worldY - origin.position.y) / resolution);
    
    return { x: pixelX, y: pixelY };
  }, [mapData]);

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    const offscreenCanvas = mapCanvasRef.current;
    if (!canvas || !mapData) return;

    const ctx = canvas.getContext('2d');
    
    if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
        canvas.width = canvas.clientWidth;
        canvas.height = canvas.clientHeight;
    }

    ctx.fillStyle = '#333740';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    ctx.translate(pan.x, pan.y);
    ctx.scale(scale, scale);

    // --- 1. Draw the Pre-rendered Occupancy Grid Map ---
    ctx.drawImage(offscreenCanvas, 0, 0);

    // --- Helper function for drawing paths ---
    const drawPath = (path, color, lineWidth) => {
      if (path.length < 2) return;
      ctx.beginPath();
      const startPoint = worldToMapPixels(path[0].pose.position.x, path[0].pose.position.y);
      ctx.moveTo(startPoint.x, startPoint.y);
      path.forEach(p => {
        const point = worldToMapPixels(p.pose.position.x, p.pose.position.y);
        ctx.lineTo(point.x, point.y);
      });
      ctx.strokeStyle = color;
      ctx.lineWidth = Math.max(0.5, lineWidth / scale);
      ctx.stroke();
    };

    drawPath(globalPath, 'rgba(0, 150, 255, 0.7)', 2);
    drawPath(finalTrajectory, 'rgba(0, 255, 150, 0.9)', 3);

    // --- 3. Draw the Drone ---
    if (odomPose) {
      const { position, orientation } = odomPose;
      const yaw = Math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
      
      const droneCanvasPos = worldToMapPixels(position.x, position.y);

      ctx.save();
      ctx.translate(droneCanvasPos.x, droneCanvasPos.y);
      ctx.rotate(-yaw + Math.PI / 2); // ROS yaw (0=x-axis) to canvas angle (0=y-axis)

      // --- NEW DRONE ICON DRAWING LOGIC ---
      const droneSize = 12 / scale; // Keep drone size consistent on screen
      ctx.lineWidth = Math.max(0.5, 1.5 / scale);
      
      // Body
      ctx.beginPath();
      ctx.rect(-droneSize * 0.5, -droneSize * 0.7, droneSize, droneSize * 1.4);
      ctx.fillStyle = '#f87171'; // Red body
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();

      // Front direction indicator
      ctx.beginPath();
      ctx.moveTo(0, -droneSize * 0.8);
      ctx.lineTo(-droneSize * 0.2, -droneSize * 0.5);
      ctx.lineTo(droneSize * 0.2, -droneSize * 0.5);
      ctx.closePath();
      ctx.fillStyle = '#ffffff'; // White arrow
      ctx.fill();

      ctx.restore();
    }
    
    ctx.restore();

    animationFrameId.current = requestAnimationFrame(draw);
  }, [mapData, odomPose, globalPath, finalTrajectory, scale, pan, worldToMapPixels]);
  
  // --- Event Handlers (Unchanged) ---
  const handleWheel = (e) => {
    e.preventDefault();
    const zoomFactor = 1.1;
    const newScale = e.deltaY < 0 ? scale * zoomFactor : scale / zoomFactor;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    const newPanX = mouseX - (mouseX - pan.x) * (newScale / scale);
    const newPanY = mouseY - (mouseY - pan.y) * (newScale / scale);

    setScale(newScale);
    setPan({ x: newPanX, y: newPanY });
  };

  const handleMouseDown = (e) => {
    isPanning.current = true;
    lastMousePos.current = { x: e.clientX, y: e.clientY };
    canvasRef.current.style.cursor = 'grabbing';
  };

  const handleMouseUp = () => {
    isPanning.current = false;
    canvasRef.current.style.cursor = 'grab';
  };

  const handleMouseMove = (e) => {
    if (!isPanning.current) return;
    const dx = e.clientX - lastMousePos.current.x;
    const dy = e.clientY - lastMousePos.current.y;
    setPan(prevPan => ({ x: prevPan.x + dx, y: prevPan.y + dy }));
    lastMousePos.current = { x: e.clientX, y: e.clientY };
  };
  
  const resetView = useCallback(() => {
    if (!mapData || !canvasRef.current || !canvasRef.current.clientWidth) return;
    const { info } = mapData;
    const scaleX = canvasRef.current.clientWidth / info.width;
    const scaleY = canvasRef.current.clientHeight / info.height;
    const initialScale = Math.min(scaleX, scaleY) * 0.9;
    setScale(initialScale);
    
    const initialPanX = (canvasRef.current.clientWidth - (info.width * initialScale)) / 2;
    const initialPanY = (canvasRef.current.clientHeight - (info.height * initialScale)) / 2;
    setPan({ x: initialPanX, y: initialPanY });
  }, [mapData]);

  // Effect to pre-render the map to the off-screen canvas
  useEffect(() => {
    if (!mapData) return;
    const { info, data } = mapData;
    const { width, height } = info;
    const offscreenCanvas = mapCanvasRef.current;
    
    offscreenCanvas.width = width;
    offscreenCanvas.height = height;
    const ctx = offscreenCanvas.getContext('2d');
    
    // This is simpler and correct: The offscreen canvas is a direct, untransformed
    // pixel representation of the map grid.
    const imageData = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const mapIndex = x + (height - 1 - y) * width;
        const occupancyValue = data[mapIndex];
        const pixelIndex = (x + y * width) * 4;

        let color = [128, 128, 128];
        if (occupancyValue === 0) color = [250, 250, 250];
        else if (occupancyValue > 50) color = [40, 44, 52];
        
        imageData.data[pixelIndex] = color[0];
        imageData.data[pixelIndex + 1] = color[1];
        imageData.data[pixelIndex + 2] = color[2];
        imageData.data[pixelIndex + 3] = 255;
      }
    }
    ctx.putImageData(imageData, 0, 0);
    resetView();
  }, [mapData, resetView]);

  useEffect(() => {
    if (isConnected && ros) {
      animationFrameId.current = requestAnimationFrame(draw);
      
      // I kept the topic name as /map from your previous code, change if needed
      const mapListener = new ROSLIB.Topic({ ros, name: '/dynamic_map/2D_occupancy_map', messageType: 'nav_msgs/OccupancyGrid' });
      mapListener.subscribe(message => setMapData(message));
      
      const odomListener = new ROSLIB.Topic({ ros, name: '/odom', messageType: 'nav_msgs/Odometry' });
      odomListener.subscribe(message => setOdomPose(message.pose.pose));
      
      const globalPathListener = new ROSLIB.Topic({ ros, name: '/global_path', messageType: 'nav_msgs/Path' });
      globalPathListener.subscribe(message => setGlobalPath(message.poses));
      
      const finalTrajectoryListener = new ROSLIB.Topic({ ros, name: '/final_trajectory', messageType: 'nav_msgs/Path' });
      finalTrajectoryListener.subscribe(message => setFinalTrajectory(message.poses));

      return () => {
        mapListener.unsubscribe();
        odomListener.unsubscribe();
        globalPathListener.unsubscribe();
        finalTrajectoryListener.unsubscribe();
        if (animationFrameId.current) cancelAnimationFrame(animationFrameId.current);
      };
    }
  }, [isConnected, ros, draw]);


  return (
    <div style={{ width: '100%', height: '100%', position: 'relative', background: '#333740', cursor: 'grab' }}>
      <canvas
        ref={canvasRef}
        style={{ width: '100%', height: '100%', display: 'block' }}
        onWheel={handleWheel}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onMouseMove={handleMouseMove}
      />
      <button
        onClick={resetView}
        title="Reset View"
        style={{
          position: 'absolute', top: '10px', right: '10px', background: 'rgba(30, 34, 54, 0.8)',
          border: '1px solid rgba(255,255,255,0.1)', color: '#e6e9f0',
          borderRadius: '50%', width: '40px', height: '40px', cursor: 'pointer',
          display: 'flex', alignItems: 'center', justifyContent: 'center', fontSize: '1.2rem'
        }}
      >
        <FaSyncAlt />
      </button>
      {!mapData && (
        <div style={{
          position: 'absolute', top: 0, left: 0, width: '100%', height: '100%',
          display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
          color: '#9ca3af', textAlign: 'center'
        }}>
          <FaMap style={{fontSize: '4rem', marginBottom: '1rem'}}/>
          <p>Waiting for map data on <code style={{background: 'rgba(255,255,255,0.1)', padding: '2px 4px', borderRadius: '4px'}}>/2D_occupancy_map</code> topic...</p>
        </div>
      )}
    </div>
  );
};

export default MapView;