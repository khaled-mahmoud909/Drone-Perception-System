import React, { useContext, useEffect, useRef, useState, useCallback } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { FaMapMarkedAlt, FaEye, FaEyeSlash, FaCog } from 'react-icons/fa';

const RtabmapViewer = () => {
    const { ros, isConnected } = useContext(RosContext);
    const mountRef = useRef(null);

    // THREE.js object refs
    const rendererRef = useRef(null);
    const sceneRef = useRef(null);
    const cameraRef = useRef(null);
    const controlsRef = useRef(null);
    const animationFrameId = useRef(null);
    
    // Scene groups for organized rendering
    const graphGroupRef = useRef(new THREE.Group());
    const pointCloudGroupRef = useRef(new THREE.Group());
    const gridHelperRef = useRef(null);
    const axesHelperRef = useRef(null);

    // State management
    const [hasReceivedData, setHasReceivedData] = useState(false);
    const [isLoading, setIsLoading] = useState(false);
    const [showPointClouds, setShowPointClouds] = useState(true);
    const [showGraph, setShowGraph] = useState(true);
    const [showGrid, setShowGrid] = useState(true);
    const [debugInfo, setDebugInfo] = useState({
        nodes: 0,
        links: 0,
        totalPoints: 0,
        lastMessageTime: null,
        processingTime: 0,
        mapId: null
    });

    // Enhanced point cloud parsing with better error handling
    const parsePointCloud = useCallback((pointCloudMsg) => {
        if (!pointCloudMsg || !pointCloudMsg.data || !pointCloudMsg.fields) {
            console.warn("Invalid point cloud message structure");
            return { positions: new Float32Array(0), colors: new Float32Array(0) };
        }

        try {
            // Handle different data formats (compressed/uncompressed)
            let dataBuffer;
            if (typeof pointCloudMsg.data === 'string') {
                // Base64 encoded data
                const binaryString = atob(pointCloudMsg.data);
                dataBuffer = new Uint8Array(binaryString.length);
                for (let i = 0; i < binaryString.length; i++) {
                    dataBuffer[i] = binaryString.charCodeAt(i);
                }
            } else if (pointCloudMsg.data instanceof Uint8Array) {
                dataBuffer = pointCloudMsg.data;
            } else {
                dataBuffer = new Uint8Array(pointCloudMsg.data);
            }

            const dataView = new DataView(dataBuffer.buffer);
            
            // Find field offsets
            let xOffset, yOffset, zOffset, rgbOffset = -1;
            pointCloudMsg.fields.forEach(field => {
                switch (field.name) {
                    case 'x': xOffset = field.offset; break;
                    case 'y': yOffset = field.offset; break;
                    case 'z': zOffset = field.offset; break;
                    case 'rgb':
                    case 'rgba': rgbOffset = field.offset; break;
                }
            });
            
            if (xOffset === undefined || yOffset === undefined || zOffset === undefined) {
                console.error("Point cloud missing required x, y, z fields:", pointCloudMsg.fields);
                return { positions: new Float32Array(0), colors: new Float32Array(0) };
            }

            const numPoints = pointCloudMsg.width * pointCloudMsg.height;
            const pointStep = pointCloudMsg.point_step || 16; // Default point step
            
            if (numPoints === 0 || dataBuffer.length < numPoints * pointStep) {
                console.warn("Invalid point cloud dimensions or insufficient data");
                return { positions: new Float32Array(0), colors: new Float32Array(0) };
            }

            const positions = new Float32Array(numPoints * 3);
            const colors = new Float32Array(numPoints * 3);
            let validPoints = 0;

            for (let i = 0; i < numPoints; i++) {
                const pointOffset = i * pointStep;
                
                // Ensure we don't read beyond buffer
                if (pointOffset + Math.max(xOffset, yOffset, zOffset) + 4 > dataBuffer.length) {
                    break;
                }

                try {
                    const x = dataView.getFloat32(pointOffset + xOffset, true);
                    const y = dataView.getFloat32(pointOffset + yOffset, true);
                    const z = dataView.getFloat32(pointOffset + zOffset, true);

                    // Filter out invalid points
                    if (!isFinite(x) || !isFinite(y) || !isFinite(z) || 
                        Math.abs(x) > 1000 || Math.abs(y) > 1000 || Math.abs(z) > 1000) {
                        continue;
                    }
                    
                    positions[validPoints * 3] = x;
                    positions[validPoints * 3 + 1] = y;
                    positions[validPoints * 3 + 2] = z;

                    // Handle RGB data
                    if (rgbOffset !== -1 && pointOffset + rgbOffset + 4 <= dataBuffer.length) {
                        try {
                            const rgbFloat = dataView.getFloat32(pointOffset + rgbOffset, true);
                            const rgb = new Uint32Array(new Float32Array([rgbFloat]).buffer)[0];
                            colors[validPoints * 3] = ((rgb >> 16) & 0xFF) / 255.0;
                            colors[validPoints * 3 + 1] = ((rgb >> 8) & 0xFF) / 255.0;
                            colors[validPoints * 3 + 2] = (rgb & 0xFF) / 255.0;
                        } catch (rgbError) {
                            // Fallback to white if RGB parsing fails
                            colors[validPoints * 3] = 0.8;
                            colors[validPoints * 3 + 1] = 0.8;
                            colors[validPoints * 3 + 2] = 0.8;
                        }
                    } else {
                        // Default color (light gray)
                        colors[validPoints * 3] = 0.8;
                        colors[validPoints * 3 + 1] = 0.8;
                        colors[validPoints * 3 + 2] = 0.8;
                    }
                    
                    validPoints++;
                } catch (pointError) {
                    console.warn(`Error parsing point ${i}:`, pointError);
                    continue;
                }
            }

            // Return trimmed arrays with only valid points
            return {
                positions: positions.slice(0, validPoints * 3),
                colors: colors.slice(0, validPoints * 3)
            };
        } catch (error) {
            console.error("Error parsing point cloud:", error);
            return { positions: new Float32Array(0), colors: new Float32Array(0) };
        }
    }, []);

    // Enhanced scene update with better error handling and ROS message compatibility
    const updateScene = useCallback((mapData) => {
        if (!sceneRef.current) return;
        
        console.log("Received map data:", mapData);
        setIsLoading(true);
        const startTime = performance.now();

        try {
            // Mark that we've received data
            if (!hasReceivedData) {
                setHasReceivedData(true);
            }

            // Clear existing content
            while (graphGroupRef.current.children.length) {
                const child = graphGroupRef.current.children[0];
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach(m => m.dispose());
                    } else {
                        child.material.dispose();
                    }
                }
                graphGroupRef.current.remove(child);
            }

            while (pointCloudGroupRef.current.children.length) {
                const child = pointCloudGroupRef.current.children[0];
                if (child.geometry) child.geometry.dispose();
                if (child.material) child.material.dispose();
                pointCloudGroupRef.current.remove(child);
            }

            let totalPoints = 0;
            let nodeCount = 0;
            let linkCount = 0;

            // Process graph data if available - handle ROS message structure carefully
            if (mapData.graph && mapData.graph.poses && Array.isArray(mapData.graph.poses) && mapData.graph.poses.length > 0) {
                console.log("Processing graph with", mapData.graph.poses.length, "poses");
                
                const nodePosMap = new Map();
                
                // Create node positions map - handle different message formats
                if (mapData.graph.poses_id && Array.isArray(mapData.graph.poses_id) && mapData.graph.poses_id.length === mapData.graph.poses.length) {
                    mapData.graph.poses.forEach((pose, i) => {
                        const poseId = mapData.graph.poses_id[i];
                        // Handle different pose message structures
                        let position = null;
                        if (pose && typeof pose === 'object') {
                            if (pose.position && typeof pose.position === 'object') {
                                position = {
                                    x: parseFloat(pose.position.x) || 0,
                                    y: parseFloat(pose.position.y) || 0,
                                    z: parseFloat(pose.position.z) || 0
                                };
                            } else if (pose.pose && pose.pose.position) {
                                position = {
                                    x: parseFloat(pose.pose.position.x) || 0,
                                    y: parseFloat(pose.pose.position.y) || 0,
                                    z: parseFloat(pose.pose.position.z) || 0
                                };
                            } else if (typeof pose.x !== 'undefined') {
                                // Direct x,y,z fields
                                position = {
                                    x: parseFloat(pose.x) || 0,
                                    y: parseFloat(pose.y) || 0,
                                    z: parseFloat(pose.z) || 0
                                };
                            }
                        }
                        
                        if (position && isFinite(position.x) && isFinite(position.y) && isFinite(position.z)) {
                            nodePosMap.set(poseId, position);
                        }
                    });
                } else {
                    // Fallback: use array index as ID
                    mapData.graph.poses.forEach((pose, i) => {
                        let position = null;
                        if (pose && typeof pose === 'object') {
                            if (pose.position && typeof pose.position === 'object') {
                                position = {
                                    x: parseFloat(pose.position.x) || 0,
                                    y: parseFloat(pose.position.y) || 0,
                                    z: parseFloat(pose.position.z) || 0
                                };
                            } else if (pose.pose && pose.pose.position) {
                                position = {
                                    x: parseFloat(pose.pose.position.x) || 0,
                                    y: parseFloat(pose.pose.position.y) || 0,
                                    z: parseFloat(pose.pose.position.z) || 0
                                };
                            }
                        }
                        
                        if (position && isFinite(position.x) && isFinite(position.y) && isFinite(position.z)) {
                            nodePosMap.set(i, position);
                        }
                    });
                }

                // Render nodes
                if (showGraph && nodePosMap.size > 0) {
                    const nodeGeometry = new THREE.SphereGeometry(0.1, 12, 8);
                    const nodeMaterial = new THREE.MeshBasicMaterial({ 
                        color: 0xff6b35,
                        transparent: true,
                        opacity: 0.8
                    });
                    
                    for (const [nodeId, position] of nodePosMap) {
                        try {
                            const sphere = new THREE.Mesh(nodeGeometry, nodeMaterial.clone());
                            sphere.position.set(position.x, position.y, position.z);
                            
                            // Add node ID as userData for debugging
                            sphere.userData = { 
                                nodeId: nodeId,
                                type: 'node'
                            };
                            
                            graphGroupRef.current.add(sphere);
                            nodeCount++;
                        } catch (sphereError) {
                            console.warn(`Error creating sphere for node ${nodeId}:`, sphereError);
                        }
                    }

                    // Render links - handle different link message structures
                    if (mapData.graph.links && Array.isArray(mapData.graph.links) && mapData.graph.links.length > 0) {
                        const lineMaterial = new THREE.LineBasicMaterial({ 
                            color: 0x4ade80,
                            transparent: true,
                            opacity: 0.6,
                            linewidth: 2
                        });
                        
                        mapData.graph.links.forEach(link => {
                            try {
                                let fromId, toId;
                                
                                // Handle different link message formats
                                if (typeof link === 'object' && link !== null) {
                                    fromId = link.from_id ?? link.fromId ?? link.from;
                                    toId = link.to_id ?? link.toId ?? link.to;
                                }
                                
                                if (fromId !== undefined && toId !== undefined) {
                                    const fromPos = nodePosMap.get(fromId);
                                    const toPos = nodePosMap.get(toId);
                                    
                                    if (fromPos && toPos) {
                                        const points = [
                                            new THREE.Vector3(fromPos.x, fromPos.y, fromPos.z),
                                            new THREE.Vector3(toPos.x, toPos.y, toPos.z)
                                        ];
                                        const geometry = new THREE.BufferGeometry().setFromPoints(points);
                                        const line = new THREE.Line(geometry, lineMaterial.clone());
                                        line.userData = { type: 'link', fromId: fromId, toId: toId };
                                        graphGroupRef.current.add(line);
                                        linkCount++;
                                    }
                                }
                            } catch (linkError) {
                                console.warn("Error processing link:", linkError, link);
                            }
                        });
                    }
                }
            }

            // Process point cloud data if available - handle ROS message compatibility
            if (showPointClouds && mapData.nodes && Array.isArray(mapData.nodes) && mapData.nodes.length > 0) {
                console.log("Processing", mapData.nodes.length, "point cloud nodes");
                
                mapData.nodes.forEach((node, index) => {
                    // Handle different node message structures
                    let pointCloudData = null;
                    
                    if (node && typeof node === 'object') {
                        pointCloudData = node.word_pts || node.wordPts || node.point_cloud || node.pointCloud;
                    }
                    
                    if (!pointCloudData || !pointCloudData.data) {
                        return;
                    }

                    try {
                        const { positions, colors } = parsePointCloud(pointCloudData);
                        
                        if (positions.length > 0) {
                            const geometry = new THREE.BufferGeometry();
                            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
                            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
                            
                            const material = new THREE.PointsMaterial({ 
                                size: 0.03,
                                vertexColors: true,
                                sizeAttenuation: true,
                                transparent: true,
                                opacity: 0.8
                            });
                            
                            const points = new THREE.Points(geometry, material);
                            points.name = `pointcloud_${node.id || index}`;
                            points.userData = { 
                                nodeId: node.id || index,
                                type: 'pointcloud',
                                pointCount: positions.length / 3
                            };
                            
                            pointCloudGroupRef.current.add(points);
                            totalPoints += positions.length / 3;
                        }
                    } catch (error) {
                        console.error(`Error processing point cloud for node ${node.id || index}:`, error);
                    }
                });
            }

            const processingTime = performance.now() - startTime;
            
            setDebugInfo({
                nodes: nodeCount,
                links: linkCount,
                totalPoints: totalPoints,
                lastMessageTime: new Date().toLocaleTimeString(),
                processingTime: Math.round(processingTime),
                mapId: mapData.header ? mapData.header.seq : null
            });

            console.log(`Scene updated: ${nodeCount} nodes, ${linkCount} links, ${totalPoints} points in ${processingTime.toFixed(1)}ms`);

        } catch (error) {
            console.error("Error updating scene:", error);
            console.error("MapData structure:", JSON.stringify(mapData, null, 2));
        } finally {
            setIsLoading(false);
        }
    }, [hasReceivedData, showGraph, showPointClouds, parsePointCloud]);

    // Initialize THREE.js scene
    useEffect(() => {
        if (!isConnected || !ros || !mountRef.current || rendererRef.current) return;

        console.log("Initializing RTAB-Map viewer...");
        
        const mount = mountRef.current;
        const width = mount.clientWidth;
        const height = mount.clientHeight;

        // Scene setup
        sceneRef.current = new THREE.Scene();
        sceneRef.current.background = new THREE.Color(0x1a1a1a);

        // Camera setup (Z-up coordinate system like ROS)
        cameraRef.current = new THREE.PerspectiveCamera(60, width / height, 0.01, 1000);
        cameraRef.current.position.set(10, -10, 8);
        cameraRef.current.up.set(0, 0, 1); // Z-up
        cameraRef.current.lookAt(0, 0, 0);

        // Renderer setup
        rendererRef.current = new THREE.WebGLRenderer({ 
            antialias: true, 
            logarithmicDepthBuffer: true 
        });
        rendererRef.current.setSize(width, height);
        rendererRef.current.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        rendererRef.current.shadowMap.enabled = true;
        rendererRef.current.shadowMap.type = THREE.PCFSoftShadowMap;
        mount.appendChild(rendererRef.current.domElement);

        // Controls setup
        controlsRef.current = new OrbitControls(cameraRef.current, rendererRef.current.domElement);
        controlsRef.current.target.set(0, 0, 0);
        controlsRef.current.enableDamping = true;
        controlsRef.current.dampingFactor = 0.05;
        controlsRef.current.maxDistance = 500;
        controlsRef.current.minDistance = 0.1;

        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        sceneRef.current.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 10);
        directionalLight.castShadow = true;
        sceneRef.current.add(directionalLight);

        // Grid helper (only add if showGrid is true initially)
        if (showGrid) {
            gridHelperRef.current = new THREE.GridHelper(20, 20, 0x666666, 0x333333);
            gridHelperRef.current.rotation.x = Math.PI / 2; // Rotate for Z-up
            sceneRef.current.add(gridHelperRef.current);
        }

        // Axes helper
        axesHelperRef.current = new THREE.AxesHelper(2);
        sceneRef.current.add(axesHelperRef.current);

        // Add groups to scene
        sceneRef.current.add(graphGroupRef.current);
        sceneRef.current.add(pointCloudGroupRef.current);

        // Animation loop
        const animate = () => {
            animationFrameId.current = requestAnimationFrame(animate);
            if (controlsRef.current) controlsRef.current.update();
            if (rendererRef.current && sceneRef.current && cameraRef.current) {
                rendererRef.current.render(sceneRef.current, cameraRef.current);
            }
        };
        animate();

        // Resize handler
        const handleResize = () => {
            if (!mount || !cameraRef.current || !rendererRef.current) return;
            const newWidth = mount.clientWidth;
            const newHeight = mount.clientHeight;
            cameraRef.current.aspect = newWidth / newHeight;
            cameraRef.current.updateProjectionMatrix();
            rendererRef.current.setSize(newWidth, newHeight);
        };
        window.addEventListener('resize', handleResize);

        // Subscribe to /mapData topic with more specific error handling
        console.log("Subscribing to /mapData topic...");
        const mapDataListener = new ROSLIB.Topic({
            ros: ros,
            name: '/mapData',
            messageType: 'rtabmap_msgs/MapData',
            compression: 'cbor'
        });

        mapDataListener.subscribe((message) => {
            console.log("Received mapData message:", message);
            try {
                updateScene(message);
            } catch (error) {
                console.error("Error in mapData callback:", error);
                console.error("Message structure:", JSON.stringify(message, null, 2));
            }
        });

        // Also try to subscribe without compression as fallback
        const mapDataListenerFallback = new ROSLIB.Topic({
            ros: ros,
            name: '/mapData',
            messageType: 'rtabmap_msgs/MapData'
        });

        // Only use fallback if CBOR fails
        const fallbackTimeout = setTimeout(() => {
            if (!hasReceivedData) {
                console.log("Trying fallback subscription without compression...");
                mapDataListenerFallback.subscribe((message) => {
                    console.log("Received mapData message (fallback):", message);
                    try {
                        updateScene(message);
                    } catch (error) {
                        console.error("Error in fallback mapData callback:", error);
                    }
                });
            }
        }, 5000); // Wait 5 seconds before trying fallback

        // Cleanup
        return () => {
            console.log("Cleaning up RTAB-Map viewer...");
            clearTimeout(fallbackTimeout);
            mapDataListener.unsubscribe();
            mapDataListenerFallback.unsubscribe();
            window.removeEventListener('resize', handleResize);
            
            if (animationFrameId.current) {
                cancelAnimationFrame(animationFrameId.current);
            }
            
            if (mountRef.current && rendererRef.current?.domElement) {
                try {
                    mountRef.current.removeChild(rendererRef.current.domElement);
                } catch (e) {
                    console.warn("Error removing renderer element:", e);
                }
            }
            
            // Dispose of THREE.js resources
            if (sceneRef.current) {
                sceneRef.current.traverse(object => {
                    if (object.geometry) object.geometry.dispose();
                    if (object.material) {
                        if (Array.isArray(object.material)) {
                            object.material.forEach(m => m.dispose());
                        } else {
                            object.material.dispose();
                        }
                    }
                });
            }
            
            if (controlsRef.current) controlsRef.current.dispose();
            if (rendererRef.current) rendererRef.current.dispose();
            
            rendererRef.current = null;
            sceneRef.current = null;
            cameraRef.current = null;
            controlsRef.current = null;
        };

    }, [isConnected, ros, updateScene, showGrid]);

    // Update visibility when toggles change
    useEffect(() => {
        if (graphGroupRef.current) {
            graphGroupRef.current.visible = showGraph;
        }
        if (pointCloudGroupRef.current) {
            pointCloudGroupRef.current.visible = showPointClouds;
        }
        if (gridHelperRef.current) {
            gridHelperRef.current.visible = showGrid;
        } else if (showGrid && sceneRef.current) {
            // Create grid if it doesn't exist and should be shown
            gridHelperRef.current = new THREE.GridHelper(20, 20, 0x666666, 0x333333);
            gridHelperRef.current.rotation.x = Math.PI / 2;
            sceneRef.current.add(gridHelperRef.current);
        }
    }, [showGraph, showPointClouds, showGrid]);

    const placeholderStyle = {
        position: 'absolute',
        top: 0,
        left: 0,
        width: '100%',
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        color: '#9ca3af',
        textAlign: 'center',
        background: 'rgba(26, 26, 26, 0.9)',
        zIndex: 101,
        pointerEvents: 'none'
    };

    const debugStyle = {
        position: 'absolute',
        top: '10px',
        left: '10px',
        background: 'rgba(0,0,0,0.8)',
        color: 'white',
        padding: '12px',
        borderRadius: '8px',
        fontFamily: 'monospace',
        fontSize: '11px',
        zIndex: 100,
        minWidth: '200px',
        backdropFilter: 'blur(4px)'
    };

    const controlsStyle = {
        position: 'absolute',
        top: '10px',
        right: '10px',
        background: 'rgba(0,0,0,0.8)',
        color: 'white',
        padding: '12px',
        borderRadius: '8px',
        zIndex: 100,
        backdropFilter: 'blur(4px)'
    };

    const toggleButtonStyle = {
        background: 'rgba(255,255,255,0.1)',
        border: '1px solid rgba(255,255,255,0.2)',
        color: 'white',
        padding: '4px 8px',
        borderRadius: '4px',
        cursor: 'pointer',
        fontSize: '10px',
        margin: '2px',
        display: 'inline-flex',
        alignItems: 'center',
        gap: '4px'
    };

    return (
        <div className="rtabmap-viewer-container" style={{ position: 'relative', width: '100%', height: '100%' }}>
            {/* Placeholder overlay */}
            {!hasReceivedData && !isLoading && (
                <div style={placeholderStyle}>
                    <FaMapMarkedAlt style={{fontSize: '4rem', marginBottom: '1rem', opacity: 0.7}}/>
                    <h4 style={{margin: '0 0 1rem 0'}}>Waiting for RTAB-Map Data...</h4>
                    <p style={{margin: 0, maxWidth: '80%', lineHeight: 1.4}}>
                        Listening to <code>/mapData</code> topic.<br/>
                        Start mapping with your robot to see the visualization.
                    </p>
                </div>
            )}

            {/* Loading indicator */}
            {isLoading && (
                <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: '50%',
                    transform: 'translate(-50%, -50%)',
                    background: 'rgba(0,0,0,0.8)',
                    color: 'white',
                    padding: '20px',
                    borderRadius: '8px',
                    zIndex: 102
                }}>
                    <FaCog className="fa-spin" style={{marginRight: '8px'}}/>
                    Processing map data...
                </div>
            )}

            {/* Debug info panel */}
            <div style={debugStyle}>
                <div style={{fontWeight: 'bold', marginBottom: '8px', color: '#4ade80'}}>
                    <FaMapMarkedAlt style={{marginRight: '6px'}}/>
                    RTAB-Map Viewer
                </div>
                <div>Status: {isConnected ? '✅ Connected' : '❌ Disconnected'}</div>
                <div>Nodes: <span style={{color: showGraph ? '#4ade80' : '#666'}}>{debugInfo.nodes}</span></div>
                <div>Links: <span style={{color: showGraph ? '#4ade80' : '#666'}}>{debugInfo.links}</span></div>
                <div>Points: <span style={{color: showPointClouds ? '#60a5fa' : '#666'}}>{debugInfo.totalPoints.toLocaleString()}</span></div>
                <div>Processing: {debugInfo.processingTime}ms</div>
                <div>Updated: {debugInfo.lastMessageTime || 'Never'}</div>
                {debugInfo.mapId && <div>Map ID: {debugInfo.mapId}</div>}
            </div>

            {/* Control panel */}
            <div style={controlsStyle}>
                <div style={{fontWeight: 'bold', marginBottom: '8px', fontSize: '12px'}}>Display Options</div>
                <div>
                    <button
                        style={{
                            ...toggleButtonStyle,
                            background: showGraph ? 'rgba(74, 222, 128, 0.3)' : 'rgba(255,255,255,0.1)'
                        }}
                        onClick={() => setShowGraph(!showGraph)}
                    >
                        {showGraph ? <FaEye /> : <FaEyeSlash />}
                        Graph
                    </button>
                    <button
                        style={{
                            ...toggleButtonStyle,
                            background: showPointClouds ? 'rgba(96, 165, 250, 0.3)' : 'rgba(255,255,255,0.1)'
                        }}
                        onClick={() => setShowPointClouds(!showPointClouds)}
                    >
                        {showPointClouds ? <FaEye /> : <FaEyeSlash />}
                        Points
                    </button>
                    <button
                        style={{
                            ...toggleButtonStyle,
                            background: showGrid ? 'rgba(156, 163, 175, 0.3)' : 'rgba(255,255,255,0.1)'
                        }}
                        onClick={() => setShowGrid(!showGrid)}
                    >
                        {showGrid ? <FaEye /> : <FaEyeSlash />}
                        Grid
                    </button>
                </div>
            </div>

            {/* 3D Viewer */}
            <div ref={mountRef} style={{ width: '100%', height: '100%' }} />
        </div>
    );
};

export default RtabmapViewer;