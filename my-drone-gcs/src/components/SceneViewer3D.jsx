import React, { useContext, useEffect, useRef, useState } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import * as THREE from 'three'; // Import THREE.js
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'; // Import OrbitControls

const SceneViewer3D = () => {
    const { ros, isConnected } = useContext(RosContext);
    const viewerRef = useRef(null);
    const [error, setError] = useState(null);
    
    // Refs to hold THREE.js objects
    const sceneRef = useRef(null);
    const rendererRef = useRef(null);
    const cameraRef = useRef(null);
    const controlsRef = useRef(null);
    const frameRef = useRef(null);
    
    // ROS-related refs
    const voxelMapMesh = useRef(null);
    const trajectoryLine = useRef(null);
    const droneModel = useRef(null);
    const dynamicObstaclesGroup = useRef(null);

    // Function to create a simple drone model (no changes)
    const createDroneModel = () => {
        const group = new THREE.Group();

        // Body - Make it longer along the X-axis
        const bodyMat = new THREE.MeshBasicMaterial({ color: 0xcccccc });
        const bodyGeo = new THREE.BoxGeometry(0.4, 0.3, 0.05); // Longer (0.4) on X, wider (0.3) on Y
        const body = new THREE.Mesh(bodyGeo, bodyMat);
        group.add(body);

        // Camera - Positioned at the front (+X side)
        const camMat = new THREE.MeshBasicMaterial({ color: 0x333333 });
        const camGeo = new THREE.BoxGeometry(0.08, 0.08, 0.1);
        const camera = new THREE.Mesh(camGeo, camMat);
        camera.position.set(0.2, 0, 0); 
        group.add(camera);
        
        // Front indicator arrow
        const arrowGeo = new THREE.ConeGeometry(0.1, 0.2, 4); // By default, cone points up its local +Y axis
        const arrowMat = new THREE.MeshBasicMaterial({ color: 0xff4444 });
        const arrow = new THREE.Mesh(arrowGeo, arrowMat);
        
        // Position the arrow at the front of the body
        arrow.position.set(0.35, 0, 0); 
        
        // Rotate the arrow from pointing up (+Y) to pointing forward (+X)
        arrow.rotation.z = -Math.PI / 2; 

        group.add(arrow);

        // No final group rotation is needed. The model is now built correctly.
        return group;
    };

    // Create a simple grid (no changes)
    const createGrid = () => {
        const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
        gridHelper.position.y = 0; // Keep it on the XZ plane (ground level)
        return gridHelper;
    };

    // Initialize THREE.js scene manually
    const initializeScene = () => {
        if (!viewerRef.current) return false;

        // Clear any existing content
        while (viewerRef.current.firstChild) {
            viewerRef.current.removeChild(viewerRef.current.firstChild);
        }

        try {
            // Scene
            sceneRef.current = new THREE.Scene();
            sceneRef.current.background = new THREE.Color(0x232946);

            // Camera
            const width = viewerRef.current.clientWidth || 800;
            const height = viewerRef.current.clientHeight || 600;
            cameraRef.current = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
            cameraRef.current.position.set(8, 8, 8);
            cameraRef.current.up.set(0, 0, 1);
            cameraRef.current.position.set(5, -5, 5);
            cameraRef.current.lookAt(0, 0, 0);

            // Renderer
            rendererRef.current = new THREE.WebGLRenderer({ 
                antialias: true,
                alpha: true
            });
            rendererRef.current.setSize(width, height);
            rendererRef.current.setClearColor(0x232946, 1);
            rendererRef.current.shadowMap.enabled = true;
            rendererRef.current.shadowMap.type = THREE.PCFSoftShadowMap;
            
            // Append renderer to DOM
            viewerRef.current.appendChild(rendererRef.current.domElement);

            // --- SIMPLIFIED: Initialize OrbitControls directly ---
            controlsRef.current = new OrbitControls(cameraRef.current, rendererRef.current.domElement);
            controlsRef.current.enableDamping = true;
            controlsRef.current.dampingFactor = 0.05;
            controlsRef.current.enableZoom = true;
            controlsRef.current.enableRotate = true;
            controlsRef.current.enablePan = true;
            controlsRef.current.maxDistance = 100;
            controlsRef.current.minDistance = 1;
            controlsRef.current.target.set(0, 0, 0);
            controlsRef.current.update();
            console.log("✅ OrbitControls initialized successfully");

            // Add grid - single grid only
            const grid = createGrid();
            grid.rotation.x = Math.PI / 2;
            sceneRef.current.add(grid);

            // Add coordinate axes for reference
            const axesHelper = new THREE.AxesHelper(2);
            sceneRef.current.add(axesHelper);

            // Add lighting
            const ambientLight = new THREE.AmbientLight(0x404040, 0.8);
            sceneRef.current.add(ambientLight);
            
            const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
            directionalLight.position.set(10, 10, 10);
            directionalLight.castShadow = true;
            directionalLight.shadow.mapSize.width = 2048;
            directionalLight.shadow.mapSize.height = 2048;
            sceneRef.current.add(directionalLight);

            // Add drone model
            droneModel.current = createDroneModel();
            droneModel.current.position.set(0, 0, 0.5); // Start slightly above ground
            droneModel.current.castShadow = true;
            droneModel.current.receiveShadow = true;
            sceneRef.current.add(droneModel.current);

            // Add dynamic obstacles group
            dynamicObstaclesGroup.current = new THREE.Group();
            sceneRef.current.add(dynamicObstaclesGroup.current);

            // Start render loop
            const animate = () => {
                frameRef.current = requestAnimationFrame(animate);
                
                if (controlsRef.current) {
                    controlsRef.current.update();
                }
                
                if (rendererRef.current && sceneRef.current && cameraRef.current) {
                    rendererRef.current.render(sceneRef.current, cameraRef.current);
                }
            };
            animate();

            console.log("✅ 3D Scene initialized successfully");
            return true;
        } catch (err) {
            console.error("Error initializing 3D scene:", err);
            setError(`Failed to initialize 3D scene: ${err.message}`);
            return false;
        }
    };

    // Handle window resize (no changes)
    const handleResize = () => {
        if (!viewerRef.current || !cameraRef.current || !rendererRef.current) return;
        
        const width = viewerRef.current.clientWidth;
        const height = viewerRef.current.clientHeight;
        
        if (width > 0 && height > 0) {
            cameraRef.current.aspect = width / height;
            cameraRef.current.updateProjectionMatrix();
            rendererRef.current.setSize(width, height);
        }
    };

    // Main initialization effect
    useEffect(() => {
        if (!isConnected || !ros || !viewerRef.current) return;

        // Initialize the 3D scene
        if (!initializeScene()) return;

        // ROS Topic Subscriptions
        const subscriptions = [];

        // 1. Voxel Map (PointCloud2)
        const voxelMapListener = new ROSLIB.Topic({
            ros: ros,
            name: '/dynamic_map/voxel_map',
            messageType: 'sensor_msgs/PointCloud2',
            compression: 'cbor'
        });
        
        voxelMapListener.subscribe(message => {
            if (!sceneRef.current) return;

            // Remove old map
            if (voxelMapMesh.current) {
                sceneRef.current.remove(voxelMapMesh.current);
                if (voxelMapMesh.current.geometry) voxelMapMesh.current.geometry.dispose();
                if (voxelMapMesh.current.material) voxelMapMesh.current.material.dispose();
            }

            const numPoints = message.width * message.height;
            if (numPoints === 0) return;

            try {
                const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
                const material = new THREE.MeshBasicMaterial({ 
                    transparent: true, 
                    opacity: 0.6 
                });
                
                const instancedMesh = new THREE.InstancedMesh(geometry, material, numPoints);
                
                const dataView = new DataView(new Uint8Array(message.data).buffer);
                // --- Pass 1: Extract points and find Z-bounds for color mapping ---
                const points = [];
                let minZ = Infinity;
                let maxZ = -Infinity;

                for (let i = 0; i < numPoints; i++) {
                    const pointOffset = i * message.point_step;
                    const x = dataView.getFloat32(pointOffset + message.fields[0].offset, true);
                    const y = dataView.getFloat32(pointOffset + message.fields[1].offset, true);
                    const z = dataView.getFloat32(pointOffset + message.fields[2].offset, true);

                    if (isFinite(x) && isFinite(y) && isFinite(z)) {
                        points.push({x, y, z});
                        if (z < minZ) minZ = z;
                        if (z > maxZ) maxZ = z;
                    }
                }
                
                // --- Pass 2: Set instance matrix and color ---
                const dummy = new THREE.Object3D();
                const color = new THREE.Color();
                const lowColor = new THREE.Color(0x00BFFF); // DeepSkyBlue
                const highColor = new THREE.Color(0xFFFF00); // Yellow
                const zRange = maxZ - minZ;

                points.forEach((point, i) => {
                    dummy.position.set(point.x, point.y, point.z);
                    dummy.updateMatrix();
                    instancedMesh.setMatrixAt(i, dummy.matrix);

                    // Calculate color based on height
                    const normalizedZ = (zRange > 0.01) ? ((point.z - minZ) / zRange) : 1.0;
                    color.lerpColors(lowColor, highColor, normalizedZ);
                    instancedMesh.setColorAt(i, color);
                });

                instancedMesh.instanceMatrix.needsUpdate = true;
                if (instancedMesh.instanceColor) { // Check if instanceColor buffer exists
                   instancedMesh.instanceColor.needsUpdate = true;
                }
                instancedMesh.count = points.length;

                voxelMapMesh.current = instancedMesh;
                sceneRef.current.add(voxelMapMesh.current);

            } catch (err) {
                console.error("Error processing voxel map:", err);
            }
        });
        subscriptions.push(voxelMapListener);

        // 2. Drone Odometry (no changes)
        const odomListener = new ROSLIB.Topic({
            ros: ros,
            name: '/odom',
            messageType: 'nav_msgs/Odometry'
        });
        
        odomListener.subscribe(message => {
            const arrivalTime = new Date().getTime() / 1000.0; // Time in seconds
            const sentTime = message.header.stamp.secs + message.header.stamp.nsecs / 1e9;
            const latency = arrivalTime - sentTime;
            console.log(`ODOM Latency: ${latency.toFixed(4)}s`);
            if (droneModel.current) {
                const { position, orientation } = message.pose.pose;
                droneModel.current.position.set(position.x, position.y, position.z);
                droneModel.current.quaternion.set(orientation.x, orientation.y, orientation.z, orientation.w);
            }
        });
        subscriptions.push(odomListener);
        
        // 3. Trajectory Visualization (no changes)
        const trajectoryListener = new ROSLIB.Topic({
            ros: ros,
            name: '/final_trajectory',
            messageType: 'nav_msgs/Path'
        });
        
        trajectoryListener.subscribe(message => {
            if (!sceneRef.current) return;

            if (trajectoryLine.current) {
                sceneRef.current.remove(trajectoryLine.current);
                if (trajectoryLine.current.geometry) trajectoryLine.current.geometry.dispose();
                if (trajectoryLine.current.material) trajectoryLine.current.material.dispose();
            }

            const points = message.poses.map(p => 
                new THREE.Vector3(p.pose.position.x, p.pose.position.y, p.pose.position.z)
            );
            
            if (points.length < 2) return;

            try {
                const geometry = new THREE.BufferGeometry().setFromPoints(points);
                const material = new THREE.LineBasicMaterial({ color: 0x00ff99 });
                
                trajectoryLine.current = new THREE.Line(geometry, material);
                sceneRef.current.add(trajectoryLine.current);
            } catch (err) {
                console.error("Error creating trajectory:", err);
            }
        });
        subscriptions.push(trajectoryListener);

        // 4. Dynamic Obstacle Bounding Boxes (no changes)
        const obstaclesListener = new ROSLIB.Topic({
            ros: ros,
            name: '/onboard_detector/dynamic_bboxes',
            messageType: 'visualization_msgs/MarkerArray'
        });
        
        obstaclesListener.subscribe(message => {
            if (!dynamicObstaclesGroup.current) return;
            
            // Clear previous obstacles
            while(dynamicObstaclesGroup.current.children.length > 0){ 
                const child = dynamicObstaclesGroup.current.children[0];
                dynamicObstaclesGroup.current.remove(child);
                if (child.geometry) child.geometry.dispose();
                if (child.material) child.material.dispose();
            }

            message.markers.forEach(marker => {
                if (marker.type !== 1) return; // Type 1 is CUBE

                try {
                    const geometry = new THREE.BoxGeometry(marker.scale.x, marker.scale.y, marker.scale.z);
                    const material = new THREE.MeshBasicMaterial({
                        color: new THREE.Color(marker.color.r, marker.color.g, marker.color.b),
                        transparent: true,
                        opacity: marker.color.a
                    });
                    
                    const boxMesh = new THREE.Mesh(geometry, material);
                    boxMesh.position.set(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                    boxMesh.quaternion.set(
                        marker.pose.orientation.x, 
                        marker.pose.orientation.y, 
                        marker.pose.orientation.z, 
                        marker.pose.orientation.w
                    );
                    
                    dynamicObstaclesGroup.current.add(boxMesh);
                } catch (err) {
                    console.error("Error creating obstacle:", err);
                }
            });
        });
        subscriptions.push(obstaclesListener);

        // Add resize listener
        window.addEventListener('resize', handleResize);

        return () => {
            // Cleanup subscriptions
            subscriptions.forEach(sub => {
                try {
                    sub.unsubscribe();
                } catch (err) {
                    console.warn("Error unsubscribing:", err);
                }
            });

            // Stop animation loop
            if (frameRef.current) {
                cancelAnimationFrame(frameRef.current);
                frameRef.current = null;
            }
            
            // Cleanup controls
            if (controlsRef.current) {
                if (controlsRef.current.dispose) {
                    controlsRef.current.dispose();
                }
                controlsRef.current = null;
            }
            
            // Cleanup renderer and remove from DOM safely
            if (rendererRef.current) {
                try {
                    // Dispose of renderer resources
                    rendererRef.current.dispose();
                    
                    // Remove canvas from DOM if it exists and is still a child
                    const canvas = rendererRef.current.domElement;
                    if (canvas && canvas.parentNode === viewerRef.current) {
                        viewerRef.current.removeChild(canvas);
                    }
                } catch (err) {
                    console.warn("Error during renderer cleanup:", err);
                }
                rendererRef.current = null;
            }

            // Cleanup scene objects
            if (sceneRef.current) {
                // Dispose of geometries and materials
                sceneRef.current.traverse((object) => {
                    if (object.geometry) {
                        object.geometry.dispose();
                    }
                    if (object.material) {
                        if (Array.isArray(object.material)) {
                            object.material.forEach(material => material.dispose());
                        } else {
                            object.material.dispose();
                        }
                    }
                });
                sceneRef.current = null;
            }

            // Clear refs
            cameraRef.current = null;
            voxelMapMesh.current = null;
            trajectoryLine.current = null;
            droneModel.current = null;
            dynamicObstaclesGroup.current = null;

            window.removeEventListener('resize', handleResize);
        };
    }, [isConnected, ros]);

    if (error) {
        return (
            <div style={{ 
                padding: '2rem', 
                textAlign: 'center', 
                color: '#ff6b6b',
                background: 'rgba(255, 107, 107, 0.1)',
                borderRadius: '8px',
                margin: '1rem'
            }}>
                <h3>3D Viewer Error</h3>
                <p>{error}</p>
                <small>
                    Make sure THREE.js is properly loaded. You may need to include OrbitControls for camera movement.
                </small>
            </div>
        );
    }

    if (!isConnected) {
        return (
            <div style={{ 
                padding: '2rem', 
                textAlign: 'center', 
                color: '#ffa500',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                justifyContent: 'center',
                height: '100%'
            }}>
                <div style={{ fontSize: '3rem', marginBottom: '1rem' }}>🔌</div>
                <h3>Not Connected</h3>
                <p>Waiting for ROS connection...</p>
            </div>
        );
    }

    return (
        <div 
            ref={viewerRef} 
            style={{ 
                width: '100%', 
                height: '100%',
                overflow: 'hidden',
                position: 'relative'
            }}
        >
            {/* Loading indicator */}
            {!sceneRef.current && (
                <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: '50%',
                    transform: 'translate(-50%, -50%)',
                    color: '#9ca3af',
                    zIndex: 10
                }}>
                    Initializing 3D Scene...
                </div>
            )}
        </div>
    );
};

export default SceneViewer3D;