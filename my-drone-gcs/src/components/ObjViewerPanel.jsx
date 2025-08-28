import React, { useState, useEffect, useRef, useCallback } from 'react';
import * as THREE from 'three';
import { TrackballControls } from 'three/examples/jsm/controls/TrackballControls.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { FaFileUpload, FaSpinner, FaExclamationTriangle, FaCheckCircle } from 'react-icons/fa';

const ObjViewerPanel = () => {
    const mountRef = useRef(null);
    const fileInputRef = useRef(null);
    const sceneRef = useRef(new THREE.Scene());
    const cameraRef = useRef(null);
    const controlsRef = useRef(null);
    const loadedObjectRef = useRef(null);
    const fileMapRef = useRef(new Map());

    const [status, setStatus] = useState('idle'); // idle, loading, success, error
    const [statusMessage, setStatusMessage] = useState('Click "Load Map Files" to begin.');

    // --- 1. Setup the 3D Scene ---
    const setupScene = useCallback(() => {
        const mount = mountRef.current;

        // Renderer
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(mount.clientWidth, mount.clientHeight);
        renderer.setClearColor(0x1e2236); // A dark blue background
        mount.appendChild(renderer.domElement);

        // Camera
        cameraRef.current = new THREE.PerspectiveCamera(75, mount.clientWidth / mount.clientHeight, 0.1, 1000);
        cameraRef.current.position.set(10, 10, 10);
        sceneRef.current.add(cameraRef.current);

        // Controls (for zoom and pan)
        controlsRef.current = new TrackballControls(cameraRef.current, renderer.domElement);
        controlsRef.current.rotateSpeed = 2.5;
        controlsRef.current.zoomSpeed = 1.2;
        controlsRef.current.panSpeed = 0.8;
        controlsRef.current.noZoom = false;
        controlsRef.current.noPan = false;
        controlsRef.current.staticMoving = true;
        controlsRef.current.dynamicDampingFactor = 0.3;

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        sceneRef.current.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(50, 50, 50);
        sceneRef.current.add(directionalLight);
        
        // Ground Grid
        const gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x333333);
        sceneRef.current.add(gridHelper);

        // Render Loop
        let frameId = null;
        const animate = () => {
            controlsRef.current.update();
            renderer.render(sceneRef.current, cameraRef.current);
            frameId = requestAnimationFrame(animate);
        };
        animate();

        // Handle Resize
        const handleResize = () => {
            if (!mount || !cameraRef.current) return;
            cameraRef.current.aspect = mount.clientWidth / mount.clientHeight;
            cameraRef.current.updateProjectionMatrix();
            renderer.setSize(mount.clientWidth, mount.clientHeight);
            controlsRef.current.handleResize();
        };
        window.addEventListener('resize', handleResize);

        // Cleanup on unmount
        return () => {
            window.removeEventListener('resize', handleResize);
            cancelAnimationFrame(frameId);
            if(mount && renderer.domElement) mount.removeChild(renderer.domElement);
            controlsRef.current.dispose();
        };
    }, []);

    useEffect(() => {
        // Run setup once when component mounts
        if (mountRef.current) {
            return setupScene();
        }
    }, [setupScene]);


    // --- 2. Handle File Loading ---
    const handleFileChange = (event) => {
        const files = event.target.files;
        if (!files || files.length === 0) return;

        setStatus('loading');
        setStatusMessage('Processing files...');

        // Clean up old files to prevent memory leaks
        for (const url of fileMapRef.current.values()) {
            URL.revokeObjectURL(url);
        }
        fileMapRef.current.clear();

        // Create a map of filenames to blob URLs
        let objFile = null;
        let mtlFile = null;

        for (const file of files) {
            fileMapRef.current.set(file.name, URL.createObjectURL(file));
            if (file.name.toLowerCase().endsWith('.obj')) objFile = file.name;
            if (file.name.toLowerCase().endsWith('.mtl')) mtlFile = file.name;
        }

        if (!objFile) {
            setStatus('error');
            setStatusMessage('No .obj file found in selection.');
            return;
        }

        // --- Setup Loading Manager to handle all files ---
        const manager = new THREE.LoadingManager();
        manager.setURLModifier((url) => {
            const fileName = url.split('/').pop();
            return fileMapRef.current.get(fileName) || url;
        });

        // If an MTL file is present, load it first
        if (mtlFile) {
            setStatusMessage('Loading materials...');
            const mtlLoader = new MTLLoader(manager);
            mtlLoader.load(mtlFile, (materials) => {
                materials.preload();
                const objLoader = new OBJLoader(manager);
                objLoader.setMaterials(materials);
                setStatusMessage('Loading geometry...');
                objLoader.load(objFile, onLoadSuccess, onProgress, onLoadError);
            }, onProgress, onLoadError);
        } else {
            // Otherwise, load the OBJ with a default material
            const objLoader = new OBJLoader(manager);
            setStatusMessage('Loading geometry (no material file)...');
            objLoader.load(objFile, onLoadSuccess, onProgress, onLoadError);
        }
    };

    // 3. Loader Callbacks
    const onLoadSuccess = (object) => {
        // Remove previous model
        if (loadedObjectRef.current) {
            sceneRef.current.remove(loadedObjectRef.current);
        }

        // If no MTL was provided, apply a default material
        let hasMaterial = false;
        object.traverse(child => {
            if(child instanceof THREE.Mesh && child.material) hasMaterial = true;
        });
        if (!hasMaterial) {
             const defaultMaterial = new THREE.MeshStandardMaterial({ color: 0xcccccc, roughness: 0.8 });
             object.traverse(child => { if(child instanceof THREE.Mesh) child.material = defaultMaterial; });
        }

        object.rotation.x = -Math.PI / 2;

        sceneRef.current.add(object);
        loadedObjectRef.current = object;
        
        // Frame camera
        frameCameraToObject(object);

        setStatus('success');
        setStatusMessage('Map loaded successfully!');
    };

    const onProgress = (xhr) => {
        if (xhr.lengthComputable) {
            const percentComplete = (xhr.loaded / xhr.total) * 100;
            setStatusMessage(`Loading... ${Math.round(percentComplete)}%`);
        }
    };
    
    const onLoadError = (error) => {
        console.error("An error happened during loading:", error);
        setStatus('error');
        setStatusMessage("Failed to load model. Check console for details.");
    };

    const frameCameraToObject = (object) => {
        const box = new THREE.Box3().setFromObject(object);
        const center = box.getCenter(new THREE.Vector3());
        const size = box.getSize(new THREE.Vector3());
        const camera = cameraRef.current;
        const controls = controlsRef.current;
        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = camera.fov * (Math.PI / 180);
        let cameraZ = Math.abs(maxDim / 1.5 / Math.tan(fov / 2));
        camera.position.copy(center);
        camera.position.z += cameraZ;
        camera.lookAt(center);
        if (controls) {
            controls.target.copy(center);
            controls.update();
        }
    };
    
    // --- 4. Render the UI ---
    return (
        <div style={{ width: '100%', height: '100%', position: 'relative', background: '#1e2236' }}>
            <div ref={mountRef} style={{ width: '100%', height: '100%' }} />
            <div style={{ position: 'absolute', top: '1rem', right: '1rem', zIndex: 10 }}>
                {/* --- FIX: Input now accepts multiple files --- */}
                <input type="file" ref={fileInputRef} onChange={handleFileChange} multiple style={{ display: 'none' }} />
                <button onClick={() => fileInputRef.current.click()} style={{ background: '#7c3aed', color: 'white', border: 'none', padding: '0.6rem 1rem', borderRadius: '8px', cursor: 'pointer', display: 'flex', alignItems: 'center', gap: '0.5rem', fontWeight: '600' }}>
                    <FaFileUpload /> Load Map Files
                </button>
            </div>
            {status !== 'success' && (
                <div style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', background: 'rgba(30, 34, 54, 0.8)', color: '#e6e9f0', zIndex: 5, pointerEvents: 'none' }}>
                    {status === 'idle' && <>
                        <FaFileUpload style={{ fontSize: '3rem', marginBottom: '1rem' }} />
                        <h3>OBJ Map Viewer</h3>
                        <p>Select the .obj, .mtl, and any texture files.</p>
                    </>}
                    {(status === 'loading') && <>
                        <FaSpinner style={{ fontSize: '3rem', marginBottom: '1rem', animation: 'spin 1.5s linear infinite' }} />
                        <h3>{statusMessage}</h3>
                    </>}
                    {status === 'error' && <>
                        <FaExclamationTriangle style={{ fontSize: '3rem', marginBottom: '1rem', color: '#ff6b6b' }} />
                        <h3>Error</h3>
                        <p>{statusMessage}</p>
                    </>}
                </div>
            )}
            {status === 'success' && (
                 <div style={{ position: 'absolute', bottom: '1rem', left: '1rem', background: 'rgba(30, 34, 54, 0.8)', padding: '0.5rem 1rem', borderRadius: '8px', color: '#4ade80', display: 'flex', alignItems: 'center', gap: '0.5rem', zIndex: 5}}>
                    <FaCheckCircle /> {statusMessage}
                 </div>
            )}
        </div>
    );
};

export default ObjViewerPanel;