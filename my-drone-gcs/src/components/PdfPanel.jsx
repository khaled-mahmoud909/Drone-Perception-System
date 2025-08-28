import React, { useState, useEffect, useRef, useContext } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import * as THREE from 'three';

// Helper Functions
const getEulerAngles = (q) => {
    const quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);
    const euler = new THREE.Euler().setFromQuaternion(quaternion, 'ZYX');
    return {
        roll: euler.y,
        pitch: euler.x,
        yaw: euler.z
    };
};

// Enhanced Attitude Indicator with pitch scale
const AttitudeIndicator = ({ roll, pitch }) => {
    const pitchPixelsPerDegree = 2.5;
    const pitchOffset = pitch * (180 / Math.PI) * pitchPixelsPerDegree;
    const rollDegrees = roll * (180 / Math.PI);

    // The moving horizon background (sky/ground) rotates and translates
    const horizonBackgroundStyle = {
        transform: `rotate(${-rollDegrees}deg) translateY(${pitchOffset}px)`,
        transition: 'transform 0.1s linear',
        position: 'absolute',
        width: '400%',
        height: '400%',
        top: '-150%',
        left: '-150%',
        background: 'linear-gradient(to bottom, #4a90e2 50%, #8b572a 50%)'
    };

    // The horizon line itself - should be straight when aircraft is level
    const horizonLineStyle = {
        position: 'absolute',
        top: '50%',
        left: '0',
        width: '100%',
        height: '4px',
        background: '#fff',
        transform: 'translateY(-50%)',
        boxShadow: '0 0 4px rgba(0,0,0,0.5)'
    };

    // Generate pitch scale lines that move with the horizon
    const pitchLines = [];
    for (let i = -90; i <= 90; i += 10) {
        if (i === 0) continue; // Skip horizon line
        const lineOffset = -i * pitchPixelsPerDegree;
        const isPositive = i > 0;
        const lineWidth = i % 30 === 0 ? '60px' : i % 20 === 0 ? '40px' : '30px';
        
        pitchLines.push(
            <div
                key={i}
                style={{
                    position: 'absolute',
                    left: '50%',
                    top: `calc(50% + ${lineOffset}px)`,
                    transform: 'translateX(-50%)',
                    width: lineWidth,
                    height: '2px',
                    background: '#fff',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center'
                }}
            >
                {i % 20 === 0 && (
                    <>
                        <span style={{
                            position: 'absolute',
                            left: '-25px',
                            fontSize: '12px',
                            color: '#fff',
                            background: 'rgba(0,0,0,0.5)',
                            padding: '1px 3px',
                            borderRadius: '2px'
                        }}>
                            {Math.abs(i)}
                        </span>
                        <span style={{
                            position: 'absolute',
                            right: '-25px',
                            fontSize: '12px',
                            color: '#fff',
                            background: 'rgba(0,0,0,0.5)',
                            padding: '1px 3px',
                            borderRadius: '2px'
                        }}>
                            {Math.abs(i)}
                        </span>
                    </>
                )}
            </div>
        );
    }

    return (
        <div style={{
            width: '220px',
            height: '220px',
            borderRadius: '50%',
            overflow: 'hidden',
            position: 'relative',
            border: '3px solid #333',
            boxShadow: 'inset 0 0 20px rgba(0,0,0,0.3), 0 0 15px rgba(0,0,0,0.5)',
        }}>
            {/* Moving horizon background and pitch scale */}
            <div style={{
                transform: `rotate(${-rollDegrees}deg) translateY(${pitchOffset}px)`,
                transition: 'transform 0.1s linear',
                position: 'absolute',
                width: '400%',
                height: '400%',
                top: '-150%',
                left: '-150%',
            }}>
                {/* Sky/Ground background */}
                <div style={{
                    position: 'absolute',
                    width: '100%',
                    height: '100%',
                    background: 'linear-gradient(to bottom, #4a90e2 50%, #8b572a 50%)'
                }}></div>
                
                {/* Horizon line - this rotates with the background */}
                <div style={horizonLineStyle}></div>
                
                {/* Pitch scale lines - these also rotate with the background */}
                {pitchLines}
            </div>
            
            {/* Aircraft symbol (fixed in center, doesn't move) */}
            <div style={{
                position: 'absolute',
                top: '50%',
                left: '50%',
                width: '120px',
                height: '30px',
                transform: 'translate(-50%, -50%)',
                zIndex: 3
            }}>
                {/* Main wing bars */}
                <div style={{ position: 'absolute', left: 0, top: '14px', width: '40px', height: '3px', background: '#ffff00', border: '1px solid #000' }}></div>
                <div style={{ position: 'absolute', right: 0, top: '14px', width: '40px', height: '3px', background: '#ffff00', border: '1px solid #000' }}></div>
                {/* Center dot */}
                <div style={{ position: 'absolute', left: '58px', top: '12.5px', width: '4px', height: '4px', background: '#ffff00', borderRadius: '50%', border: '1px solid #000' }}></div>
            </div>

            {/* Roll scale (fixed, doesn't rotate) */}
            <div style={{
                position: 'absolute',
                top: '10px',
                left: '50%',
                transform: 'translateX(-50%)',
                width: '160px',
                height: '30px',
                zIndex: 4
            }}>
                {[-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60].map(angle => (
                    <div
                        key={angle}
                        style={{
                            position: 'absolute',
                            left: `${50 + (angle / 60) * 40}%`,
                            top: '0',
                            transform: 'translateX(-50%)',
                            width: '2px',
                            height: Math.abs(angle) % 30 === 0 ? '20px' : '15px',
                            background: '#fff',
                            transformOrigin: 'bottom'
                        }}
                    >
                        {Math.abs(angle) % 30 === 0 && (
                            <span style={{
                                position: 'absolute',
                                top: '-18px',
                                left: '50%',
                                transform: 'translateX(-50%)',
                                fontSize: '10px',
                                color: '#fff',
                                fontWeight: 'bold'
                            }}>
                                {Math.abs(angle)}
                            </span>
                        )}
                    </div>
                ))}
                {/* Roll indicator triangle - moves with roll angle */}
                <div style={{
                    position: 'absolute',
                    left: `${50 + (rollDegrees / 60) * 40}%`,
                    top: '30px',
                    transform: 'translateX(-50%)',
                    width: '0',
                    height: '0',
                    borderLeft: '6px solid transparent',
                    borderRight: '6px solid transparent',
                    borderTop: '12px solid #ffff00',
                    filter: 'drop-shadow(0 1px 2px rgba(0,0,0,0.5))'
                }}></div>
            </div>
        </div>
    );
};

// Enhanced Tape with better styling and vertical speed indicator
const Tape = ({ value, label, unit, isHorizontal = false, showVerticalSpeed = false, verticalSpeed = 0 }) => {
    const pixelsPerUnit = isHorizontal ? 20 : 30;
    const numMarkers = 300;
    const tapeOffset = -value * pixelsPerUnit;

    const tapeStyle = {
        position: 'absolute',
        transition: 'transform 0.1s linear',
        fontFamily: "'Roboto Mono', monospace",
    };

    if (isHorizontal) {
        tapeStyle.transform = `translateX(${tapeOffset}px)`;
        tapeStyle.display = 'flex';
        tapeStyle.left = '50%';
        tapeStyle.top = '50%';
        tapeStyle.transform = `translate(-50%, -50%) translateX(${tapeOffset}px)`;
    } else {
        tapeStyle.transform = `translateY(${tapeOffset}px)`;
        tapeStyle.top = '50%';
        tapeStyle.left = '0';
        tapeStyle.transform = `translateY(-50%) translateY(${tapeOffset}px)`;
    }

    const markers = [];
    for (let i = -numMarkers / 2; i <= numMarkers / 2; i++) {
        const val = Math.round(value) + i;
        const increment = isHorizontal ? (label === 'HEADING' ? 5 : 1) : (label === 'SPEED' ? 5 : 10);
        const isMajor = val % (increment * 2) === 0;
        const isMinor = val % increment === 0;
        
        if (!isMinor) continue;

        const markerStyle = {
            position: 'relative',
            color: '#e0e0e0',
            fontSize: isMajor ? '14px' : '12px',
            fontWeight: isMajor ? 'bold' : 'normal',
        };

        if (isHorizontal) {
            markerStyle.width = `${pixelsPerUnit * increment}px`;
            markerStyle.height = '35px';
            markerStyle.textAlign = 'center';
            markerStyle.borderLeft = '1px solid #666';
            markerStyle.display = 'flex';
            markerStyle.alignItems = 'center';
            markerStyle.justifyContent = 'center';
        } else {
            markerStyle.height = `${pixelsPerUnit * increment}px`;
            markerStyle.width = '70px';
            markerStyle.textAlign = 'right';
            markerStyle.borderBottom = '1px solid #666';
            markerStyle.paddingRight = '8px';
            markerStyle.display = 'flex';
            markerStyle.alignItems = 'center';
            markerStyle.justifyContent = 'flex-end';
        }

        let displayVal = val;
        if (label === 'HEADING') {
            displayVal = ((val % 360) + 360) % 360;
            if (displayVal === 0) displayVal = 360;
        }

        markers.push(
            <div key={val} style={markerStyle}>
                {isMajor ? displayVal : (isMinor ? '•' : '')}
            </div>
        );
    }

    return (
        <div style={{
            position: 'relative',
            background: 'rgba(0,0,0,0.6)',
            overflow: 'hidden',
            color: 'white',
            border: '2px solid #444',
            borderRadius: '12px',
            backdropFilter: 'blur(8px)',
            height: isHorizontal ? '80px' : '280px',
            width: isHorizontal ? '350px' : '120px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.3)'
        }}>
            {/* Label */}
            <div style={{
                position: 'absolute',
                top: '8px',
                left: '50%',
                transform: 'translateX(-50%)',
                background: 'rgba(0,0,0,0.8)',
                padding: '4px 12px',
                borderRadius: '6px',
                fontSize: '12px',
                fontWeight: 'bold',
                zIndex: 3,
                border: '1px solid #666'
            }}>{label}</div>

            {/* Tape markers */}
            <div style={tapeStyle}>{markers}</div>

            {/* Center reference line and value box */}
            <div style={{
                position: 'absolute',
                top: isHorizontal ? '50%' : '50%',
                left: isHorizontal ? '50%' : '0',
                transform: isHorizontal ? 'translate(-50%, -50%)' : 'translate(0, -50%)',
                width: isHorizontal ? '3px' : '100%',
                height: isHorizontal ? '60px' : '3px',
                background: '#00ffff',
                zIndex: 2,
                boxShadow: '0 0 8px #00ffff'
            }}>
                {/* Value display box */}
                <div style={{
                    position: 'absolute',
                    top: isHorizontal ? 'calc(100% + 8px)' : '50%',
                    left: isHorizontal ? '50%' : 'calc(100% + 8px)',
                    transform: isHorizontal ? 'translateX(-50%)' : 'translateY(-50%)',
                    background: 'rgba(0,0,0,0.9)',
                    color: '#00ffff',
                    padding: '6px 12px',
                    borderRadius: '6px',
                    border: '2px solid #00ffff',
                    fontSize: '18px',
                    fontWeight: 'bold',
                    fontFamily: "'Roboto Mono', monospace",
                    minWidth: '80px',
                    textAlign: 'center',
                    boxShadow: '0 2px 8px rgba(0,0,0,0.5)'
                }}>
                    {label === 'HEADING' ? 
                        `${Math.round(((value % 360) + 360) % 360).toString().padStart(3, '0')}°` :
                        `${value.toFixed(1)} ${unit}`
                    }
                </div>
            </div>

            {/* Vertical Speed Indicator (for altitude tape only) */}
            {showVerticalSpeed && (
                <div style={{
                    position: 'absolute',
                    right: '-50px',
                    top: '50%',
                    transform: 'translateY(-50%)',
                    width: '40px',
                    height: '120px',
                    background: 'rgba(0,0,0,0.8)',
                    border: '2px solid #666',
                    borderRadius: '8px',
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    justifyContent: 'center'
                }}>
                    <div style={{
                        fontSize: '10px',
                        color: '#ccc',
                        marginBottom: '4px'
                    }}>VS</div>
                    <div style={{
                        fontSize: '14px',
                        fontWeight: 'bold',
                        color: Math.abs(verticalSpeed) > 5 ? '#ff6b6b' : '#4ecdc4'
                    }}>
                        {verticalSpeed > 0 ? '+' : ''}{verticalSpeed.toFixed(1)}
                    </div>
                    <div style={{
                        fontSize: '10px',
                        color: '#ccc',
                        marginTop: '2px'
                    }}>m/s</div>
                </div>
            )}
        </div>
    );
};

// Status indicators
const StatusIndicators = ({ telemetry, isConnected, hasData }) => {
    const indicators = [
        { label: 'IMU', status: hasData ? 'OK' : 'FAIL', color: hasData ? '#4ecdc4' : '#ff6b6b' },
        { label: 'LINK', status: isConnected ? 'OK' : 'LOST', color: isConnected ? '#4ecdc4' : '#ff6b6b' }
    ];

    return (
        <div style={{
            display: 'flex',
            gap: '12px',
            marginTop: '16px'
        }}>
            {indicators.map(indicator => (
                <div key={indicator.label} style={{
                    padding: '6px 12px',
                    background: 'rgba(0,0,0,0.7)',
                    border: `2px solid ${indicator.color}`,
                    borderRadius: '8px',
                    fontSize: '12px',
                    fontWeight: 'bold',
                    color: indicator.color,
                    textAlign: 'center',
                    minWidth: '50px'
                }}>
                    <div>{indicator.label}</div>
                    <div style={{ fontSize: '10px', opacity: 0.8 }}>{indicator.status}</div>
                </div>
            ))}
        </div>
    );
};

const PfdPanel = () => {
    const { ros, isConnected } = useContext(RosContext);
    
    const [telemetry, setTelemetry] = useState({
        roll: 0, 
        pitch: 0, 
        yaw: 0,
        altitude: 0, 
        speed: 0,
        verticalSpeed: 0
    });
    const [hasData, setHasData] = useState(false);

    useEffect(() => {
        let odomListener = null;

        if (isConnected && ros) {
            console.log("Enhanced PFD: Subscribing to /fc/odom");
            odomListener = new ROSLIB.Topic({
                ros: ros,
                name: '/fc/odom',
                messageType: 'nav_msgs/Odometry'
            });

            odomListener.subscribe((message) => {
                const { pose, twist } = message;
                const { roll, pitch, yaw } = getEulerAngles(pose.pose.orientation);
                
                const speed = Math.sqrt(
                    Math.pow(twist.twist.linear.x, 2) +
                    Math.pow(twist.twist.linear.y, 2)
                );

                setTelemetry(prevTelemetry => ({
                    ...prevTelemetry,
                    roll,
                    pitch,
                    yaw,
                    altitude: pose.pose.position.z,
                    speed,
                    verticalSpeed: twist.twist.linear.z
                }));
                setHasData(true);
            });
        } else {
            console.log("Enhanced PFD: Not connected to ROS");
            setHasData(false);
        }

        return () => {
            if (odomListener) {
                console.log("Enhanced PFD: Unsubscribing from /fc/odom");
                odomListener.unsubscribe();
            }
        };
    }, [isConnected, ros]);

    const yawDegrees = (telemetry.yaw * 180 / Math.PI + 360) % 360;

    return (
        <div style={{
            width: '100%',
            height: '100%',
            background: 'linear-gradient(135deg, #1a1d2b 0%, #2d3748 100%)',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            padding: '1rem',
            gap: '1rem',
            color: 'white',
            fontFamily: "'Segoe UI', Roboto, Helvetica, Arial, sans-serif",
            minHeight: '500px',
            position: 'relative'
        }}>
            {/* Main container for PFD and raw data */}
            <div style={{
                display: 'flex',
                width: '100%',
                gap: '2rem',
                alignItems: 'flex-start',
                justifyContent: 'center'
            }}>
                {/* Left side - Main PFD instruments */}
                <div style={{ 
                    display: 'flex', 
                    flexDirection: 'column',
                    alignItems: 'center',
                    gap: '1rem'
                }}>
                    <div style={{ 
                        display: 'flex', 
                        alignItems: 'center', 
                        justifyContent: 'center', 
                        gap: '2rem',
                        flexWrap: 'wrap'
                    }}>
                        <Tape 
                            value={telemetry.speed} 
                            label="AIRSPEED" 
                            unit="m/s" 
                        />
                        <AttitudeIndicator 
                            roll={telemetry.roll} 
                            pitch={telemetry.pitch} 
                        />
                        <Tape 
                            value={telemetry.altitude} 
                            label="ALTITUDE" 
                            unit="m"
                            showVerticalSpeed={true}
                            verticalSpeed={telemetry.verticalSpeed}
                        />
                    </div>
                    
                    {/* Heading tape */}
                    <Tape 
                        value={yawDegrees} 
                        label="HEADING" 
                        unit="°" 
                        isHorizontal={true} 
                    />

                    {/* Status indicators */}
                    <StatusIndicators telemetry={telemetry} isConnected={isConnected} hasData={hasData} />
                </div>

                {/* Right side - Raw telemetry data */}
                <div style={{
                    background: 'rgba(0,0,0,0.7)',
                    border: '2px solid #444',
                    borderRadius: '12px',
                    padding: '16px',
                    minWidth: '200px',
                    backdropFilter: 'blur(8px)'
                }}>
                    <div style={{
                        fontSize: '14px',
                        fontWeight: 'bold',
                        marginBottom: '12px',
                        color: '#00ffff',
                        textAlign: 'center',
                        borderBottom: '1px solid #444',
                        paddingBottom: '8px'
                    }}>
                        RAW TELEMETRY
                    </div>
                    
                    <div style={{ fontSize: '12px', fontFamily: "'Roboto Mono', monospace" }}>
                        <div style={{ marginBottom: '8px' }}>
                            <div style={{ color: '#ccc', marginBottom: '2px' }}>Roll:</div>
                            <div style={{ color: '#fff', marginLeft: '8px' }}>{(telemetry.roll * 180 / Math.PI).toFixed(2)}°</div>
                        </div>
                        
                        <div style={{ marginBottom: '8px' }}>
                            <div style={{ color: '#ccc', marginBottom: '2px' }}>Pitch:</div>
                            <div style={{ color: '#fff', marginLeft: '8px' }}>{(telemetry.pitch * 180 / Math.PI).toFixed(2)}°</div>
                        </div>
                        
                        <div style={{ marginBottom: '8px' }}>
                            <div style={{ color: '#ccc', marginBottom: '2px' }}>Yaw:</div>
                            <div style={{ color: '#fff', marginLeft: '8px' }}>{(telemetry.yaw * 180 / Math.PI).toFixed(2)}°</div>
                        </div>
                        
                        <div style={{ marginBottom: '8px' }}>
                            <div style={{ color: '#ccc', marginBottom: '2px' }}>Altitude:</div>
                            <div style={{ color: '#fff', marginLeft: '8px' }}>{telemetry.altitude.toFixed(3)} m</div>
                        </div>
                        
                        <div style={{ marginBottom: '8px' }}>
                            <div style={{ color: '#ccc', marginBottom: '2px' }}>Speed:</div>
                            <div style={{ color: '#fff', marginLeft: '8px' }}>{telemetry.speed.toFixed(3)} m/s</div>
                        </div>
                        
                        <div style={{ marginBottom: '8px' }}>
                            <div style={{ color: '#ccc', marginBottom: '2px' }}>Vertical Speed:</div>
                            <div style={{ color: '#fff', marginLeft: '8px' }}>{telemetry.verticalSpeed.toFixed(3)} m/s</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default PfdPanel;