import React, { useState, useEffect, useContext, useRef } from 'react';
import { RosContext } from '../RosConnection';
import ROSLIB from 'roslib';
import { FaPowerOff, FaArrowUp, FaArrowDown, FaExclamationTriangle, FaCheckCircle, FaSpinner, FaCompass } from 'react-icons/fa';

// CommandButton component remains the same...
const CommandButton = ({ icon, label, onClick, status, disabled, color, hoverColor }) => {
    const baseStyle = {
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        width: '120px',
        height: '100px',
        border: `2px solid ${color}`,
        borderRadius: '12px',
        background: `rgba(${parseInt(color.slice(1,3),16)}, ${parseInt(color.slice(3,5),16)}, ${parseInt(color.slice(5,7),16)}, 0.1)`,
        color: color,
        cursor: disabled ? 'not-allowed' : 'pointer',
        transition: 'all 0.2s ease',
        opacity: disabled ? 0.4 : 1,
        position: 'relative',
    };

    const [style, setStyle] = useState(baseStyle);

    useEffect(() => {
        let finalStyle = {...baseStyle};
        if (status === 'pending') {
            finalStyle.color = '#ffffff';
            finalStyle.background = `rgba(${parseInt(hoverColor.slice(1,3),16)}, ${parseInt(hoverColor.slice(3,5),16)}, ${parseInt(hoverColor.slice(5,7),16)}, 0.5)`;
        }
        setStyle(finalStyle);
    }, [status, disabled, color, hoverColor]);


    return (
        <button
            onClick={onClick}
            disabled={disabled || status === 'pending'}
            style={style}
            onMouseEnter={() => !disabled && setStyle(prev => ({...prev, background: `rgba(${parseInt(hoverColor.slice(1,3),16)}, ${parseInt(hoverColor.slice(3,5),16)}, ${parseInt(hoverColor.slice(5,7),16)}, 0.3)`}))}
            onMouseLeave={() => setStyle(baseStyle)}
        >
            <div style={{ fontSize: '2.5rem', marginBottom: '0.5rem' }}>
                {status === 'pending' ? <FaSpinner className="fa-spin" /> : icon}
            </div>
            <div style={{ fontWeight: 'bold', fontSize: '1rem' }}>{label}</div>
        </button>
    );
};


const CommandPanel = () => {
    const { ros, isConnected } = useContext(RosContext);
    const [isArmed, setIsArmed] = useState(false);
    const [commandStatus, setCommandStatus] = useState({ arm: 'idle', takeoff: 'idle', land: 'idle', explore: 'idle' });
    const [statusMessage, setStatusMessage] = useState({ text: 'Ready for commands.', type: 'info' });
    const [takeoffAltitude, setTakeoffAltitude] = useState(2.5);

    const serviceClients = useRef({ arming: null, takeoff: null, land: null, explore: null });

    // Subscribe to armed status
    useEffect(() => {
        if (!isConnected || !ros) return;
        const armedListener = new ROSLIB.Topic({ ros, name: '/fc/armed_status', messageType: 'std_msgs/Bool' });
        armedListener.subscribe(message => setIsArmed(message.data));
        return () => armedListener.unsubscribe();
    }, [isConnected, ros]);

    // Setup ROS service clients
    useEffect(() => {
        if (!isConnected || !ros) return;
        
        // --- ADDED FOR DEBUGGING ---
        console.log("Setting up ROS Service clients...");

        serviceClients.current = {
            arming: new ROSLIB.Service({ ros, name: '/mavros/cmd/arming', serviceType: 'mavros_msgs/CommandBool' }),
            takeoff: new ROSLIB.Service({ ros, name: '/mavros/cmd/takeoff', serviceType: 'mavros_msgs/CommandTOL' }),
            land: new ROSLIB.Service({ ros, name: '/mavros/cmd/land', serviceType: 'mavros_msgs/CommandTOL' }),
            explore: new ROSLIB.Service({ ros, name: '/my_drone/start_exploration_command', serviceType: 'std_srvs/Trigger' })
        };
        
        // --- ADDED FOR DEBUGGING ---
        console.log("'/start_exploration' client created.");

    }, [isConnected, ros]);

    const handleCommand = (cmd, request) => {
        // --- ADDED FOR DEBUGGING ---
        console.log(`handleCommand triggered for: ${cmd}`);

        const client = serviceClients.current[cmd];
        if (!client) {
            console.error(`Service client for '${cmd}' not available.`);
            setStatusMessage({ text: `Service client for '${cmd}' not available.`, type: 'error' });
            return;
        }

        setCommandStatus(prev => ({ ...prev, [cmd]: 'pending' }));
        setStatusMessage({ text: `Sending ${cmd} command...`, type: 'info' });

        const serviceRequest = new ROSLIB.ServiceRequest(request);

        client.callService(serviceRequest, (response) => {
            console.log(`Service response for '${cmd}':`, response);
            const message = response.message || `${cmd.charAt(0).toUpperCase() + cmd.slice(1)} command successful.`;
            if (response.success) {
                setCommandStatus(prev => ({ ...prev, [cmd]: 'success' }));
                setStatusMessage({ text: message, type: 'success' });
            } else {
                setCommandStatus(prev => ({ ...prev, [cmd]: 'error' }));
                setStatusMessage({ text: `Command '${cmd}' failed: ${message}`, type: 'error' });
            }
            setTimeout(() => setCommandStatus(prev => ({ ...prev, [cmd]: 'idle' })), 3000);
        }, (error) => {
            console.error(`Service call for '${cmd}' failed:`, error);
            setStatusMessage({ text: `Service call for '${cmd}' failed: ${error}`, type: 'error' });
            setTimeout(() => setCommandStatus(prev => ({ ...prev, [cmd]: 'idle' })), 3000);
        });
    };

    const getStatusColor = () => {
        switch(statusMessage.type) {
            case 'success': return '#4ade80';
            case 'error': return '#f87171';
            default: return '#9ca3af';
        }
    };

    return (
        <div style={{ width: '100%', height: '100%', background: '#1a1d2b', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', padding: '1rem', gap: '1.5rem' }}>
            <style>{`.fa-spin { animation: spin 1s linear infinite; } @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }`}</style>
            
            <div style={{ display: 'flex', gap: '1.5rem', flexWrap: 'wrap', justifyContent: 'center' }}>
                <CommandButton
                    icon={<FaPowerOff />}
                    label={isArmed ? "DISARM" : "ARM"}
                    onClick={() => handleCommand('arming', { value: !isArmed })}
                    status={commandStatus.arming}
                    disabled={!isConnected}
                    color={isArmed ? '#facc15' : '#4ade80'}
                    hoverColor={isArmed ? '#fde047' : '#86efac'}
                />
                <CommandButton
                    icon={<FaArrowUp />}
                    label="TAKEOFF"
                    onClick={() => handleCommand('takeoff', { altitude: takeoffAltitude, latitude: 0, longitude: 0, min_pitch: 0, yaw: 0 })}
                    status={commandStatus.takeoff}
                    disabled={!isConnected || !isArmed}
                    color="#38bdf8"
                    hoverColor="#7dd3fc"
                />
                <CommandButton
                    icon={<FaCompass />}
                    label="EXPLORE"
                    onClick={() => {
                        // --- ADDED FOR DEBUGGING ---
                        console.log("Explore button CLICKED.");
                        handleCommand('explore', {});
                    }}
                    status={commandStatus.explore}
                    disabled={!isConnected}
                    color="#2dd4bf"
                    hoverColor="#5eead4"
                />
                <CommandButton
                    icon={<FaArrowDown />}
                    label="LAND"
                    onClick={() => handleCommand('land', { altitude: 0, latitude: 0, min_pitch: 0, yaw: 0 })}
                    status={commandStatus.land}
                    disabled={!isConnected || !isArmed}
                    color="#f472b6"
                    hoverColor="#f9a8d4"
                />
            </div>

            {/* The rest of the component remains the same... */}
            <div style={{ display: 'flex', alignItems: 'center', gap: '1rem', background: 'rgba(0,0,0,0.3)', padding: '0.5rem 1rem', borderRadius: '8px' }}>
                <label htmlFor="takeoff-alt" style={{ color: '#9ca3af', fontSize: '0.9rem' }}>Takeoff Alt (m):</label>
                <input
                    id="takeoff-alt"
                    type="number"
                    value={takeoffAltitude}
                    onChange={(e) => setTakeoffAltitude(parseFloat(e.target.value))}
                    min="1"
                    max="20"
                    step="0.5"
                    style={{ background: 'rgba(255,255,255,0.1)', border: '1px solid rgba(255,255,255,0.2)', color: 'white', borderRadius: '4px', padding: '0.3rem', width: '70px', textAlign: 'center' }}
                />
            </div>

            <div style={{
                position: 'absolute',
                bottom: '1rem',
                left: '50%',
                transform: 'translateX(-50%)',
                background: 'rgba(0,0,0,0.5)',
                padding: '0.5rem 1rem',
                borderRadius: '8px',
                fontSize: '0.9rem',
                color: getStatusColor(),
                display: 'flex',
                alignItems: 'center',
                gap: '0.5rem',
                border: `1px solid ${getStatusColor()}`
            }}>
                {statusMessage.type === 'success' && <FaCheckCircle />}
                {statusMessage.type === 'error' && <FaExclamationTriangle />}
                {statusMessage.text}
            </div>
        </div>
    );
};

export default CommandPanel;