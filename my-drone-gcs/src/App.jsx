import React, { useState, useEffect, useRef, useCallback } from 'react';
import { FaRobot, FaCamera, FaCube, FaTimes, FaPlus, FaSignal, FaMap, FaGripVertical, FaSearchLocation, FaMapMarkedAlt, FaTachometerAlt, FaPaperPlane } from 'react-icons/fa';
import StatusBar from './components/StatusBar';
import ImageViewer from './components/ImageViewer';
import RtabmapViewer from './components/RtabmapViewer';
import SceneViewer3D from './components/SceneViewer3D';
import MapView from './components/MapViewer'; // Import the new MapView component
import ObjectDetectionPanel from './components/ObjectDetectionPanel';
import ObjViewerPanel from './components/ObjViewerPanel';
import DepthImageViewer from './components/DepthImageViewer';
import PfdPanel from './components/PdfPanel';
import CommandPanel from './components/CommandPanel';

const ALL_PANELS = [
  {
    key: 'pfd',
    title: 'Primary Flight Display',
    icon: <FaTachometerAlt />,
    content: <PfdPanel />,
    defaultWidth: 6,
    defaultHeight: 3
  },
  {
    key: 'command',
    title: 'Command & Control',
    icon: <FaPaperPlane />,
    content: <CommandPanel />,
    defaultWidth: 2,
    defaultHeight: 3
  },
  {
    key: 'scene_3d', // Updated key for the 3D viewer
    title: '3D Scene Viewer', // Updated title
    icon: <FaCube />,
    content: <SceneViewer3D />, // Use the new component
    defaultWidth: 4,
    defaultHeight: 3
  },

  {
    key: 'mapview', // Add the new MapView panel
    title: '2D Map View',
    icon: <FaMap />,
    content: <MapView />,
    defaultWidth: 4,
    defaultHeight: 3
  },
  {
    key: 'rgb',
    title: 'RGB Camera',
    icon: <FaCamera />,
    content: <ImageViewer title="RGB Camera" topicName="/oak/rgb/preview/image_raw/compressed" />,
    defaultWidth: 2,
    defaultHeight: 2
  },
  {
    key: 'depth',
    title: 'Depth Camera',
    icon: <FaCamera />,
    content: <DepthImageViewer title="Depth Camera" topicName="/oak/stereo/image_raw/compressedDepth" />,
    defaultWidth: 2,
    defaultHeight: 2
  },
  {
    key: 'object_detection',
    title: 'Object Detection',
    icon: <FaSearchLocation />,
    content: <ObjectDetectionPanel />,
    defaultWidth: 2,
    defaultHeight: 3
  },
  {
    key: 'obj_viewer',
    title: 'OBJ Map Viewer',
    icon: <FaMapMarkedAlt />,
    content: <ObjViewerPanel />,
    defaultWidth: 4,
    defaultHeight: 3
  },
  {
    key: 'rtabmap',
    title: 'RTAB-Map 3D',
    icon: <FaCube />,
    content: <RtabmapViewer />,
    defaultWidth: 4,
    defaultHeight: 3
  },
  {
    key: 'telemetry',
    title: 'Telemetry',
    icon: <FaSignal />,
    content: (
      <div style={{ padding: '1rem', height: '100%' }}>
        {/* ... content remains the same ... */}
        <div style={{ marginBottom: '1rem' }}>
          <div style={{ fontSize: '0.9rem', color: '#9ca3af', marginBottom: '0.5rem' }}>GPS Position</div>
          <div>Lat: 37.7749° N</div>
          <div>Lng: 122.4194° W</div>
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <div style={{ fontSize: '0.9rem', color: '#9ca3af', marginBottom: '0.5rem' }}>Flight Time</div>
          <div>00:12:34</div>
        </div>
        <div>
          <div style={{ fontSize: '0.9rem', color: '#9ca3af', marginBottom: '0.5rem' }}>Distance</div>
          <div>284.5m</div>
        </div>
      </div>
    ),
    defaultWidth: 1,
    defaultHeight: 2
  },
  {
    key: 'waypoints',
    title: 'Mission Plan',
    icon: <FaMap />,
    content: (
      <div style={{ padding: '1rem', height: '100%' }}>
        {/* ... content remains the same ... */}
        <div style={{ marginBottom: '1rem' }}>
          <div style={{ fontSize: '0.9rem', color: '#9ca3af', marginBottom: '0.5rem' }}>Active Mission</div>
          <div>Survey Pattern Alpha</div>
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <div style={{ fontSize: '0.9rem', color: '#9ca3af', marginBottom: '0.5rem' }}>Waypoints</div>
          <div>3 of 8 completed</div>
        </div>
        <div style={{
          background: 'rgba(74, 222, 128, 0.1)',
          padding: '0.5rem',
          borderRadius: '4px',
          border: '1px solid rgba(74, 222, 128, 0.2)'
        }}>
          Next: Waypoint 4
        </div>
      </div>
    ),
    defaultWidth: 1,
    defaultHeight: 1
  }
];

// ResizablePanel component remains the same
const ResizablePanel = ({
  panel,
  onRemove,
  onResize,
  width,
  height,
  maxColumns,
  maxRows,
  onDragStart,
  onDragOver,
  onDrop,
  isBeingDragged,
  isDragOver
}) => {
  const [isResizing, setIsResizing] = useState(false);
  const panelRef = useRef(null);
  const gridRef = useRef({
    cellWidth: 0,
    cellHeight: 0,
    gap: 24, // 1.5rem
    startWidth: 0,
    startHeight: 0,
    startX: 0,
    startY: 0
  });

  const handleResizeStart = (e) => {
    e.preventDefault();
    e.stopPropagation();

    const gridEl = panelRef.current?.parentElement;
    if (!gridEl) return;
    
    // Calculate cell dimensions based on the grid container
    const cellWidth = (gridEl.offsetWidth - (maxColumns - 1) * gridRef.current.gap) / maxColumns;
    const cellHeight = 150; // Based on grid-auto-rows min value

    gridRef.current = {
      ...gridRef.current,
      cellWidth,
      cellHeight,
      startWidth: width,
      startHeight: height,
      startX: e.clientX,
      startY: e.clientY,
    };

    setIsResizing(true);
  };

  useEffect(() => {
    const handleResize = (e) => {
      if (!isResizing) return;

      const { startX, startY, startWidth, startHeight, cellWidth, cellHeight, gap } = gridRef.current;
      const dx = e.clientX - startX;
      const dy = e.clientY - startY;

      const widthChange = Math.round(dx / (cellWidth + gap));
      const heightChange = Math.round(dy / (cellHeight + gap));

      let newWidth = Math.max(1, startWidth + widthChange);
      let newHeight = Math.max(1, startHeight + heightChange);
      
      newWidth = Math.min(newWidth, maxColumns);
      newHeight = Math.min(newHeight, maxRows);

      if (newWidth !== width || newHeight !== height) {
        onResize(panel.key, newWidth, newHeight);
      }
    };

    const handleResizeEnd = () => {
      setIsResizing(false);
      document.body.style.cursor = '';
    };

    if (isResizing) {
      document.body.style.cursor = 'se-resize';
      document.addEventListener('mousemove', handleResize);
      document.addEventListener('mouseup', handleResizeEnd);
    }

    return () => {
      document.removeEventListener('mousemove', handleResize);
      document.removeEventListener('mouseup', handleResizeEnd);
    };
  }, [isResizing, onResize, panel.key, width, height, maxColumns, maxRows]);
  
  const handleDragStart = (e) => {
      e.dataTransfer.setData('panelKey', panel.key);
      e.dataTransfer.effectAllowed = 'move';
      setTimeout(() => onDragStart(panel.key), 0);
  };
  
  const handleDragOver = (e) => {
    e.preventDefault();
    onDragOver(panel.key);
  };
  
  const handleDrop = (e) => {
    e.preventDefault();
    const draggedKey = e.dataTransfer.getData('panelKey');
    onDrop(draggedKey, panel.key);
  };


  return (
    <div
      ref={panelRef}
      onDragOver={handleDragOver}
      onDrop={handleDrop}
      style={{
        gridColumn: `span ${Math.min(width, maxColumns)}`,
        gridRow: `span ${Math.min(height, maxRows)}`,
        minHeight: `${150 * height + (height - 1) * 24}px`,
        background: 'rgba(30, 34, 54, 0.98)',
        borderRadius: '16px',
        boxShadow: '0 4px 24px rgba(0,0,0,0.22)',
        border: `1.5px solid ${isDragOver ? '#7c3aed' : 'rgba(255,255,255,0.07)'}`,
        backdropFilter: 'blur(10px)',
        display: 'flex',
        flexDirection: 'column',
        transition: 'all 0.2s ease',
        transform: isBeingDragged ? 'scale(0.95)' : 'scale(1)',
        opacity: isBeingDragged ? 0.5 : 1,
        position: 'relative',
        overflow: 'hidden' // Hide content that might overflow
      }}
    >
      {/* Panel Header */}
      <div
        draggable={true}
        onDragStart={handleDragStart}
        className="panel-header"
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '1rem 1.2rem 0.5rem 1.2rem',
          borderBottom: '1px solid rgba(255,255,255,0.04)',
          cursor: 'grab',
          flexShrink: 0
        }}
        onMouseDown={(e) => { e.currentTarget.style.cursor = 'grabbing'; }}
        onMouseUp={(e) => { e.currentTarget.style.cursor = 'grab'; }}
      >
        <span style={{
          display: 'flex',
          alignItems: 'center',
          gap: '0.5em',
          fontSize: '1.1rem',
          fontWeight: 600,
          color: '#e6e9f0'
        }}>
          <FaGripVertical style={{ color: '#9ca3af', fontSize: '0.9rem' }} />
          {panel.icon} {panel.title}
        </span>
        
        <div style={{ display: 'flex', alignItems: 'center' }}>
          <button
            onClick={() => onRemove(panel.key)}
            style={{
              background: 'none', border: 'none', color: '#b0b3c6', fontSize: '1.1em', cursor: 'pointer', padding: '0.2em 0.4em', borderRadius: '50%', transition: 'all 0.15s'
            }}
            onMouseEnter={(e) => {
              const target = e.currentTarget;
              target.style.background = 'rgba(255,255,255,0.08)';
              target.style.color = '#ff5c7c';
            }}
            onMouseLeave={(e) => {
              const target = e.currentTarget;
              target.style.background = 'none';
              target.style.color = '#b0b3c6';
            }}
            title="Remove panel"
          >
            <FaTimes />
          </button>
        </div>
      </div>
      
      {/* Panel Content */}
      <div style={{
        flex: '1 1 auto', display: 'flex', flexDirection: 'column', overflow: 'hidden'
      }}>
        {panel.content}
      </div>

      {/* Resize Handle */}
      <div
        onMouseDown={handleResizeStart}
        style={{
          position: 'absolute', bottom: '5px', right: '5px', width: '20px', height: '20px', cursor: 'se-resize', display: 'flex', alignItems: 'flex-end', justifyContent: 'flex-end', opacity: 0.5
        }}
        title="Drag to resize"
      >
        <svg width="14" height="14" viewBox="0 0 100 100" fill="#e6e9f0">
            <path d="M100 0 L0 100 H20 L100 20 V0 Z M100 60 L60 100 H80 L100 80 V60 Z"/>
        </svg>
      </div>
    </div>
  );
};

// Main App component logic remains largely the same
function ResponsiveDroneDashboard() {
  const [panels, setPanels] = useState([
    ALL_PANELS[0],
    ALL_PANELS[1],
    ALL_PANELS[2], // Default to showing MapView
    ALL_PANELS[4], // RGB Camera
  ]);
  const [panelSizes, setPanelSizes] = useState({});
  const [showAddMenu, setShowAddMenu] = useState(false);
  const [maxColumns, setMaxColumns] = useState(6);
  const [maxRows] = useState(4);
  const [draggedPanelKey, setDraggedPanelKey] = useState(null);
  const [dragOverPanelKey, setDragOverPanelKey] = useState(null);

  useEffect(() => {
    const initialSizes = {};
    panels.forEach(panel => {
      if (!panelSizes[panel.key]) {
        initialSizes[panel.key] = {
          width: panel.defaultWidth,
          height: panel.defaultHeight
        };
      }
    });
    if (Object.keys(initialSizes).length > 0) {
      setPanelSizes(prev => ({ ...prev, ...initialSizes }));
    }
  }, [panels, panelSizes]);

  useEffect(() => {
    const updateMaxColumns = () => {
      const screenWidth = window.innerWidth;
      if (screenWidth < 768) setMaxColumns(2);
      else if (screenWidth < 1024) setMaxColumns(4);
      else if (screenWidth < 1440) setMaxColumns(6);
      else setMaxColumns(8);
    };

    updateMaxColumns();
    window.addEventListener('resize', updateMaxColumns);
    return () => window.removeEventListener('resize', updateMaxColumns);
  }, []);

  const removePanel = (key) => {
    setPanels(panels.filter(p => p.key !== key));
    const newSizes = { ...panelSizes };
    delete newSizes[key];
    setPanelSizes(newSizes);
  };

  const addPanel = (key) => {
    if (panels.find(p => p.key === key)) return;
    
    const panel = ALL_PANELS.find(p => p.key === key);
    setPanels([...panels, panel]);
    setPanelSizes(prev => ({
      ...prev,
      [key]: {
        width: panel.defaultWidth,
        height: panel.defaultHeight
      }
    }));
    setShowAddMenu(false);
  };
  
  const handleDragStart = useCallback((key) => {
    setDraggedPanelKey(key);
  }, []);

  const handleDragOver = useCallback((key) => {
    if (draggedPanelKey && draggedPanelKey !== key) {
        setDragOverPanelKey(key);
    }
  }, [draggedPanelKey]);
  
  const handleDrop = useCallback((draggedKey, targetKey) => {
      if (!draggedKey || !targetKey || draggedKey === targetKey) return;

      const draggedIndex = panels.findIndex(p => p.key === draggedKey);
      const targetIndex = panels.findIndex(p => p.key === targetKey);
      
      if (draggedIndex === -1 || targetIndex === -1) return;
      
      const newPanels = [...panels];
      const [draggedItem] = newPanels.splice(draggedIndex, 1);
      newPanels.splice(targetIndex, 0, draggedItem);
      
      setPanels(newPanels);
      setDraggedPanelKey(null);
      setDragOverPanelKey(null);

  }, [panels]);


  const resizePanel = useCallback((key, newWidth, newHeight) => {
    setPanelSizes(prev => ({
      ...prev,
      [key]: {
        width: newWidth,
        height: newHeight
      }
    }));
  }, []);

  const availablePanels = ALL_PANELS.filter(p => !panels.find(panel => panel.key === p.key));

  return (
    <div style={{
      fontFamily: "'Inter', system-ui, Avenir, Helvetica, Arial, sans-serif",
      background: 'linear-gradient(120deg, #181c2f 0%, #232946 60%, #3a185a 100%)',
      minHeight: '100vh',
      color: '#e6e9f0',
      width: '100vw',
      overflow: 'hidden'
    }}>
      {/* Header */}
      <header style={{
        padding: '1.5rem 1rem 1rem 1rem',
        textAlign: 'center',
        position: 'sticky',
        top: 0,
        zIndex: 100,
        background: 'linear-gradient(120deg, rgb(35, 41, 70) 0%, rgb(58, 24, 90) 100%)',
        boxShadow: '0 2px 20px rgba(0,0,0,0.3)',
        width: '100%'
      }}>
        <h1 style={{
          fontSize: '2rem', fontWeight: 700, margin: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.7em'
        }}>
          <FaRobot /> Autonomous Drone Ground Control
        </h1>
      </header>

      {/* Persistent Status Bar */}
      <StatusBar />

      {/* Main Content Grid */}
      <main style={{
        padding: '1.5rem',
        width: '100vw',
        height: 'calc(100vh - 95px - 60px)', // Adjusted for header and status bar
        overflowY: 'auto',
        boxSizing: 'border-box'
      }}>
        <div style={{
          display: 'grid',
          gridTemplateColumns: `repeat(${maxColumns}, 1fr)`,
          gridAutoRows: 'minmax(150px, auto)',
          gap: '1.5rem',
          width: '100%',
          minHeight: '100%'
        }}>
          {panels.map((panel) => (
            <ResizablePanel
              key={panel.key}
              panel={panel}
              onRemove={removePanel}
              onResize={resizePanel}
              width={panelSizes[panel.key]?.width || panel.defaultWidth}
              height={panelSizes[panel.key]?.height || panel.defaultHeight}
              maxColumns={maxColumns}
              maxRows={maxRows}
              onDragStart={handleDragStart}
              onDragOver={handleDragOver}
              onDrop={handleDrop}
              isBeingDragged={draggedPanelKey === panel.key}
              isDragOver={dragOverPanelKey === panel.key}
            />
          ))}
        </div>
      </main>

      {/* Floating Add Panel Button & Menu */}
      <div style={{
        position: 'fixed', bottom: '2rem', right: '2rem', zIndex: 2000, display: 'flex', flexDirection: 'column', alignItems: 'flex-end'
      }}>
        {showAddMenu && (
          <div style={{
            background: 'rgba(30, 34, 54, 0.98)', borderRadius: '14px', boxShadow: '0 8px 32px rgba(0,0,0,0.3)', border: '1.5px solid rgba(255,255,255,0.1)', padding: '1.1rem 1.2rem', minWidth: '210px', marginBottom: '0.5rem', animation: 'fadeInMenu 0.18s ease-out'
          }}>
            <div style={{
              fontSize: '1.1rem', fontWeight: 600, color: '#e6e9f0', marginBottom: '0.7em'
            }}>
              Add Panel
            </div>
            
            {availablePanels.length === 0 ? (
              <div style={{
                color: '#b0b3c6', fontSize: '0.98rem', padding: '0.5em 0', textAlign: 'center'
              }}>
                All panels added
              </div>
            ) : (
              availablePanels.map(panel => (
                <button
                  key={panel.key}
                  onClick={() => addPanel(panel.key)}
                  style={{
                    background: 'none', border: 'none', color: '#e6e9f0', fontSize: '1rem', fontWeight: 500, padding: '0.6em 0.5em', borderRadius: '8px', marginBottom: '0.3em', textAlign: 'left', display: 'flex', alignItems: 'center', gap: '0.6em', cursor: 'pointer', transition: 'all 0.13s', width: '100%'
                  }}
                  onMouseEnter={(e) => {
                    const target = e.currentTarget;
                    target.style.background = 'rgba(138, 43, 226, 0.2)';
                    target.style.color = '#a855f7';
                  }}
                  onMouseLeave={(e) => {
                    const target = e.currentTarget;
                    target.style.background = 'none';
                    target.style.color = '#e6e9f0';
                  }}
                >
                  {panel.icon} {panel.title}
                </button>
              ))
            )}
          </div>
        )}

        <button
          onClick={() => setShowAddMenu(!showAddMenu)}
          style={{
            background: 'linear-gradient(135deg, #7c3aed 0%, #5b21b6 100%)', color: '#ffffff', border: '2px solid rgba(255,255,255,0.2)', borderRadius: '50%', width: '60px', height: '60px', fontSize: '1.8rem', boxShadow: '0 6px 20px rgba(124, 58, 237, 0.4), 0 2px 8px rgba(0,0,0,0.2)', display: 'flex', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', transition: 'all 0.2s ease', transform: showAddMenu ? 'rotate(45deg)' : 'rotate(0deg)'
          }}
          title="Add/Close Menu"
        >
          <FaPlus />
        </button>
      </div>

      {/* Instructions overlay */}
      <div className="instructions" style={{
        position: 'fixed', bottom: '20px', left: '20px', background: 'rgba(30, 34, 54, 0.9)', borderRadius: '12px', padding: '1rem', fontSize: '0.85rem', color: '#b0b3c6', maxWidth: '320px', zIndex: 1000, border: '1px solid rgba(255,255,255,0.1)'
      }}>
        <strong style={{ color: '#e6e9f0' }}>Controls:</strong><br/>
        • Drag a panel's header to reorder it.<br/>
        • Drag the handle in the bottom-right corner to resize.<br/>
        • Use the + button to add or remove panels.
      </div>

      {/* CSS Styles */}
      <style>{`
        * { box-sizing: border-box; }
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; }
        @keyframes fadeInMenu { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }
        @media (max-width: 768px) { .instructions { display: none; } }
      `}</style>
    </div>
  );
}

export default ResponsiveDroneDashboard;