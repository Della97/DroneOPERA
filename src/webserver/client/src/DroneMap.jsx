import React, { useEffect, useState } from 'react';
import Plot from 'react-plotly.js';

const DroneMap = () => {
  const [dronePaths, setDronePaths] = useState({});

  useEffect(() => {
    const socket = new WebSocket('ws://localhost:8080');

    socket.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        const { id, x, y } = msg;

        if (x != null && y != null) {
          setDronePaths(prev => {
            const updated = { ...prev };
            if (!updated[id]) updated[id] = [];
            updated[id] = [...updated[id], { x, y }].slice(-30); // Keep last 30
            return updated;
          });
        }
      } catch (e) {
        console.error('Invalid 2D data:', event.data);
      }
    };

    return () => socket.close();
  }, []);

  const traces = Object.entries(dronePaths).flatMap(([id, path]) => {
    const xs = path.map(p => p.x);
    const ys = path.map(p => p.y);
    const n = path.length;

    const fadeTrace = {
      x: xs,
      y: ys,
      mode: 'markers',
      name: `Drone ${id} Trail`,
      marker: {
        size: 6,
        color: path.map((_, i) => `rgba(0, 100, 255, ${0.2 + (i / n) * 0.8})`), // Fading opacity
      },
      showlegend: false
    };

    const current = path[n - 1];

    const markerTrace = {
      x: [current.x],
      y: [current.y],
      mode: 'markers+text',
      name: `Drone ${id}`,
      marker: { size: 12, color: 'red' },
      text: [`Drone ${id}`],
      textposition: 'top center'
    };

    return [fadeTrace, markerTrace];
  });

  return (
    <div style={{ marginTop: '2rem' }}>
      <h2 style={{ textAlign: 'center' }}>📍 2D Drone Map with Fading Trails</h2>
      <div style={{ display: 'flex', justifyContent: 'center' }}>
        <Plot
          data={traces}
          layout={{
            width: 700,
            height: 500,
            xaxis: { title: 'X', range: [0, 500], fixedrange: true },
            yaxis: { title: 'Y', range: [0, 500], fixedrange: true },
            margin: { l: 40, r: 40, t: 40, b: 40 },
            showlegend: true
          }}
          config={{
            scrollZoom: false,
            staticPlot: false
          }}
        />
      </div>
    </div>
  );
};

export default DroneMap;
