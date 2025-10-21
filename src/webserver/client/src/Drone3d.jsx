import React, { useEffect, useState } from 'react';
import Plot from 'react-plotly.js';

const Drone3D = () => {
  const [drones, setDrones] = useState({});

  useEffect(() => {
    const socket = new WebSocket('ws://localhost:8080');
    socket.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        const { id, x, y, z } = msg;

        if (x != null && y != null && z != null) {
          setDrones(prev => ({
            ...prev,
            [id]: { x, y, z }
          }));
        }
      } catch (e) {
        console.error('Invalid data received:', event.data);
      }
    };

    return () => socket.close();
  }, []);

  const ids = Object.keys(drones);
  const xs = ids.map(id => drones[id].x);
  const ys = ids.map(id => drones[id].y);
  const zs = ids.map(id => drones[id].z);
  const labels = ids.map(id => `Drone ${id} (z: ${drones[id].z.toFixed(1)})`);

  return (
    <div style={{ marginTop: '2rem' }}>
      <div style={{ marginTop: '2rem' }}>
        <h2 style={{ textAlign: 'center' }}>🛰️ 3D Drone Position Viewer</h2>
        <Plot
          data={[
            {
              x: xs,
              y: ys,
              z: zs,
              text: labels,
              type: 'scatter3d',
              mode: 'markers+text',
              marker: {
                size: 8,
                color: zs,
                colorscale: 'Viridis',
                colorbar: { title: 'Altitude (z)' }
              },
              textposition: 'top center'
            }
          ]}
          layout={{
            width: 700,
            height: 600,
            margin: { l: 0, r: 0, b: 0, t: 0 },
            scene: {
              xaxis: { title: 'X', fixedrange: true },
              yaxis: { title: 'Y', fixedrange: true },
              zaxis: { title: 'Z (Altitude)', fixedrange: true },
              camera: {
                eye: { x: 500, y: 500, z: 500 } // Fixed camera position
              }
            }
          }}
          config={{
            scrollZoom: false,
            staticPlot: true // Lock interactions
          }}
        />
      </div>
    </div>
  );
};

export default Drone3D;
