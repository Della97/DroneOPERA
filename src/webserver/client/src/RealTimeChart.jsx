import React, { useEffect, useState } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip } from 'recharts';

const MAX_POINTS = 20; // Keep last 20 points for each drone

const RealTimeChart = () => {
  const [dataByDrone, setDataByDrone] = useState({
    0: [],
    1: [],
    2: [],
    3: []
  });

  useEffect(() => {
    const socket = new WebSocket('ws://localhost:8080');

    socket.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        const { id, time, value } = msg;

        setDataByDrone(prev => {
          const updated = [...(prev[id] || []), { time, value }];
          if (updated.length > MAX_POINTS) updated.shift();

          return { ...prev, [id]: updated };
        });
      } catch (e) {
        console.error("Invalid message:", event.data);
      }
    };

    return () => socket.close();
  }, []);

  return (
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '2rem', padding: '2rem' }}>
      {[0, 1, 2, 3].map(id => (
        <div key={id} style={{ border: '1px solid #ddd', borderRadius: '12px', padding: '1rem', boxShadow: '0 2px 8px rgba(0,0,0,0.1)' }}>
          <h2 style={{ textAlign: 'center' }}>Drone #{id}</h2>
          <LineChart width={400} height={250} data={dataByDrone[id]}>
            <CartesianGrid stroke="#ccc" strokeDasharray="5 5" />
            <XAxis dataKey="time" />
            <YAxis domain={[0, 100]} />
            <Tooltip />
            <Line type="monotone" dataKey="value" stroke="#8884d8" strokeWidth={2} dot={false} />
          </LineChart>
        </div>
      ))}
    </div>
  );
};

export default RealTimeChart;
