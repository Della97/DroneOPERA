import React from 'react';
import RealTimeChart from './RealTimeChart';
import DroneMap from './DroneMap';
import Drone3D from './Drone3D';

function App() {
  return (
    <div>
      <h1 style={{ textAlign: 'center', padding: '1rem' }}>🚀 Drone Dashboard</h1>
      <RealTimeChart />
      <DroneMap />
      <Drone3D />
    </div>
  );
}

export default App;
