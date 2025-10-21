# Real-Time C++ Simulation Visualizer

## Run Instructions

### 1. Start WebSocket Server
```bash
cd server
node server.js
```

### 2. Start Frontend
```bash
cd client
npm install
npm run dev
```

### 3. Run C++ Simulation
```bash
cd cpp-sender
mkdir build && cd build
cmake ..
make
./simulation_sender
```

Charts update in real-time at: http://localhost:5173