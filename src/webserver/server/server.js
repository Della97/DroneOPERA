// server.js
const net = require('net');
const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 8080 });
const tcpServer = net.createServer();

tcpServer.on('connection', (socket) => {
  console.log('C++ sender connected');
  socket.on('data', (data) => {
    const str = data.toString();
    // Forward line-by-line JSON messages to all WebSocket clients
    str.split('\n').forEach(msg => {
      if (msg.trim()) {
        wss.clients.forEach(ws => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(msg.trim());
          }
        });
      }
    });
  });
});

tcpServer.listen(7070, () => console.log('TCP server listening on port 7070'));
