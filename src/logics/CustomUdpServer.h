// CustomUdpServer.h
#ifndef CUSTOM_UDP_SERVER_H
#define CUSTOM_UDP_SERVER_H

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

/**
 * @brief Custom UDP server application for NS-3 simulation.
 */
class CustomUdpServer : public Application
{
public:
  CustomUdpServer(uint16_t port);
  virtual ~CustomUdpServer();

private:
  virtual void StartApplication(void);
  virtual void StopApplication(void);
  void HandleRead(Ptr<Socket> socket);
  void ServerLogic(std::string &data);

  uint16_t m_port;
  Ptr<Socket> m_socket;
  uint32_t packetReceived = 0;
};

#endif // CUSTOM_UDP_SERVER_H
