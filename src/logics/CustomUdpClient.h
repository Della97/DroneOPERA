// CustomUdpClient.h
#ifndef CUSTOM_UDP_CLIENT_H
#define CUSTOM_UDP_CLIENT_H

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"

//UTILS
#include "../utils/MatrixFunctions.h"

using namespace ns3;

/**
 * @brief Custom UDP client application for NS-3 simulation.
 */
class CustomUdpClient : public Application
{
public:
  CustomUdpClient(Ipv4Address serverAddress, uint16_t port, uint32_t maxPacketCount, uint32_t packetSize, Time interPacketInterval, uint32_t X, uint32_t Y, std::vector<std::vector<int>> matrix);
  virtual ~CustomUdpClient();

private:
  virtual void StartApplication(void);
  virtual void StopApplication(void);
  void SendPacket(void);
  void CustomLogic(void);

  Ipv4Address m_serverAddress;
  uint16_t m_port;
  uint32_t m_maxPacketCount;
  uint32_t m_packetSize;
  Time m_interPacketInterval;
  Ptr<Socket> m_socket;
  EventId m_sendEvent;
  uint32_t m_packetsSent;
  //Drone variables
  uint32_t m_Y;
  uint32_t m_X;
  uint32_t m_battery = 100;
  uint32_t movements = 0;
  std::vector<std::vector<int>> m_matrix;
};

#endif // CUSTOM_UDP_CLIENT_H
