// CustomUdpServer.cc
#include "CustomUdpServer.h"

CustomUdpServer::CustomUdpServer(uint16_t port) : m_port(port)
{
}

CustomUdpServer::~CustomUdpServer()
{
  m_socket = 0;
}

void CustomUdpServer::StartApplication(void)
{
  m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
  m_socket->Bind(local);
  m_socket->SetRecvCallback(MakeCallback(&CustomUdpServer::HandleRead, this));
}

void CustomUdpServer::StopApplication(void)
{
  std::cout << "[SERVER] -> Received data count: " << packetReceived << std::endl;
  if (m_socket)
  {
    m_socket->Close();
  }
}

void CustomUdpServer::HandleRead(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom(from)))
  {
    // Process the received packet as needed
    uint8_t buffer[1024];
    packet->CopyData(buffer, packet->GetSize());
    std::string receivedData(reinterpret_cast<const char *>(buffer), packet->GetSize());

    // Print received data to the console
    NS_LOG_INFO("Received data: " << receivedData);
    std::cout << "[SERVER] -> Received data: " << receivedData << std::endl;
    packetReceived++;
    // Perform custom action with the received data
    //CustomAction(receivedData);
  }
}

void CustomUdpServer::ServerLogic(std::string &data)
{
  // Example: Convert received data to uppercase and print
  std::transform(data.begin(), data.end(), data.begin(), ::toupper);
  std::cout << "Modified data: " << data << std::endl;
  // Perform other actions as needed
}