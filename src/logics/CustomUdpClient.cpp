// CustomUdpClient.cc
#include "CustomUdpClient.h"
#include <random>

CustomUdpClient::CustomUdpClient(Ipv4Address serverAddress, uint16_t port, uint32_t maxPacketCount, uint32_t packetSize, Time interPacketInterval, uint32_t X, uint32_t Y, std::vector<std::vector<int>> matrix)
    : m_serverAddress(serverAddress), m_port(port), m_maxPacketCount(maxPacketCount), m_packetSize(packetSize), m_interPacketInterval(interPacketInterval),
      m_packetsSent(0), m_X(X), m_Y(Y), m_matrix(matrix)
{
}

CustomUdpClient::~CustomUdpClient()
{
  m_socket = 0;
}

void CustomUdpClient::StartApplication(void)
{
  m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_socket->Bind(); // bind to any available port

  InetSocketAddress serverSocketAddress(m_serverAddress, m_port);
  m_socket->Connect(serverSocketAddress);

  m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
  m_socket->SetAcceptCallback(MakeNullCallback<bool, Ptr<Socket>, const Address &>(), MakeNullCallback<void, Ptr<Socket>, const Address &>());

  // Schedule sending of the first packet
  m_sendEvent = Simulator::Schedule(Seconds(2.0), &CustomUdpClient::SendPacket, this);
}

void CustomUdpClient::StopApplication(void)
{
  std::cout << "[CLIENT] -> Movements done: " << movements << std::endl;
  printMatrix(m_matrix);
  if (m_sendEvent.IsRunning())
  {
    Simulator::Cancel(m_sendEvent);
  }

  if (m_socket)
  {
    m_socket->Close();
  }
}

void CustomUdpClient::SendPacket(void)
{

  CustomLogic();

  // Set the payload string
  std::ostringstream msgx;
  msgx << "Coord! X: " << m_X  << " Y: "<< m_Y << '\0';

  uint16_t packetSize = msgx.str().length() + 1;
  Ptr<Packet> packet = Create<Packet>((uint8_t *)msgx.str().c_str(), packetSize);

  // Send the packet
  m_socket->Send(packet);

  ++m_packetsSent;

  if (m_packetsSent < m_maxPacketCount)
  {
    // Schedule the next packet transmission
    m_sendEvent = Simulator::Schedule(m_interPacketInterval, &CustomUdpClient::SendPacket, this);
  }
}

void CustomUdpClient::CustomLogic(void){
  // Seed the random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range [0, 3]
  std::uniform_int_distribution<int> distribution(0, 3);

  // Generate a random number between 0 and 3
  int random_number = distribution(gen);

  if (random_number == 0){
    m_X--;
  } else if (random_number == 1){
    m_Y++;
  } else if (random_number == 2){
    m_X++;
  } else {
    m_Y--;
  }
  movements++;
}