//NS3
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/boolean.h"
#include "ns3/netanim-module.h"
#include "ns3/netsimulyzer-module.h"

//STD
#include <cstdlib>
#include <ctime>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiSimpleInfra");

Ptr<netsimulyzer::LogStream> eventLog;


/**
 * Function called when a packet is received.
 *
 * \param socket The receiving socket.
 */
void ReceivePacket(Ptr<Socket> socket) {
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom(from))) {
    // Process the received packet as needed
    uint8_t buffer[1024];
    packet->CopyData(buffer, packet->GetSize());
    std::string receivedData(reinterpret_cast<const char *>(buffer), packet->GetSize());

    // Print received data to the console
    NS_LOG_INFO("Received data: " << receivedData);
    std::cout << "Received data: " << receivedData << std::endl;
    // Perform custom action with the received data
    //CustomAction(receivedData);
  }
}

/**
 * Generate traffic. This function sends the coordinate of the node to the server.
 *
 * \param socket The sending socket.
 * \param pktSize The packet size.
 * \param pktCount The packet count.
 * \param pktInterval The interval between two packets.
 * \param node The done instance.
 */
static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval, Ptr<Node> node) {
    //MOBILITY FIRST
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();

    //MESSAGE TO SERVER
    if (pktCount > 0) {
        // Set the payload string
        std::ostringstream msgx;
        msgx << "[HOST " << node->GetId() << "] -> Position X: " << pos.x << " Y: " << pos.y << '\0';
        uint16_t packetSize = msgx.str().length() + 1;
        Ptr<Packet> packet = Create<Packet>((uint8_t *)msgx.str().c_str(), packetSize);
        socket->Send(packet);

        Simulator::Schedule(pktInterval,
                            &GenerateTraffic,
                            socket,
                            pktSize,
                            pktCount - 1,
                            pktInterval,
                            node);
    }
    else {
        socket->Close();
    }
}

int main(int argc, char* argv[]) {
    std::string phyMode("DsssRate1Mbps");
    double rss = -80;           // -dBm
    uint32_t packetSize = 1000; // bytes
    uint32_t numPackets = 1000;
    Time interval = Seconds(10.0);
    bool verbose = false;
    bool spawnBuildings = false;

    WifiHelper wifi;
    MobilityHelper mobility;
    MobilityHelper mobilityAP;
    NodeContainer stas;
    NodeContainer ap;
    NetDeviceContainer staDevs;
    PacketSocketHelper packetSocket;

    uint32_t numbHosts = 4;
    stas.Create(numbHosts);

    ap.Create(1);

    // NETSIMULYZER THINGS***********************************************************
    double minNodePosition = 0;
    double maxNodePosition = 200;
    // These must remain positive (since the RandomDirection2dMobilityModel only accepts positive
    // values)
    double minSpeed = .1;
    double maxSpeed = 5;
    double duration = 100;
    std::string outputFileName = "netsimulyzer-mobility-buildings-example.json";
    std::string phoneModelPath = netsimulyzer::models::SERVER;
    std::string droneModelPath = netsimulyzer::models::QUADCOPTER_UAV;
    std::string dname = "Drone";
    std::string sname = "Server";

    // ---- NetSimulyzer ----
    Ptr<ns3::netsimulyzer::Orchestrator> orchestrator = CreateObject<netsimulyzer::Orchestrator>(outputFileName);

    // ---- Buildings ----
    BuildingContainer buildings;
    if (spawnBuildings) {
        Ptr<Building> twoFloorBuilding = CreateObject<Building>();
        twoFloorBuilding->SetBoundaries({150.0, 130.0, 150.0, 130.0, 0.0, 100.0});
        twoFloorBuilding->SetNFloors(2);
        buildings.Add(twoFloorBuilding);
        netsimulyzer::BuildingConfigurationHelper buildingConfigHelper(orchestrator);
        buildingConfigHelper.Install(buildings);
    }

    // Mark possible Node locations
    auto possibleNodeLocations = CreateObject<netsimulyzer::RectangularArea>(
        orchestrator,
        Rectangle{minNodePosition, maxNodePosition, minNodePosition, maxNodePosition});

    // Identify the area
    possibleNodeLocations->SetAttribute("Name", StringValue("Possible Node Locations"));

    // Mark with a light green color
    possibleNodeLocations->SetAttribute("FillColor", netsimulyzer::Color3Value{204u, 255u, 204u});

    auto infoLog = CreateObject<netsimulyzer::LogStream>(orchestrator);

    // Log the base configuration for the scenario
    *infoLog << "----- Scenario Settings -----\n";
    *infoLog << "Node Position Range: [" << minNodePosition << ',' << maxNodePosition << "]\n";
    *infoLog << "Node Speed Range: [" << minSpeed << ',' << maxSpeed << "]\n";
    *infoLog << "Models: Phone [" << phoneModelPath << "], Drone [" << droneModelPath << "]\n";
    *infoLog << "Scenario Duration (Seconds): " << duration << '\n';

    netsimulyzer::NodeConfigurationHelper nodeConfigHelper(orchestrator);
    nodeConfigHelper.Set("EnableMotionTrail", BooleanValue(true));
    nodeConfigHelper.Set("Model", StringValue(phoneModelPath));
    nodeConfigHelper.Set("Scale", DoubleValue(3));
    nodeConfigHelper.Set("Name", StringValue(sname));
    nodeConfigHelper.Install(ap);

    nodeConfigHelper.Set("Model", StringValue(droneModelPath));
    nodeConfigHelper.Set("Scale", DoubleValue(10));
    // Set the names inside netsimulyzer
    for (int i = 0; i < numbHosts; i++) {
        std::string tmp = dname + std::to_string(i);
        nodeConfigHelper.Set("Name", StringValue(tmp));
        nodeConfigHelper.Install(stas.Get(i));
    }

    //****************************************************************************

    //GENERAL SETUP

    // The below set of helpers will help us to put together the wifi NICs we want
    if (verbose)
    {
        WifiHelper::EnableLogComponents(); // Turn on all Wifi logging
    }
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiPhyHelper wifiPhy;
    // This is one parameter that matters when using FixedRssLossModel
    // set it to zero; otherwise, gain will be added
    wifiPhy.Set("RxGain", DoubleValue(0));
    // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // The below FixedRssLossModel will cause the rss to be fixed regardless
    // of the distance between the two stations, and the transmit power
    wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(rss));
    wifiPhy.SetChannel(wifiChannel.Create());

    // Add a mac and disable rate control
    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue(phyMode),
                                 "ControlMode",
                                 StringValue(phyMode));

    // Setup the rest of the MAC
    Ssid ssid = Ssid("wifi-default");

    //SETUP
    // setup AP
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice = wifi.Install(wifiPhy, wifiMac, ap.Get(0));
    NetDeviceContainer devices= apDevice;
    devices.Add(apDevice);

    // Setup STA
    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));


    for (int i = 0; i < numbHosts; i++) {
        NetDeviceContainer tmp = wifi.Install(wifiPhy, wifiMac, stas.Get(i));
        devices.Add(tmp);
    }

    //MOBILITY STAS
    // Note that with FixedRssLossModel, the positions below are not
    // used for received signal strength.
    std::array<std::string, 4> boundArray = {"0|100|0|100", "100|200|0|100", "0|100|100|200", "100|200|100|200"};
    std::array<std::string, 4> startArrayX = {"50", "150", "50", "150"};
    std::array<std::string, 4> startArrayY = {"50", "50", "150", "150"};
    for (int i = 0; i < numbHosts; i++) {
        mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                      "X",
                                      StringValue(startArrayX[i]),
                                      "Y",
                                      StringValue(startArrayY[i]),
                                      "Rho",
                                      StringValue("ns3::UniformRandomVariable[Min=0|Max=0]"));
        mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                  "Mode",
                                  StringValue("Time"),
                                  "Time",
                                  StringValue("5s"),
                                  "Speed",
                                  StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
                                  "Bounds",
                                  StringValue(boundArray[i]));
        mobility.Install(stas.Get(i));
    }

    //MOBILITY AP (STATIONARY AP)
    Ptr<ListPositionAllocator> positionAllocAP = CreateObject<ListPositionAllocator>();
    positionAllocAP->Add(Vector(100.0, 100.0, 0.0));
    mobilityAP.SetPositionAllocator(positionAllocAP);
    mobilityAP.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAP.Install(ap);

    AnimationInterface anim ("SimpleNS3Simulation_NetAnimationOutput.xml");

    for (uint32_t i = 0; i < numbHosts; ++i) {
        anim.SetConstantPosition (stas.Get(i), 0, 0);
    }

    //IP
    InternetStackHelper internet;
    internet.Install(stas);
    internet.Install(ap);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    //SOCKETS
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(ap.Get(0), tid);
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), 80);

    // Create an array of Ptr<Socket>
    std::vector<Ptr<Socket>> socketArray;

    // Create sockets for each node
    for (uint32_t i = 0; i < numbHosts; ++i) {
        Ptr<Socket> socket = Socket::CreateSocket(stas.Get(i), tid);
        socket->SetAllowBroadcast(true);
        socket->Connect(remote);
        socketArray.push_back(socket);
    }

    // Tracing
    wifiPhy.EnablePcap("wifi-simple-infra", devices);


    //SIMULATION

    // Output what the simulation will do
    std::cout << "Testing " << numPackets << " packets sent with receiver rss " << rss << " Number of Hosts: " << numbHosts << std::endl;

    for (uint32_t i = 0; i < numbHosts; ++i) {
        
        Simulator::ScheduleWithContext(socketArray[i]->GetNode()->GetId(),
                                   Seconds(1.0),
                                   &GenerateTraffic,
                                   socketArray[i],
                                   packetSize,
                                   numPackets,
                                   interval,
                                   stas.Get(i));
                                   
    }

    Simulator::Stop(Seconds(1000.0));
    Simulator::Run();

    *infoLog << "Scenario Finished\n";

    Simulator::Destroy();

    return 0;
}