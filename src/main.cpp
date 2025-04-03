/*
*                 TOPOLOGY STRUCTURE
*                 -------   -------
*                  RANK 0    RANK 1
*                 ------- | -------
*                         |
* HOST 0 -------\         |         /------- HOST 2
*                ---(ApL)-|-(ApR)---
* HOST 1 -------/         |         \------- HOST 3
*
*
* - The connection between ApL and ApR (Accesses point for Left and Right nodes)
*   is done by cable. (Still needs to actually do the link for now no need)
* - The connection between the hosts and the respective Ap is done by WiFi.
*/



//NS3
#include "ns3/core-module.h"
#include "ns3/network-module.h"
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
#include "ns3/gnuplot.h"
#include "ns3/mpi-module.h"
#include "ns3/mpi-interface.h"
#include "ns3/mpi-receiver.h"
#include "ns3/box.h"
#include <ns3/energy-module.h>
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/wifi-radio-energy-model.h"
#include <ns3/core-module.h>

#include "drone/Drone.h"
#include "includes/rapidjson/include/rapidjson/document.h"
#include "includes/rapidjson/include/rapidjson/filereadstream.h"
#include "mobility/custom-mobility-model.h"
#include "parser/JsonParser.h"

//MPI
#ifdef NS3_MPI
#include <mpi.h>
#endif

//STD
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <filesystem> // Include for filesystem operations (C++17)

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiSimpleInfra");

Ptr<netsimulyzer::LogStream> eventLog;

std::vector<std::string> v;

void RxEndCallback (Ptr<const Packet> packet, double rssi)
{
    NS_LOG_UNCOND ("Received packet with RSSI: " << rssi << " dBm");
    std::cout << "Received packet with RSSI: " << rssi << " dBm" << std::endl;
}


/**
 * Function called when a packet is received.
 *
 * \param socket The receiving socket.
 */
void EdgeLogic(Ptr<Socket> socket) {
  Ptr<Packet> packet;
  Address from;

  // Open the file in append mode
  //std::ofstream outFile("results/results.txt", std::ios::app);
  //if (!outFile) {
  //  NS_LOG_ERROR("Failed to open file for writing.");
  //  return;
  //}

  while ((packet = socket->RecvFrom(from))) {
    // Process the received packet as needed
    uint8_t buffer[1024];
    packet->CopyData(buffer, packet->GetSize());
    std::string receivedData(reinterpret_cast<const char *>(buffer), packet->GetSize());

    // Print received data to the console
    Time now = Simulator::Now();
    // Write the received data and timestamp to the file
    v.push_back(receivedData);
    std::cout << receivedData << std::endl;
  }
}  //ReceivePacket()

/**
 * Drone logic. This function sends the coordinate of the node to the server.
 *
 * \param socket The sending socket.
 * \param pktSize The packet size.
 * \param pktCount The packet count.
 * \param pktInterval The interval between two packets.
 * \param node The done instance.
 */
static void DroneLogic(Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval, Drone drone, double volt) {
    //MOBILITY FIRST AND GATHER DATA
    // Get the Ptr to the MobilityModel from the Drone
    Ptr<CustomMobilityModel> mobilityModel = drone.getNode()->GetObject<CustomMobilityModel>();
    Ptr<SimpleDeviceEnergyModel> battery = drone.getEnergyModel();

    Vector pos = mobilityModel->GetPosition();
    double ampere = 0;
    double mobilityA = 0;
    double computingA = 0;
    double hwA = 0;

    //TRAIN
    if (mobilityModel->getState() == 0) {
        ampere = 0;
        ampere = drone.calcMovePower(mobilityModel->getState())/volt;
        mobilityA = drone.calcMovePower(mobilityModel->getState())/volt;
        battery->SetCurrentA(ampere); // Set the actual draw of energy
    }
    if (mobilityModel->getState() == 1) {
        ampere = 0;
        const std::vector<std::vector<double>>& hardware = drone.getHardware();
        if (mobilityModel->getCompState()) { //COMP STARTED BUT NO IN AOI
            //all
            double sum = 0.0;

            // Iterate over each sub-array in hardware
            for (const auto& hwElement : hardware) {
                if (hwElement.size() > 2) {
                    sum += hwElement[1]/hwElement[3];
                }
            }
            ampere = drone.calculateComputePower()/1.3 + drone.calcMovePower(mobilityModel->getState())/volt + sum;
            computingA = drone.calculateComputePower()/1.3;
            mobilityA = drone.calcMovePower(mobilityModel->getState())/volt;
            hwA = sum;
            battery->SetCurrentA(ampere); // Set the actual draw of energy
        } else {  //NOT IN AOI NO COMP
            double sum = 0.0;

            // Iterate over each sub-array in hardware
            for (const auto& hwElement : hardware) {
                if (hwElement.size() > 2) {
                    sum += hwElement[1]/hwElement[3];
                }
            }
            //only hover + drag
            ampere = drone.calcMovePower(mobilityModel->getState())/volt;
            mobilityA = drone.calcMovePower(mobilityModel->getState())/volt;
            battery->SetCurrentA(ampere); // Set the actual draw of energy
        }
        //std::cout << "Hover Power + drag + computation: " << drone.calculateComputePower() << std::endl;
    }
    if (mobilityModel->getState() == 2) {  // IN AOI AND COMPUTE
        ampere = 0;
        const std::vector<std::vector<double>>& hardware = drone.getHardware();
        double sum = 0.0;

        // Iterate over each sub-array in hardware
        for (const auto& hwElement : hardware) {
            if (hwElement.size() > 2) {
                sum += hwElement[2]/hwElement[3]; //HARDWARE ON
            }
        }
        ampere = drone.calculateComputePower()/1.3 + drone.calcMovePower(mobilityModel->getState())/volt + sum;
        computingA = drone.calculateComputePower()/1.3;
        mobilityA = drone.calcMovePower(mobilityModel->getState())/volt;
        hwA = sum;
        battery->SetCurrentA(ampere); // Set the actual draw of energy
    }
    if (mobilityModel->getState() == 3) {
        ampere = 0;
        ampere = drone.calcMovePower(mobilityModel->getState())/volt;
        mobilityA = drone.calcMovePower(mobilityModel->getState())/volt;
        battery->SetCurrentA(ampere); // Set the actual draw of energy
    }

    double percentage = (battery->GetTotalEnergyConsumption() / drone.getMaxCapacity())*100;

    if (drone.getNode()->GetId() != 0){
        //std::cout << percentage << std::endl;
    }

    //MESSAGE TO SERVER
    if (percentage < 100) {
        // Set the payload string
        std::ostringstream msgx;
        Time now = Simulator::Now();
        msgx << drone.getNode()->GetId() << " " << pos.x << " " << pos.y << " " << pos.z << " " << battery->GetTotalEnergyConsumption()<< " " << now << " " << mobilityModel->getAoI() << " " << ampere << " " << 100-percentage << " " << mobilityA << " " << hwA << " " << computingA << " " << mobilityModel->getState() << '\0';
        uint16_t packetSize = msgx.str().length() + 1;
        Ptr<Packet> packet = Create<Packet>((uint8_t *)msgx.str().c_str(), packetSize);
        socket->Send(packet);
        if (drone.getNode()->GetId() == 3) {
            //std::cout << "mobility state: " << mobilityModel->getState() << " ID: " << drone.getNode()->GetId() << " Z -> " << pos.z << std::endl;
        }

        Simulator::Schedule(pktInterval,
                            &DroneLogic,
                            socket,
                            pktSize,
                            pktCount - 1,
                            pktInterval,
                            drone,
                            volt);
    }
    else {
        socket->Close();
    }
}  //DroneLogic()


int main(int argc, char* argv[]) {
    // Check if the program received a file path as an argument
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config file path>" << std::endl;
        return 1; // Return error code if no argument is provided
    }

    // Store the file path in a std::string
    std::string configPath = argv[1];

    // Output the stored file path to verify
    std::cout << "The file path provided is: " << configPath << std::endl;

    
    //////////////////////////////////////
    //          MPI INIT                //
    //////////////////////////////////////
    MpiInterface::Enable (&argc, &argv);
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::DistributedSimulatorImpl"));
    uint32_t systemId = MpiInterface::GetSystemId ();
    uint32_t systemCount = MpiInterface::GetSize ();
    /*
    char name[MPI_MAX_PROCESSOR_NAME];
    int length;
    MPI_Get_processor_name(name, &length);
    */
    std::string phyMode("DsssRate1Mbps");
    double rss = -80;           // -dBm
    uint32_t packetSize = 1000; // bytes
    uint32_t numPackets = 10000;
    Time interval = Seconds(1.0);
    bool verbose = false;
    bool spawnBuildings = false;

    WifiHelper wifi;
    MobilityHelper mobility;
    MobilityHelper mobilityAP;
    NodeContainer stas;  //stas on the left node  (LP 0)
    NodeContainer ap;    //right access point (LP 1)
    NetDeviceContainer staDevs;
    PacketSocketHelper packetSocket;

    uint32_t numbHosts = 2;
    stas.Create(4, 0);  //Clients on LP rank 0 (CUSTOM mobility)

    ap.Create(1, 0);            //Server on LP rank 0

    // NETSIMULYZER THINGS***********************************************************
    double minNodePosition = 0;
    double maxNodePosition = 500;
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

    // Mark possible Node locations
    auto possibleNodeLocations = CreateObject<netsimulyzer::RectangularArea>(
        orchestrator,
        Rectangle{minNodePosition, maxNodePosition, minNodePosition, maxNodePosition});

    // Identify the area
    possibleNodeLocations->SetAttribute("Name", StringValue("Possible Node Locations"));

    // Mark with a light green color
    possibleNodeLocations->SetAttribute("FillColor", netsimulyzer::Color3Value{204u, 255u, 204u});

    auto infoLog = CreateObject<netsimulyzer::LogStream>(orchestrator);

    netsimulyzer::NodeConfigurationHelper nodeConfigHelper(orchestrator);
    nodeConfigHelper.Set("EnableMotionTrail", BooleanValue(true));
    nodeConfigHelper.Set("Model", StringValue(phoneModelPath));
    nodeConfigHelper.Set("Scale", DoubleValue(3));
    nodeConfigHelper.Set("Name", StringValue(sname));
    nodeConfigHelper.Install(ap);

    nodeConfigHelper.Set("Model", StringValue(droneModelPath));
    nodeConfigHelper.Set("Scale", DoubleValue(10));
    // Set the names inside netsimulyzer
    std::string tmp;
    for (int i = 0; i < 4; i++) {
        tmp = dname + std::to_string(i);
        nodeConfigHelper.Set("Name", StringValue(tmp));
        nodeConfigHelper.Install(stas.Get(i));
    }
    //GENERAL SETUP

    // The below set of helpers will help us to put together the wifi NICs we want
    if (verbose)
    {
        WifiHelper::EnableLogComponents(); // Turn on all Wifi logging
    }
    wifi.SetStandard(WIFI_STANDARD_80211b);

    // Create wifiPhyHelper
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper();
    wifiPhy.Set("RxGain", DoubleValue(0));
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    // Create wifiChannelHelper
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // Use LogDistancePropagationLossModel instead of FixedRssLossModel
    wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(rss));

    // Attach the channel to the phy
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


    for (int i = 0; i < 4; i++) {
        NetDeviceContainer tmp = wifi.Install(wifiPhy, wifiMac, stas.Get(i));
        devices.Add(tmp);
    }

    // Connect the callback to the PhyRxEnd trace source
    // Connect the callback to the PhyRxEnd trace source for each device
    

    //****************************************************************************
    //BATTERY SETUP

    /********************************BATTERY MODEL**********************************************/

    Ptr<GenericBatteryModel> batteryModel1 = CreateObject<GenericBatteryModel>();

    batteryModel1->SetAttribute("FullVoltage", DoubleValue(12.6)); // Vfull (4.2V per cell, 3S)
    batteryModel1->SetAttribute("MaxCapacity", DoubleValue(3.6));  // Q in Ah (3600mAh)

    batteryModel1->SetAttribute("NominalVoltage", DoubleValue(11.1));  // Vnom (3.7V per cell, 3S)
    batteryModel1->SetAttribute("NominalCapacity", DoubleValue(3.6));  // QNom in Ah

    batteryModel1->SetAttribute("ExponentialVoltage", DoubleValue(11.4)); // Vexp
    batteryModel1->SetAttribute("ExponentialCapacity", DoubleValue(1.8)); // Qexp (around 50% of the capacity)

    batteryModel1->SetAttribute("InternalResistance", DoubleValue(0.01));   // R in ohms
    batteryModel1->SetAttribute("TypicalDischargeCurrent", DoubleValue(20)); // i typical in A (20A)
    batteryModel1->SetAttribute("CutoffVoltage", DoubleValue(9.9));           // End of charge (3.3V per cell, 3S)


    // Capacity Ah(qMax) * (Vfull) voltage * 3600 = 9 * 11.1 * 3600 = 360 000
    /********************************BATTERY MODEL**********************************************/
    /********************************BATTERY MODEL**********************************************/

    Ptr<GenericBatteryModel> batteryModel2 = CreateObject<GenericBatteryModel>();

    batteryModel2->SetAttribute("FullVoltage", DoubleValue(12.6)); // Vfull (4.2V per cell, 3S)
    batteryModel2->SetAttribute("MaxCapacity", DoubleValue(3.6));  // Q in Ah (3600mAh)

    batteryModel2->SetAttribute("NominalVoltage", DoubleValue(11.1));  // Vnom (3.7V per cell, 3S)
    batteryModel2->SetAttribute("NominalCapacity", DoubleValue(3.6));  // QNom in Ah

    batteryModel2->SetAttribute("ExponentialVoltage", DoubleValue(11.4)); // Vexp
    batteryModel2->SetAttribute("ExponentialCapacity", DoubleValue(1.8)); // Qexp (around 50% of the capacity)

    batteryModel2->SetAttribute("InternalResistance", DoubleValue(0.01));   // R in ohms
    batteryModel2->SetAttribute("TypicalDischargeCurrent", DoubleValue(20)); // i typical in A (20A)
    batteryModel2->SetAttribute("CutoffVoltage", DoubleValue(9.9));           // End of charge (3.3V per cell, 3S)



    // Capacity Ah(qMax) * (Vfull) voltage * 3600 = 9 * 11.1 * 3600 = 360 000
    /********************************BATTERY MODEL**********************************************/
    /********************************BATTERY MODEL**********************************************/

    Ptr<GenericBatteryModel> batteryModel3 = CreateObject<GenericBatteryModel>();

    batteryModel3->SetAttribute("FullVoltage", DoubleValue(12.6)); // Vfull (4.2V per cell, 3S)
    batteryModel3->SetAttribute("MaxCapacity", DoubleValue(3.6));  // Q in Ah (3600mAh)

    batteryModel3->SetAttribute("NominalVoltage", DoubleValue(11.1));  // Vnom (3.7V per cell, 3S)
    batteryModel3->SetAttribute("NominalCapacity", DoubleValue(3.6));  // QNom in Ah

    batteryModel3->SetAttribute("ExponentialVoltage", DoubleValue(11.4)); // Vexp
    batteryModel3->SetAttribute("ExponentialCapacity", DoubleValue(1.8)); // Qexp (around 50% of the capacity)

    batteryModel3->SetAttribute("InternalResistance", DoubleValue(0.01));   // R in ohms
    batteryModel3->SetAttribute("TypicalDischargeCurrent", DoubleValue(20)); // i typical in A (20A)
    batteryModel3->SetAttribute("CutoffVoltage", DoubleValue(9.9));           // End of charge (3.3V per cell, 3S)


    // Capacity Ah(qMax) * (Vfull) voltage * 3600 = 9 * 11.1 * 3600 = 360 000
    /********************************BATTERY MODEL**********************************************/

    Ptr<GenericBatteryModel> batteryModel4 = CreateObject<GenericBatteryModel>();

    batteryModel4->SetAttribute("FullVoltage", DoubleValue(12.6)); // Vfull (4.2V per cell, 3S)
    batteryModel4->SetAttribute("MaxCapacity", DoubleValue(3.6));  // Q in Ah (3600mAh)

    batteryModel4->SetAttribute("NominalVoltage", DoubleValue(11.1));  // Vnom (3.7V per cell, 3S)
    batteryModel4->SetAttribute("NominalCapacity", DoubleValue(3.6));  // QNom in Ah

    batteryModel4->SetAttribute("ExponentialVoltage", DoubleValue(11.4)); // Vexp
    batteryModel4->SetAttribute("ExponentialCapacity", DoubleValue(1.8)); // Qexp (around 50% of the capacity)

    batteryModel4->SetAttribute("InternalResistance", DoubleValue(0.01));   // R in ohms
    batteryModel4->SetAttribute("TypicalDischargeCurrent", DoubleValue(20)); // i typical in A (20A)
    batteryModel4->SetAttribute("CutoffVoltage", DoubleValue(9.9));           // End of charge (3.3V per cell, 3S)



    // Capacity Ah(qMax) * (Vfull) voltage * 3600 = 9 * 11.1 * 3600 = 360 000 J
    /********************************BATTERY MODEL**********************************************/

    
    std::vector<Ptr<SimpleDeviceEnergyModel>> deviceEnergyModels;
    Ptr<SimpleDeviceEnergyModel> deviceEnergyModel1 = CreateObject<SimpleDeviceEnergyModel>();
    Ptr<SimpleDeviceEnergyModel> deviceEnergyModel2 = CreateObject<SimpleDeviceEnergyModel>();
    Ptr<SimpleDeviceEnergyModel> deviceEnergyModel3 = CreateObject<SimpleDeviceEnergyModel>();
    Ptr<SimpleDeviceEnergyModel> deviceEnergyModel4 = CreateObject<SimpleDeviceEnergyModel>();

    batteryModel1->SetNode(stas.Get(0));
    deviceEnergyModel1->SetEnergySource(batteryModel1);
    batteryModel1->AppendDeviceEnergyModel(deviceEnergyModel1);
    deviceEnergyModel1->SetNode(stas.Get(0));

    batteryModel2->SetNode(stas.Get(1));
    deviceEnergyModel2->SetEnergySource(batteryModel2);
    batteryModel2->AppendDeviceEnergyModel(deviceEnergyModel2);
    deviceEnergyModel2->SetNode(stas.Get(1));

    batteryModel3->SetNode(stas.Get(2));
    deviceEnergyModel3->SetEnergySource(batteryModel3);
    batteryModel3->AppendDeviceEnergyModel(deviceEnergyModel3);
    deviceEnergyModel3->SetNode(stas.Get(2));

    batteryModel4->SetNode(stas.Get(3));
    deviceEnergyModel4->SetEnergySource(batteryModel4);
    batteryModel4->AppendDeviceEnergyModel(deviceEnergyModel4);
    deviceEnergyModel4->SetNode(stas.Get(3));
    

    //****************************************************************************

    //MOBILITY STAS
    // Note that with FixedRssLossModel, the positions below are not
    // used for received signal strength.
    std::array<std::string, 4> startArrayX = {"1", "1", "50", "50"};
    std::array<std::string, 4> startArrayY = {"1", "50", "1", "50"};

    auto box1 = BoxValue(Box(0.0, 50.0, 0.0, 50.0, 0.0, 100.0));
    auto box2 = BoxValue(Box(0.0, 50.0, 50.0, 100.0, 0.0, 100.0));
    auto box3 = BoxValue(Box(50.0, 100.0, 0.0, 50.0, 0.0, 100.0));
    auto box4 = BoxValue(Box(50.0, 100.0, 50.0, 100.0, 0.0, 100.0));

    auto AoI1 = BoxValue(Box(10.0, 40.0, 10.0, 40.0, 5.0, 100.0));
    auto AoI2 = BoxValue(Box(10.0, 40.0, 60.0, 90.0, 5.0, 100.0));
    auto AoI3 = BoxValue(Box(60.0, 90.0, 10.0, 40.0, 5.0, 100.0));
    auto AoI4 = BoxValue(Box(60.0, 90.0, 60.0, 90.0, 5.0, 100.0));

    // Create the vector of drones
    std::vector<Drone> drones;

    double maxCapacityJ = 3.6 * 11.1 * 3600;

    // Create a Node, MobilityModel, and EnergyModel for each drone
    for (int i = 0; i < 4; i++) {
        // Create a Drone object and push it into the vector
        if (i == 0) {
            Drone drone(stas.Get(i), deviceEnergyModel1, maxCapacityJ, configPath, i);
            drones.push_back(drone);
        }
        if (i == 1) {
            Drone drone(stas.Get(i), deviceEnergyModel2, maxCapacityJ, configPath, i);
            drones.push_back(drone);
        }
        if (i == 2) {
            Drone drone(stas.Get(i), deviceEnergyModel3, maxCapacityJ, configPath, i);
            drones.push_back(drone);
        }
        if (i == 3) {
            Drone drone(stas.Get(i), deviceEnergyModel4, maxCapacityJ, configPath, i);
            drones.push_back(drone);
        }
    }
    
    for (int i = 0; i < 4; i++) {
        mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                      "X",
                                      DoubleValue(drones[i].getInitialX()),
                                      "Y",
                                      DoubleValue(drones[i].getInitialY()),
                                      "Rho",
                                      StringValue("ns3::UniformRandomVariable[Min=0|Max=0]"));
        
        if (i == 0) {                              
            mobility.SetMobilityModel("ns3::CustomMobilityModel",
                                      "maxHeight",
                                      DoubleValue(drones[i].getMaxHeight()),
                                      "AoI",
                                      BoxValue(drones[i].getAoI()),
                                      "Bounds",
                                      BoxValue(drones[i].getBounds()),
                                      "AvgVelocity",
                                      DoubleValue(drones[i].getSpeed()));
        }
        if (i == 1) {                              
            mobility.SetMobilityModel("ns3::CustomMobilityModel",
                                      "maxHeight",
                                      DoubleValue(drones[i].getMaxHeight()),
                                      "AoI",
                                      BoxValue(drones[i].getAoI()),
                                      "Bounds",
                                      BoxValue(drones[i].getBounds()),
                                      "AvgVelocity",
                                      DoubleValue(drones[i].getSpeed()));
        }
        if (i == 2) {                              
            mobility.SetMobilityModel("ns3::CustomMobilityModel",
                                      "maxHeight",
                                      DoubleValue(drones[i].getMaxHeight()),
                                      "AoI",
                                      BoxValue(drones[i].getAoI()),
                                      "Bounds",
                                      BoxValue(drones[i].getBounds()),
                                      "AvgVelocity",
                                      DoubleValue(drones[i].getSpeed()));
        }
        if (i == 3) {                              
            mobility.SetMobilityModel("ns3::CustomMobilityModel",
                                      "maxHeight",
                                      DoubleValue(drones[i].getMaxHeight()),
                                      "AoI",
                                      BoxValue(drones[i].getAoI()),
                                      "Bounds",
                                      BoxValue(drones[i].getBounds()),
                                      "AvgVelocity",
                                      DoubleValue(drones[i].getSpeed()));
        }
        //mobility->SetAttribute("Bounds", StringValue(boundArray[i]));
                                    
        mobility.Install(stas.Get(i));
    }

    //MOBILITY AP (STATIONARY AP)
    Ptr<ListPositionAllocator> positionAllocAP = CreateObject<ListPositionAllocator>();
    positionAllocAP->Add(Vector(50.0, 50.0, 0.0));
    mobilityAP.SetPositionAllocator(positionAllocAP);
    mobilityAP.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAP.Install(ap);

    AnimationInterface anim ("SimpleNS3Simulation_NetAnimationOutput.xml");

    for (uint32_t i = 0; i < 4; ++i) {
        anim.SetConstantPosition (stas.Get(i), 0, 0);
    }

    
    /////////////////////////////////////
    //            IP                   //
    /////////////////////////////////////

    InternetStackHelper internet;
    internet.Install(stas);
    internet.Install(ap);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    /////////////////////////////////////
    //            SOCKETS              //
    /////////////////////////////////////
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    Ptr<Socket> recvSinkL = Socket::CreateSocket(ap.Get(0), tid);
    InetSocketAddress localL = InetSocketAddress(Ipv4Address::GetAny(), 80);
    recvSinkL->Bind(localL);
    recvSinkL->SetRecvCallback(MakeCallback(&EdgeLogic));

    InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), 80);

    // Create an array of Ptr<Socket>
    std::vector<Ptr<Socket>> socketArray;

    // Create sockets for each node
    for (uint32_t i = 0; i < 4; ++i) {
        Ptr<Socket> socket = Socket::CreateSocket(stas.Get(i), tid);
        socket->SetAllowBroadcast(true);
        socket->Connect(remote);
        socketArray.push_back(socket);
    }

    // Tracing
    wifiPhy.EnablePcap("wifi-simple-infra", devices);

    //SET-UP THE SIMULATION

    // Output Drone data
    /*
    for (size_t i = 0; i < drones.size(); ++i) {
        std::cout << "Drone " << (i + 1) << " Data:" << std::endl;
        std::cout << "Weight: " << drones[i].getWeight() << " kg" << std::endl;
        std::cout << "Number of Propellers: " << drones[i].getNumbPropellers() << std::endl;
        std::cout << "Propellers Radius: " << drones[i].getPropellersRadius() << " meters" << std::endl;
        std::cout << "Speed: " << drones[i].getSpeed() << " m/s" << std::endl;
        std::cout << "Energy: " << drones[i].getEnergy() << " W/h" << std::endl;
        std::cout << "Hardware:" << std::endl;
        for (const auto& hwElement : drones[i].getHardware()) {
            std::cout << "  [" << hwElement[0] << ", " << hwElement[1] << ", " << hwElement[2] << "]" << std::endl;
        }
        std::cout << "ComputingPower Data:" << std::endl;
        std::cout << "Number of Local Iterations: " << drones[i].getNumLocalIter() << std::endl;
        //std::cout << "CPU Cycles x Data: " << drones[i].getCpuCycleXData() << std::endl;
        std::cout << "Number of Training Data Sets: " << drones[i].getNumbTrainDataSet() << std::endl;
        std::cout << "Switch Capacitance: " << drones[i].getSwitchCapacitance() << std::endl;
        //std::cout << "CPU Frequency: " << drones[i].getCpuFreq() << " GHz" << std::endl;
        std::cout << std::endl;
    }
    */

    //SIMULATION

    // Output what the simulation will do
    //std::cout << "Testing " << numPackets << " packets sent with receiver rss " << rss << " Number of Hosts: " << numbHosts << std::endl;

    if (systemId == 0) {
        for (uint32_t i = 0; i < 4; ++i) {
            
            Simulator::ScheduleWithContext(socketArray[i]->GetNode()->GetId(),
                                    Seconds(1.0),
                                    &DroneLogic,
                                    socketArray[i],
                                    packetSize,
                                    numPackets,
                                    interval,
                                    drones[i],
                                    12.6);
                                    
        }
    }


    Time now = Simulator::Now();
    //now += Seconds (105);
    now += Seconds (1000);
    Simulator::Stop(now);
    Simulator::Run();

    *infoLog << "Scenario Finished\n";

    Simulator::Destroy();

    // Exit the MPI execution environment
    MpiInterface::Disable ();

    std::string filename = "../results/results.csv";
    std::ofstream outFile(filename);

    if (outFile.is_open()) {
        for (const auto& str : v) {
            outFile << str << "\n";
        }
        outFile.close();
    } else {
        std::cerr << "Error write" << std::endl;
    }

    return 0;
}