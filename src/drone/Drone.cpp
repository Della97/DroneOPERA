#include "Drone.h"
#include "../energy/energy.h"
#include "../parser/JsonParser.h"
#include <iostream>

// Default Constructor
Drone::Drone() : weight(0), numbPropellers(0), propellersRadius(0), speed(0), energy(0), 
                 maxHeight(0), avgVelocity(0), initialX(0), initialY(0), initialZ(0), 
                 numLocalIter(0), numbTrainDataSet(0), switchCapacitance(0), cpuFreq(0),
                 hoverPower(0), vertPower(0), pDrag(0), commPower(0), commEnergy(0) {}

// Constructor that initializes the drone with node, energy model, and data from JSON
Drone::Drone(ns3::Ptr<ns3::Node> nodeRef, ns3::Ptr<ns3::SimpleDeviceEnergyModel> energyModelRef, double maxCapacityJ, const std::string& jsonFilePath, int index)
    : node(nodeRef), energyModel(energyModelRef), maxCapacity(maxCapacityJ) {
    // Initialize the fields using the JSON parser
    JsonParser parser;
    if (!parser.parseJson(jsonFilePath, *this, index)) {
        std::cerr << "Error parsing JSON for Drone at index " << index << std::endl;
    }
}

// Setters for drone-specific fields
void Drone::setWeight(double wt) { weight = wt; }
void Drone::setNumbPropellers(double numProp) { numbPropellers = numProp; }
void Drone::setPropellersRadius(double propRadius) { propellersRadius = propRadius; }
void Drone::setDragCoefficient(double dragCoeff) { pDrag = dragCoeff; }
void Drone::setSpeed(double spd) { speed = spd; }
void Drone::setEnergy(double eng) { energy = eng; }
void Drone::setHardware(const std::vector<std::vector<double>>& hw) { hardware = hw; }

// Setters for mobility and bounds-related fields
void Drone::setMaxHeight(double height) { maxHeight = height; }
void Drone::setBounds(const ns3::Box& bounds) { this->bounds = bounds; }
void Drone::setAoI(const ns3::Box& aoi) { this->aoi = aoi; }
void Drone::setAvgVelocity(double velocity) { avgVelocity = velocity; }

// Setters for initial starting coordinates
void Drone::setInitialCoordinates(double x, double y, double z) {
    initialX = x;
    initialY = y;
    initialZ = z;
}

// Setters for computing power fields
void Drone::setNumLocalIter(double numIter) { numLocalIter = numIter; }
void Drone::setCpuCycleXop(double cxo) { cpuCyclexop = cxo; }
void Drone::setOpxData(double oxd) { opxdata = oxd; }
void Drone::setVoltage(double v) { voltage = v; }
void Drone::setNumbTrainDataSet(double numTrainData) { numbTrainDataSet = numTrainData; }
void Drone::setSwitchCapacitance(double switchCap) { switchCapacitance = switchCap; }
void Drone::setCpuFreq(double freq) { cpuFreq = freq; }

void Drone::setBandwidth(double b) { bandwidth = b; }
void Drone::setWirelessTransmissionPower(double p) { power = p; }
void Drone::setCarrierFrequency(double f) { frequency = f;}
void Drone::setLocalModelSize(double s) { MLsize = s; }

// Setters for NS-3 Node and EnergyModel references
void Drone::setNode(ns3::Ptr<ns3::Node> nodeRef) {
    node = nodeRef;
}

void Drone::setEnergyModel(ns3::Ptr<ns3::SimpleDeviceEnergyModel> energyModelRef) {
    energyModel = energyModelRef;
}

// Getters for drone-specific fields
double Drone::getWeight() const { return weight; }
double Drone::getNumbPropellers() const { return numbPropellers; }
double Drone::getPropellersRadius() const { return propellersRadius; }
double Drone::getDragCoefficient() const { return pDrag;}
double Drone::getSpeed() const { return speed; }
double Drone::getEnergy() const { return energy; }
double Drone::getMaxCapacity() const { return maxCapacity; }
const std::vector<std::vector<double>>& Drone::getHardware() const { return hardware; }

double Drone::getBandwidth() const { return bandwidth; }
double Drone::getWirelessTransmissionPower() const { return power; }
double Drone::getCarrierFrequency() const { return frequency;}
double Drone::getLocalModelSize() const { return MLsize; }

// Getters for computing power fields
double Drone::getNumLocalIter() const { return numLocalIter; }
double Drone::getCpuCycleXop() const { return cpuCyclexop; }
double Drone::getOpxData() const { return opxdata; }
double Drone::getVoltage() const { return voltage; }
double Drone::getNumbTrainDataSet() const { return numbTrainDataSet; }
double Drone::getSwitchCapacitance() const { return switchCapacitance; }
double Drone::getCpuFreq() const { return cpuFreq; }

// Getters for NS-3 Node and EnergyModel references
ns3::Ptr<ns3::Node> Drone::getNode() const { return node; }
ns3::Ptr<ns3::SimpleDeviceEnergyModel> Drone::getEnergyModel() const { return energyModel; }


//************************************************************************************************************************

/*
// Energy calculation-related functions
double Drone::calculateHoverPower() {
    //return calcHoverPower(weight, propellersRadius, numbPropellers);
    //double radiusPropellers, double numbProp, double dragCoeff, double mass, double speed
    std::cout << calcHorizontal(propellersRadius, numbPropellers, pDrag, weight, speed) << std::endl;
    return calcHorizontal(propellersRadius, numbPropellers, pDrag, weight, speed);
}

double Drone::calculateVertPower() {
    //return calcVertPower(weight, speed);
    //std::cout << calcAscend(propellersRadius, numbPropellers, pDrag, weight, speed) << std::endl;
    return calcAscend(propellersRadius, numbPropellers, pDrag, weight, speed);
}

double Drone::calculatePDrag() {
    auto mobility = node->GetObject<ns3::CustomMobilityModel>();
    if ((mobility->getState()) == 1) {
        return calcPDrag(propellersRadius, numbPropellers, pDrag, speed, 0);
    }
    return calcPDrag(propellersRadius, numbPropellers, pDrag, 0, speed);
}
*/

double Drone::calcMovePower(int state) {
    if (state == 0) return P_UAV(weight, pDrag, propellersRadius, numbPropellers, speed, speed, speed);
    if (state == 1) return P_UAV(weight, pDrag, propellersRadius, numbPropellers, speed, 0, 0);
    if (state == 2) return P_UAV(weight, pDrag, propellersRadius, numbPropellers, speed, 0, 0);
    if (state == 3) return P_UAV(weight, pDrag, propellersRadius, numbPropellers, 0, 0, speed);
    return 0;
}

//***********************************************************************************************************************

double Drone::calculateCommEnergy(double distance) {
    return calcCommEnergy(power, MLsize, bandwidth, frequency, distance);
}

double Drone::calculateComputePower(){
    return calcCompPower(switchCapacitance, voltage, cpuCyclexop, opxdata, numbTrainDataSet, numLocalIter);
    //return calcCompPower(8e-11, 1.3, 2, 1000000, 60, 10000);
}



double Drone::getMaxHeight() const{ return maxHeight;}
ns3::Box Drone::getBounds() const{ return bounds;}
ns3::Box Drone::getAoI() const{ return aoi;}
double Drone::getAvgVelocity() const{ return avgVelocity;}

// Getters for initial starting coordinates
void Drone::getInitialCoordinates(double& x, double& y, double& z) const {
    x = initialX;
    y = initialY;
    z = initialZ;
}

double Drone::getInitialX() const { return initialX;}
double Drone::getInitialY() const { return initialY;}
double Drone::getInitialZ() const { return initialZ;}
