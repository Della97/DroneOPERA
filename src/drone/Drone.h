#ifndef DRONE_H
#define DRONE_H

#include <vector>
#include "ns3/box.h" // For ns3::Box
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/simple-device-energy-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "../mobility/custom-mobility-model.h"

class Drone {
private:
    // Drone-specific fields
    double weight;
    double numbPropellers;
    double propellersRadius;
    double speed;  // Speed in meters per second
    double energy; // Energy in Watt-hours (W/h)
    std::vector<std::vector<double>> hardware; // Hardware vector with 2 elements, each containing 3 values

    // Mobility and bounds-related fields
    double maxHeight;
    ns3::Box bounds;
    ns3::Box aoi;
    double avgVelocity;

    // Initial starting coordinates
    double initialX;
    double initialY;
    double initialZ;

    // Computing power fields
    double numLocalIter;
    double cpuCyclexop;
    double opxdata;
    double voltage;
    double numbTrainDataSet;
    double switchCapacitance;
    double cpuFreq;

    // Energy calculation-related fields
    double hoverPower;
    double vertPower;
    double pDrag;
    double commPower;
    double commEnergy;

    // Comm fields
    double bandwidth;   //bandwidth
    double power;       //transmission power in watt
    double frequency;   //carrier dist freq
    double MLsize;      //size of local model

    // NS-3 related fields
    ns3::Ptr<ns3::Node> node;  // NS-3 Node reference
    ns3::Ptr<ns3::SimpleDeviceEnergyModel> energyModel;  // Pointer to SimpleDeviceEnergyModel
    double maxCapacity;

public:
    // Default Constructor
    Drone();

    // Constructor that initializes the drone with node, energy model, and data from JSON
    Drone(ns3::Ptr<ns3::Node> nodeRef, ns3::Ptr<ns3::SimpleDeviceEnergyModel> energyModelRef, double maxCapacityJ, const std::string& jsonFilePath, int index);

    // Setters for drone-specific fields
    void setWeight(double wt);
    void setNumbPropellers(double numProp);
    void setPropellersRadius(double propRadius);
    void setDragCoefficient(double dragCoeff);
    void setSpeed(double spd);
    void setEnergy(double eng);
    void setHardware(const std::vector<std::vector<double>>& hw);

    void setBandwidth(double b);
    void setWirelessTransmissionPower(double p);
    void setCarrierFrequency(double f);
    void setLocalModelSize(double s);

    // Setters for mobility and bounds fields
    void setMaxHeight(double height);
    void setBounds(const ns3::Box& bounds);
    void setAoI(const ns3::Box& aoi);
    void setAvgVelocity(double velocity);

    // Setters for initial starting coordinates
    void setInitialCoordinates(double x, double y, double z);

    // Setters for computing power fields
    void setNumLocalIter(double numIter);
    void setCpuCycleXop(double cxo);
    void setOpxData(double oxd);
    void setVoltage(double v);
    void setNumbTrainDataSet(double numTrainData);
    void setSwitchCapacitance(double switchCap);
    void setCpuFreq(double freq);

    // Setters for NS-3 Node and EnergyModel references
    void setNode(ns3::Ptr<ns3::Node> nodeRef);
    void setEnergyModel(ns3::Ptr<ns3::SimpleDeviceEnergyModel> energyModelRef);

    // Getters for drone-specific fields
    double getWeight() const;
    double getNumbPropellers() const;
    double getPropellersRadius() const;
    double getDragCoefficient() const;
    double getSpeed() const;
    double getEnergy() const;
    double getMaxCapacity() const;
    const std::vector<std::vector<double>>& getHardware() const;

    // Getters for mobility and bounds fields
    double getMaxHeight() const;
    ns3::Box getBounds() const;
    ns3::Box getAoI() const;
    double getAvgVelocity() const;

    // Getters for initial starting coordinates
    void getInitialCoordinates(double& x, double& y, double& z) const;
    double getInitialX()const;
    double getInitialY()const;
    double getInitialZ()const;

    // Getters for computing power fields
    double getNumLocalIter() const;
    double getCpuCycleXop() const;
    double getOpxData() const;
    double getVoltage() const;
    double getNumbTrainDataSet() const;
    double getSwitchCapacitance() const;
    double getCpuFreq() const;

    double getBandwidth() const;
    double getWirelessTransmissionPower() const;
    double getCarrierFrequency() const;
    double getLocalModelSize() const;

    // Getters for NS-3 Node and EnergyModel references
    ns3::Ptr<ns3::Node> getNode() const;
    ns3::Ptr<ns3::SimpleDeviceEnergyModel> getEnergyModel() const;

    // Energy calculation-related functions
    double calculateHoverPower();
    double calculateVertPower();
    double calculatePDrag();
    double calculateCommEnergy(double distance);
    double calculateComputePower();
    double calcMovePower(int state);
};

#endif // DRONE_H
