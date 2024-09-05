#include "JsonParser.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"
#include <iostream>
#include <cstdio>

bool JsonParser::parseJson(const std::string& filename, Drone& drone, int index) {
    FILE* fp = fopen(filename.c_str(), "r");
    if (!fp) {
        std::cerr << "Could not open file for reading." << std::endl;
        return false;
    }

    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    // Parse the JSON document
    rapidjson::Document document;
    document.ParseStream(is);
    fclose(fp);

    // Ensure the document is an object
    if (!document.IsObject()) {
        std::cerr << "Invalid JSON format." << std::endl;
        return false;
    }

    // Parse the "Drones" array
    if (document.HasMember("Drones") && document["Drones"].IsArray()) {
        const rapidjson::Value& dronesArray = document["Drones"];

        if (index >= dronesArray.Size()) {
            std::cerr << "Index out of bounds for drones array." << std::endl;
            return false;
        }

        const rapidjson::Value& droneObj = dronesArray[index];

        // Set basic drone properties
        if (droneObj.HasMember("weight") && droneObj["weight"].IsDouble()) {
            drone.setWeight(droneObj["weight"].GetDouble());
        }

        if (droneObj.HasMember("numbPropellers") && droneObj["numbPropellers"].IsDouble()) {
            drone.setNumbPropellers(droneObj["numbPropellers"].GetDouble());
        }

        if (droneObj.HasMember("propellersRadius") && droneObj["propellersRadius"].IsDouble()) {
            drone.setPropellersRadius(droneObj["propellersRadius"].GetDouble());
        }

        if (droneObj.HasMember("speed") && droneObj["speed"].IsDouble()) {
            drone.setSpeed(droneObj["speed"].GetDouble());
        }

        if (droneObj.HasMember("energy") && droneObj["energy"].IsDouble()) {
            drone.setEnergy(droneObj["energy"].GetDouble());
        }

        if (droneObj.HasMember("dragCoefficient") && droneObj["dragCoefficient"].IsDouble()) {
            drone.setDragCoefficient(droneObj["dragCoefficient"].GetDouble());
        }

        // Set computing power properties
        if (droneObj.HasMember("numLocalIter") && droneObj["numLocalIter"].IsDouble()) {
            drone.setNumLocalIter(droneObj["numLocalIter"].GetDouble());
        }

        if (droneObj.HasMember("cpuCyclePerOperation") && droneObj["cpuCyclePerOperation"].IsDouble()) {
            drone.setCpuCycleXop(droneObj["cpuCyclePerOperation"].GetDouble());
        }

        if (droneObj.HasMember("operationPerData") && droneObj["operationPerData"].IsDouble()) {
            drone.setOpxData(droneObj["operationPerData"].GetDouble());
        }

        // Parse and set hardware vector
        if (droneObj.HasMember("hardware") && droneObj["hardware"].IsArray()) {
            const rapidjson::Value& hardwareArray = droneObj["hardware"];
            std::vector<std::vector<double>> hardware;

            for (rapidjson::SizeType j = 0; j < hardwareArray.Size(); ++j) {
                const rapidjson::Value& hardwareElement = hardwareArray[j];
                std::vector<double> hwElement;

                for (rapidjson::SizeType k = 0; k < hardwareElement.Size(); ++k) {
                    hwElement.push_back(hardwareElement[k].GetDouble());
                }
                hardware.push_back(hwElement);
            }
            drone.setHardware(hardware);
        }

        if (droneObj.HasMember("numbTrainDataSet") && droneObj["numbTrainDataSet"].IsDouble()) {
            drone.setNumbTrainDataSet(droneObj["numbTrainDataSet"].GetDouble());
        }

        if (droneObj.HasMember("switchCapacitance") && droneObj["switchCapacitance"].IsDouble()) {
            drone.setSwitchCapacitance(droneObj["switchCapacitance"].GetDouble());
        }

        if (droneObj.HasMember("cpuFreq") && droneObj["cpuFreq"].IsDouble()) {
            drone.setCpuFreq(droneObj["cpuFreq"].GetDouble());
        }

        if (droneObj.HasMember("voltage") && droneObj["voltage"].IsDouble()) {
            drone.setVoltage(droneObj["voltage"].GetDouble());
        }

        // Set mobility properties
        if (droneObj.HasMember("maxHeight") && droneObj["maxHeight"].IsDouble()) {
            drone.setMaxHeight(droneObj["maxHeight"].GetDouble());
        }

        if (droneObj.HasMember("bounds") && droneObj["bounds"].IsObject()) {
            const rapidjson::Value& boundsObj = droneObj["bounds"];
            ns3::Box bounds(
                boundsObj["xMin"].GetDouble(),
                boundsObj["xMax"].GetDouble(),
                boundsObj["yMin"].GetDouble(),
                boundsObj["yMax"].GetDouble(),
                boundsObj["zMin"].GetDouble(),
                boundsObj["zMax"].GetDouble()
            );
            drone.setBounds(bounds);
        }

        if (droneObj.HasMember("aoi") && droneObj["aoi"].IsObject()) {
            const rapidjson::Value& aoiObj = droneObj["aoi"];
            ns3::Box aoi(
                aoiObj["xMin"].GetDouble(),
                aoiObj["xMax"].GetDouble(),
                aoiObj["yMin"].GetDouble(),
                aoiObj["yMax"].GetDouble(),
                aoiObj["zMin"].GetDouble(),
                aoiObj["zMax"].GetDouble()
            );
            drone.setAoI(aoi);
        }

        if (droneObj.HasMember("avgVelocity") && droneObj["avgVelocity"].IsDouble()) {
            drone.setAvgVelocity(droneObj["avgVelocity"].GetDouble());
        }

        // Set initial coordinates
        if (droneObj.HasMember("initialCoordinates") && droneObj["initialCoordinates"].IsObject()) {
            const rapidjson::Value& coordObj = droneObj["initialCoordinates"];
            double x = coordObj["x"].GetDouble();
            double y = coordObj["y"].GetDouble();
            double z = coordObj["z"].GetDouble();
            drone.setInitialCoordinates(x, y, z);
        }

        // Set wireless communication and federated learning properties
        if (droneObj.HasMember("bandwidth") && droneObj["bandwidth"].IsDouble()) {
            drone.setBandwidth(droneObj["bandwidth"].GetDouble());
        }

        if (droneObj.HasMember("wirelessTransmissionPower") && droneObj["wirelessTransmissionPower"].IsDouble()) {
            drone.setWirelessTransmissionPower(droneObj["wirelessTransmissionPower"].GetDouble());
        }

        if (droneObj.HasMember("carrierFrequency") && droneObj["carrierFrequency"].IsDouble()) {
            drone.setCarrierFrequency(droneObj["carrierFrequency"].GetDouble());
        }

        if (droneObj.HasMember("localModelSize") && droneObj["localModelSize"].IsDouble()) {
            drone.setLocalModelSize(droneObj["localModelSize"].GetDouble());
        }

    } else {
        std::cerr << "Drones array not found in JSON." << std::endl;
        return false;
    }

    return true;
}
