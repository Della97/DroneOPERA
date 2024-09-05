#ifndef JSONPARSER_H
#define JSONPARSER_H

#include "../drone/Drone.h"
#include "rapidjson/document.h"
#include <vector>
#include <string>

class Drone;

class JsonParser {
public:
    bool parseJson(const std::string& filename, Drone& drone, int index);
};

#endif // JSONPARSER_H
