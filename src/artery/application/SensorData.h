#ifndef ARTERY_SENSORDATA_H_
#define ARTERY_SENSORDATA_H_

#include "artery/utility/Geometry.h"
#include <string>


namespace artery
{

struct SensorData
{
    const std::string type;
    const Position referencePoint;
    const std::string shape;
    const int range;
    const int confidence;
    const bool shadowingApplies;
};

} // namespace artery

#endif /* ARTERY_SENSORDATA_H_ */