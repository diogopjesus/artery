#ifndef ARTERY_OBJECTDATA_H_
#define ARTERY_OBJECTDATA_H_

#include <omnetpp/simtime.h>
#include <vector>

namespace artery
{

struct ObjectData
{
    const omnetpp::SimTime firstDetection;
    const omnetpp::SimTime lastDetection;
    const uint32_t xCoordinate;
    const uint32_t yCoordinate;
    const std::vector<int> sensors;
};

} // namespace artery

#endif /* ARTERY_OBJECTDATA_H_ */