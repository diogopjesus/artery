#ifndef ARTERY_OBJECTDATA_H_
#define ARTERY_OBJECTDATA_H_

#include "artery/utility/Geometry.h"
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/geonet/station_type.hpp>
#include <omnetpp/simtime.h>
#include <vector>

namespace artery
{

struct ObjectData
{
    const omnetpp::SimTime firstDetection;
    const int64_t taiFirstDetection;
    const omnetpp::SimTime lastDetection;
    const int64_t taiLastDetection;
    const Position position;
    const vanetza::units::Velocity speed;
    const vanetza::units::Angle orientation;
    const vanetza::geonet::StationType stationType;
    const std::vector<int> sensors;
};

} // namespace artery

#endif /* ARTERY_OBJECTDATA_H_ */