/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/Middleware.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/SensorData.h"
#include "artery/application/ObjectData.h"
#include "artery/application/Timer.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/envmod/sensor/FovSensor.h"
#include "artery/utility/FilterRules.h"
#include <inet/common/ModuleAccess.h>
#include <omnetpp/cxmlelement.h>
#include <boost/units/systems/si/prefixes.hpp>
#include <utility>

using namespace omnetpp;

namespace artery
{

/**** static local functions *****/

template<typename T, typename U>
static long roundToUnit(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

Define_Module(LocalEnvironmentModel)

static const simsignal_t EnvironmentModelRefreshSignal = cComponent::registerSignal("EnvironmentModel.refresh");

LocalEnvironmentModel::LocalEnvironmentModel() :
    mGlobalEnvironmentModel(nullptr)
{
}

int LocalEnvironmentModel::numInitStages() const
{
    return 2;
}

void LocalEnvironmentModel::initialize(int stage)
{
    if (stage == 0) {
        mGlobalEnvironmentModel = inet::getModuleFromPar<GlobalEnvironmentModel>(par("globalEnvironmentModule"), this);
        mGlobalEnvironmentModel->subscribe(EnvironmentModelRefreshSignal, this);

        auto vehicle = inet::findContainingNode(this);
        mMiddleware = inet::getModuleFromPar<Middleware>(par("middlewareModule"), vehicle);
        Facilities& fac = mMiddleware->getFacilities();
        fac.register_mutable(mGlobalEnvironmentModel);
        fac.register_mutable(this);
    } else if (stage == 1) {
        initializeSensors();

        mLocalDynamicMap = &mMiddleware->getFacilities().get_mutable<LocalDynamicMap>();
        mTimer = &mMiddleware->getFacilities().get_const<Timer>();

        // add static sensor information to LocalDynamicMap
        for (Sensor* sensor : mSensors) {
            if (auto fovSensor = dynamic_cast<const FovSensor*>(sensor)) {
                std::string type = fovSensor->getSensorCategory();
                std::string shape = "cone";
                FieldOfView fov = fovSensor->getFieldOfView();
                long range = roundToUnit(fov.range, vanetza::units::si::meter * boost::units::si::deci);
                int confidence = 100;
                bool shadowingApplies = true; // TODO: check if shadowing applies checking if line of sight is drawn

                SensorData sensorData { type, shape, range, confidence, shadowingApplies };
                mLocalDynamicMap->updateSensor(sensor->getId(), sensorData);
            }
        }
    }
}

void LocalEnvironmentModel::finish()
{
    mGlobalEnvironmentModel->unsubscribe(EnvironmentModelRefreshSignal, this);
    mObjects.clear();
}

void LocalEnvironmentModel::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal == EnvironmentModelRefreshSignal) {
        for (auto* sensor : mSensors) {
            sensor->measurement();
        }
        update();
    }
}

void LocalEnvironmentModel::complementObjects(const SensorDetection& detection, const Sensor& sensor)
{
    for (auto& detectedObject : detection.objects) {
        auto foundObject = mObjects.find(detectedObject);
        Tracking& tracking = foundObject->second;

        if (foundObject != mObjects.end()) {
            tracking.tap(&sensor);
        } else {
            mObjects.emplace(detectedObject, Tracking { ++mTrackingCounter, &sensor });
        }
    }

    // update LocalDynamicMap
    for (auto& detectedObject : detection.objects) {
        auto foundObject = mObjects.find(detectedObject);
        Tracking& tracking = foundObject->second;

        omnetpp::SimTime firstDetection = omnetpp::SimTime::getMaxTime();
        omnetpp::SimTime lastDetection = omnetpp::SimTime::ZERO;
        std::vector<int> sensorIds;

        for(const auto& sensor : tracking.sensors()) {
            firstDetection = std::min(firstDetection, sensor.second.first());
            lastDetection = std::max(lastDetection, sensor.second.last());
            sensorIds.push_back(sensor.first->getId());
        }

        const VehicleDataProvider& vdp = detectedObject->getVehicleData();
        Position position = detectedObject->getCentrePoint();
        vanetza::units::Velocity speed = vdp.speed();
        vanetza::units::Angle orientation = vdp.heading();
        vanetza::geonet::StationType stationType = vdp.getStationType();
        uint32_t stationId = vdp.getStationId();

        int64_t taiFirstDetection = countTaiMilliseconds(mTimer->getTimeFor(firstDetection));
        int64_t taiLastDetection = countTaiMilliseconds(mTimer->getTimeFor(lastDetection));

        ObjectData objectData { firstDetection, taiFirstDetection, lastDetection, taiLastDetection, position, speed, orientation, stationType, sensorIds };

        mLocalDynamicMap->updateObject(stationId, objectData);
    }
}

void LocalEnvironmentModel::update()
{
    for (auto it = mObjects.begin(); it != mObjects.end();) {
        const Object& object = it->first;
        Tracking& tracking = it->second;
        tracking.update();

        if (object.expired() || tracking.expired()) {
            it = mObjects.erase(it);
        } else {
            ++it;
        }
    }
}

void LocalEnvironmentModel::initializeSensors()
{
    cXMLElement* config = par("sensors").xmlValue();
    for (cXMLElement* sensor_cfg : config->getChildrenByTagName("sensor"))
    {
        cXMLElement* sensor_filters = sensor_cfg->getFirstChildWithTag("filters");
        bool sensor_applicable = true;
        if (sensor_filters) {
            auto identity = mMiddleware->getIdentity();
            FilterRules rules(getRNG(0), identity);
            sensor_applicable = rules.applyFilterConfig(*sensor_filters);
        }

        if (sensor_applicable) {
            cModuleType* module_type = cModuleType::get(sensor_cfg->getAttribute("type"));
            const char* sensor_name = sensor_cfg->getAttribute("name");
            if (!sensor_name || !*sensor_name) {
                sensor_name = module_type->getName();
            }

            cModule* module = module_type->create(sensor_name, this);
            module->finalizeParameters();
            module->buildInside();
            auto sensor = dynamic_cast<artery::Sensor*>(module);

            if (sensor != nullptr) {
                // set sensor name at very early stage so it is available during sensor initialization
                sensor->setSensorName(sensor_name);
            } else {
                throw cRuntimeError("%s is not of type Sensor", module_type->getFullName());
            }

            module->scheduleStart(simTime());
            module->callInitialize();
            mSensors.push_back(sensor);
        }
    }
}


LocalEnvironmentModel::Tracking::Tracking(int id, const Sensor* sensor) : mId(id)
{
    mSensors.emplace(sensor, TrackingTime {});
}

bool LocalEnvironmentModel::Tracking::expired() const
{
    return mSensors.empty();
}

void LocalEnvironmentModel::Tracking::update()
{
    for (auto it = mSensors.begin(); it != mSensors.end();) {
      const Sensor* sensor = it->first;
      const TrackingTime& tracking = it->second;

      const bool expired = tracking.last() + sensor->getValidityPeriod() < simTime();
      if (expired) {
          it = mSensors.erase(it);
      } else {
          ++it;
      }
    }
}

void LocalEnvironmentModel::Tracking::tap(const Sensor* sensor)
{
    auto found = mSensors.find(sensor);
    if (found != mSensors.end()) {
         TrackingTime& tracking = found->second;
         tracking.tap();
    } else {
         mSensors.emplace(sensor, TrackingTime {});
    }
}


LocalEnvironmentModel::TrackingTime::TrackingTime() :
   mFirst(simTime()), mLast(simTime())
{
}

void LocalEnvironmentModel::TrackingTime::tap()
{
    mLast = simTime();
}


TrackedObjectsFilterRange filterBySensorCategory(const LocalEnvironmentModel::TrackedObjects& all, const std::string& category)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByCategory = [category](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&category](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorCategory() == category;
                });
    };

    auto begin = boost::make_filter_iterator(seenByCategory, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByCategory, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

TrackedObjectsFilterRange filterBySensorName(const LocalEnvironmentModel::TrackedObjects& all, const std::string& name)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByName = [name](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&name](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorName() == name;
                });
    };

    auto begin = boost::make_filter_iterator(seenByName, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByName, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

} // namespace artery
