/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/CpmSensor.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/CpObject.h"
#include "artery/application/Middleware.h"
#include "artery/utility/IdentityRegistry.h"
#include <inet/common/ModuleAccess.h>

using namespace omnetpp;

namespace artery
{

static const simsignal_t CpmReceivedSignal = cComponent::registerSignal("CpmReceived");

Define_Module(CpmSensor)

void CpmSensor::initialize()
{
    mValidityPeriod = par("validityPeriod");
    BaseSensor::initialize();
    mIdentityRegistry = inet::getModuleFromPar<IdentityRegistry>(par("identityRegistryModule"), this);
    getMiddleware().subscribe(CpmReceivedSignal, this);
}

void CpmSensor::finish()
{
    getMiddleware().unsubscribe(CpmReceivedSignal, this);
    BaseSensor::finish();
}

void CpmSensor::measurement()
{
    Enter_Method("measurement");
}

void CpmSensor::receiveSignal(cComponent*, simsignal_t signal, cObject *obj, cObject*)
{
    if (signal == CpmReceivedSignal) {
        auto* cpm = dynamic_cast<CpObject*>(obj);
        if (cpm) {
            uint32_t stationID = cpm->asn1()->header.stationId;
            auto identity = mIdentityRegistry->lookup<IdentityRegistry::application>(stationID);
            if (identity) {
                auto object = mGlobalEnvironmentModel->getObject(identity->traci);
                SensorDetection detection;
                detection.objects.push_back(object);
                mLocalEnvironmentModel->complementObjects(detection, *this);
            } else {
                EV_WARN << "Unknown identity for station ID " << stationID;
            }
        } else {
            EV_ERROR << "received signal has no CaObject";
        }
    }
}

omnetpp::SimTime CpmSensor::getValidityPeriod() const
{
    return mValidityPeriod;
}

const std::string& CpmSensor::getSensorCategory() const
{
    static const std::string category = "CP";
    return category;
}

SensorDetection CpmSensor::detectObjects() const
{
    // return empty sensor detection because CPM objects are added upon CPM reception signal
    SensorDetection detection;
    return detection;
}

} // namespace artery
