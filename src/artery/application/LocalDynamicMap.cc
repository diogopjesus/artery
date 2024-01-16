#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include <omnetpp/csimulation.h>
#include <cassert>
#include <algorithm>

namespace artery
{

LocalDynamicMap::LocalDynamicMap(const Timer& timer) :
    mTimer(timer)
{
}

void LocalDynamicMap::updateAwareness(const CaObject& obj)
{
    const vanetza::asn1::Cam& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->cam.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received CAM is out of bounds";
        return;
    }

    AwarenessEntry entry(obj, expiry);
    auto found = mCaMessages.find(msg->header.stationId);
    if (found != mCaMessages.end()) {
        found->second = std::move(entry);
    } else {
        mCaMessages.emplace(msg->header.stationId, std::move(entry));
    }
}

void LocalDynamicMap::updatePerception(const CpObject& obj)
{
    const vanetza::asn1::Cpm& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };

    // IMPORTANT NOTE:
    // 42 bit sequence number (0..4398046511103). Minimum 6 bytes required.
    static_assert(sizeof(unsigned long) >= 6, "unsigned long cannot represent ReferenceTime");
    const INTEGER_t& referenceTime = msg->payload.managementContainer.referenceTime;
    unsigned long genTime;
    if(asn_INTEGER2ulong(&referenceTime, &genTime) != 0) {
        EV_STATICCONTEXT
        EV_WARN << "Could not convert ReferenceTime to unsigned long";
        return;
    }

    vanetza::Clock::time_point tai = vanetza::Clock::time_point { std::chrono::milliseconds(genTime) };
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received CPM is out of bounds";
        return;
    }

    PerceptionEntry entry(obj, expiry);
    auto found = mCpMessages.find(msg->header.stationId);
    if (found != mCpMessages.end()) {
        found->second = std::move(entry);
    } else {
        mCpMessages.emplace(msg->header.stationId, std::move(entry));
    }    
}

void LocalDynamicMap::updateSensor(const SensorId& sId, const SensorData& data)
{
    static const omnetpp::SimTime lifetime = omnetpp::SimTime::getMaxTime();
    const omnetpp::SimTime expiry = lifetime;

    DataObject<SensorData> obj(data);

    DataObjectEntry<SensorData> entry(std::move(obj), expiry);
    auto found = mSensors.find(sId);
    if (found != mSensors.end()) {
        found->second = std::move(entry);
    } else {
        mSensors.emplace(sId, std::move(entry));
    }
}

void LocalDynamicMap::updateObject(const ObjectId& oId, const ObjectData& data)
{
    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    const omnetpp::SimTime expiry = data.lastDetection + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of object is out of bounds";
        return;
    }

    DataObject<ObjectData> obj(data);

    DataObjectEntry<ObjectData> entry(std::move(obj), expiry);
    auto found = mObjects.find(oId);
    if (found != mObjects.end()) {
        found->second = std::move(entry);
    } else {
        mObjects.emplace(oId, std::move(entry));
    }
}

void LocalDynamicMap::dropExpired()
{
    const auto now = omnetpp::simTime();
    for (auto it = mCaMessages.begin(); it != mCaMessages.end();) {
        if (it->second.expiry() < now) {
            it = mCaMessages.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = mCpMessages.begin(); it != mCpMessages.end();) {
        if (it->second.expiry() < now) {
            it = mCpMessages.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = mSensors.begin(); it != mSensors.end();) {
        if (it->second.expiry() < now) {
            it = mSensors.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = mObjects.begin(); it != mObjects.end();) {
        if (it->second.expiry() < now) {
            it = mObjects.erase(it);
        } else {
            ++it;
        }
    }
}

unsigned LocalDynamicMap::countCams(const CamPredicate& predicate) const
{
    return std::count_if(mCaMessages.begin(), mCaMessages.end(),
            [&predicate](const AwarenessEntries::value_type& map_entry) {
                return predicate(map_entry.second.cam());
            });
}

std::shared_ptr<const LocalDynamicMap::Cam> LocalDynamicMap::getCam(StationId stationId) const
{
    auto cam = mCaMessages.find(stationId);
    if (cam != mCaMessages.end()) {
        return cam->second.camPtr();
    }

    return nullptr;
}

std::shared_ptr<const LocalDynamicMap::Cpm> LocalDynamicMap::getCpm(StationId stationId) const
{
    auto cpm = mCpMessages.find(stationId);
    if (cpm != mCpMessages.end()) {
        return cpm->second.cpmPtr();
    }

    return nullptr;
}

LocalDynamicMap::AwarenessEntry::AwarenessEntry(const CaObject& obj, omnetpp::SimTime t) :
    mExpiry(t), mObject(obj)
{
}

LocalDynamicMap::PerceptionEntry::PerceptionEntry(const CpObject& obj, omnetpp::SimTime t) :
    mExpiry(t), mObject(obj)
{
}

template<typename T>
LocalDynamicMap::DataObjectEntry<T>::DataObjectEntry(const DataObject<T>& obj, omnetpp::SimTime t) :
    mExpiry(t), mObject(obj)
{
}

} // namespace artery
