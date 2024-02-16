#ifndef ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT
#define ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT

#include "artery/application/CaObject.h"
#include "artery/application/CpObject.h"
#include "artery/application/DataObject.h"
#include "artery/application/SensorData.h"
#include "artery/application/ObjectData.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>

namespace artery
{

class Timer;

class LocalDynamicMap
{
public:
    using StationId = uint32_t;
    using SensorId = uint32_t;
    using ObjectId = uint32_t;
    using Cam = vanetza::asn1::Cam;
    using CamPredicate = std::function<bool(const Cam&)>;
    using Cpm = vanetza::asn1::Cpm;
    using CpmPredicate = std::function<bool(const Cpm&)>;
    using SensorPredicate = std::function<bool(const SensorData&)>;
    using ObjectPredicate = std::function<bool(const ObjectData&)>;

    class AwarenessEntry
    {
    public:
        AwarenessEntry(const CaObject&, omnetpp::SimTime);
        AwarenessEntry(AwarenessEntry&&) = default;
        AwarenessEntry& operator=(AwarenessEntry&&) = default;

        omnetpp::SimTime expiry() const { return mExpiry; }
        const Cam& cam() const { return mObject.asn1(); }
        std::shared_ptr<const Cam> camPtr() const { return mObject.shared_ptr(); }

    private:
        omnetpp::SimTime mExpiry;
        CaObject mObject;
    };

    class PerceptionEntry
    {
    public:
        PerceptionEntry(const CpObject&, omnetpp::SimTime);
        PerceptionEntry(PerceptionEntry&&) = default;
        PerceptionEntry& operator=(PerceptionEntry&&) = default;

        omnetpp::SimTime expiry() const { return mExpiry; }
        const Cpm& cpm() const { return mObject.asn1(); }
        std::shared_ptr<const Cpm> cpmPtr() const { return mObject.shared_ptr(); }

    private:
        omnetpp::SimTime mExpiry;
        CpObject mObject;
    };

    template<typename T>
    class DataObjectEntry
    {
    public:
        DataObjectEntry(const DataObject<T>&, omnetpp::SimTime);
        DataObjectEntry(DataObjectEntry&&) = default;
        DataObjectEntry& operator=(DataObjectEntry&&) = default;

        omnetpp::SimTime expiry() const { return mExpiry; }
        const T& object() const { return mObject.value(); }
        std::shared_ptr<const T> objectPtr() const { return mObject.shared_ptr(); }

    private:
        omnetpp::SimTime mExpiry;
        DataObject<T> mObject;
    };

    using AwarenessEntries = std::map<StationId, AwarenessEntry>;
    using PerceptionEntries = std::map<StationId, PerceptionEntry>;
    using SensorEntries = std::map<SensorId, DataObjectEntry<SensorData>>;
    using ObjectEntries = std::map<StationId, DataObjectEntry<ObjectData>>;

    LocalDynamicMap(const Timer&);
    void updateAwareness(const CaObject&);
    void updatePerception(const CpObject&);
    void updateSensor(const SensorId&, const SensorData&);
    void updateObject(const ObjectId&, const ObjectData&);
    void dropExpired();
    unsigned countCams(const CamPredicate&) const;
    unsigned countCpms(const CpmPredicate&) const;
    unsigned countSensors(const SensorPredicate&) const;
    unsigned countObjects(const ObjectPredicate&) const;
    unsigned totalObjects() const;
    std::shared_ptr<const Cam> getCam(StationId) const;
    std::shared_ptr<const Cpm> getCpm(StationId) const;
    std::shared_ptr<const SensorData> getSensorData(SensorId) const;
    std::shared_ptr<const ObjectData> getObjectData(ObjectId) const;
    const AwarenessEntries& allCamEntries() const { return mCaMessages; }
    const PerceptionEntries& allCpmEntries() const { return mCpMessages; }
    const SensorEntries& allSensorEntries() const { return mSensors; }
    const ObjectEntries& allObjectEntries() const { return mObjects; }

private:
    const Timer& mTimer;
    AwarenessEntries mCaMessages;
    PerceptionEntries mCpMessages;
    SensorEntries mSensors;
    ObjectEntries mObjects;
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

