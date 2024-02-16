/*
 * Artery V2X Simulation Framework
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_CPSERVICE_H_
#define ARTERY_CPSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/btp/data_request.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <unordered_set>
#include <map>
#include <vector>

namespace artery
{

class VehicleDataProvider;
class Timer;
class LocalDynamicMap;

class ObjectIdHandler
{
public:
    ObjectIdHandler();

    /**
     * Checks if object (represented with the omnetpp identifier) is already assigned to an object id.
     * If so, returns the respective object id.
     * If not, assigns a new id to the object and return it.
     * @param oId omnetpp identifier of the object
     * @param T_now current simulation time
     * @return assigned perceived object id
    */
    uint16_t getOrAssign(int64_t, int64_t);

    void setRetentionPeriod(int);

private:
    std::map<int16_t, int64_t> mMap;
    int64_t mTimestamps[65535];
    int mRetentionPeriod;
    // TODO: implement this with a dynamic reference time to reduce total size of timestamps array
    //       currently the array total size is 524280 bytes (65535 * 8 bytes)
    //       if we use a dynamic reference time, we can reduce this to 262140 bytes (65535 * 4 bytes) + 8 bytes
    //       the 8 bytes are for the reference time
};


class CpService : public ItsG5Service
{
public:
    CpService();
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>, const NetworkInterface&) override;
    void trigger() override;

private:
    void updateFrequencyAndContentManagement(const omnetpp::SimTime&);
    void updateObjectPerceptionQuality(const omnetpp::SimTime&);
    void checkTriggeringConditions(const omnetpp::SimTime&);

    void sendCpm(const omnetpp::SimTime&);

    int calcObjectPerceptionQuality(uint32_t,int,int,int);
    std::map<uint32_t, double> m_cEMA;
    std::map<uint32_t, double> m_dEMA;
    std::map<uint32_t, int> mObjectPerceptionQuality;

    bool checkSensorInformationInclusion(const omnetpp::SimTime&);
    bool checkPerceptionRegionInclusion(const omnetpp::SimTime&);
    bool checkPerceivedObjectInclusion(const omnetpp::SimTime&);

    std::vector<uint32_t> mSelectedObjects;
    std::vector<uint32_t>::iterator mSelectedObjectsStart;
    std::vector<uint32_t>::iterator mSelectedObjectsEnd;

    void selectObjectsForTransmission(const omnetpp::SimTime&);
    int calcValueOfInformation(uint32_t);

    /* NOTE: not the most efficient way to implement, id is replicated 4 times */
    std::map<uint32_t, omnetpp::SimTime> mLastInclusionTimes;
    std::map<uint32_t, Position> mLastPositions;
    std::map<uint32_t, vanetza::units::Velocity> mLastSpeeds;
    std::map<uint32_t, vanetza::units::Angle> mLastOrientations;

    double calcObjectUtilityFunction(const omnetpp::SimTime&, uint32_t);

    // double objectUtilityFunction();
    // int valueOfInformation();
    // int objectPerceptionQuality();

    // /* Selected objects to be sent in a CPM */
    // std::vector<std::shared_ptr<const ObjectData>> mSelectedObjects;
    // std::vector<std::shared_ptr<const ObjectData>> mOffloadedObjects;

    ChannelNumber mPrimaryChannel = channel::CCH;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    LocalDynamicMap* mLocalDynamicMap = nullptr;

    omnetpp::SimTime mLastCpmTimestamp;
    omnetpp::SimTime mLastSensorInformationTimestamp;

    omnetpp::SimTime mGenCpm;

    /**** Generation Frequency and Content Management ****/
    int mMaxPerceivedObjects; // MaxPerceptionRegions is a configuration parameter

    ObjectIdHandler mObjectIdHandler;

    /**** Configuration parameter values ****/

    /* Configuration parameter values recommended for the CPS data inclusion management */
    omnetpp::SimTime mGenCpmMin;
    omnetpp::SimTime mGenCpmMax;
    omnetpp::SimTime mAddSensorInformation;

    int mMaxPerceptionRegions;
    int mObjectInclusionConfig; /* TODO: check if is correct type. */
    
    double mObjectPerceptionQualityThreshold;
    vanetza::units::Length mMinPositionChangeThreshold;
    vanetza::units::Velocity mMinGroundSpeedChangeThreshold;
    vanetza::units::Angle mMinGroundVelocityOrientationChangeThreshold;

    /* Configuration parameter values recommended for the CPM assembly */
    int mMessageAssemblyConfig; /* TODO: check if is correct type. */
    vanetza::units::Length mMinPositionChangePriorityThreshold;
    vanetza::units::Length mMaxPositionChangePriorityThreshold;
    vanetza::units::Velocity mMinGroundSpeedChangePriorityThreshold;
    vanetza::units::Velocity mMaxGroundSpeedChangePriorityThreshold;
    vanetza::units::Angle mMinGroundVelocityOrientationChangePriorityThreshold;
    vanetza::units::Angle mMaxGroundVelocityOrientationChangePriorityThreshold;
    omnetpp::SimTime mMinLastInclusionTimePriorityThreshold;
    omnetpp::SimTime mMaxLastInclusionTimePriorityThreshold;
    
    /* Configuration parameter values recommended for sensor and object ID management */
    omnetpp::SimTime mUnusedSensorIdRetentionPeriod;
    omnetpp::SimTime mUnusedObjectIdRetentionPeriod;

    /* Configuration parameter values recommended for the object perception quality assessment */
    double mAlpha; // Moving average factor
    double mOmega_d; // Weight factor for detection success rating
    double mOmega_c; // Weight factor for detection confidence rating
    double mOmega_oa; // Weight factor for object age rating

    /* Configuration parameter values recommended for the object inclusion rate control */
    double mC_InclusionRateControl; // Confidence level
    double mW_InclusionrateControl; // Time window
    int mN_InclusionRateControl; // number of times an object may be omitted due to low VoI before it is included into a CPM again
};

vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider&, uint64_t);
void addOriginatingVehicleContainer(vanetza::asn1::Cpm&, const VehicleDataProvider&);
void addOriginatingRsuContainer(vanetza::asn1::Cpm&);
void addSensorInformationContainer(vanetza::asn1::Cpm&, const LocalDynamicMap&);
void addPerceptionRegionContainer(vanetza::asn1::Cpm&);
void addPerceivedObjectContainer(vanetza::asn1::Cpm&, const VehicleDataProvider&, const LocalDynamicMap&, const std::vector<uint32_t>::const_iterator&, const std::vector<uint32_t>::const_iterator&, ObjectIdHandler&);

} // namespace artery

#endif // ARTERY_CPSERVICE_H_
