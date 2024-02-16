#include "artery/application/CpService.h"
#include "artery/application/CpObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/Timer.h"
#include "artery/application/LocalDynamicMap.h"
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <limits>
#include <cassert>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

/************************ Helper Functions ***************************/
template<typename T, typename U>
static long roundToUnit(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

static ObjectClass_t StationType2ObjectClass(const vanetza::geonet::StationType& type)
{
    using StationType = vanetza::geonet::StationType;
    ObjectClass_t objectClass;

    if(type == StationType::Pedestrian) {
        // NOTE: add pedestrian as a VRU instead of TrafficParticipant
        objectClass.present = ObjectClass_PR_vruSubClass;
        VruProfileAndSubprofile_t& vruSubClass = objectClass.choice.vruSubClass;
        vruSubClass.present = VruProfileAndSubprofile_PR_pedestrian;
        vruSubClass.choice.pedestrian = VruSubProfilePedestrian_unavailable;
    
    } else if(type == StationType::Cyclist) {
        // NOTE: add pedestrian as a VRU instead of TrafficParticipant
        objectClass.present = ObjectClass_PR_vruSubClass;
        VruProfileAndSubprofile_t& vruSubClass = objectClass.choice.vruSubClass;
        vruSubClass.present = VruProfileAndSubprofile_PR_bicyclistAndLightVruVehicle;
        vruSubClass.choice.bicyclistAndLightVruVehicle = VruSubProfileBicyclist_unavailable;

    } else if(type == StationType::Moped) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_moped;
    
    } else if(type == StationType::Motorcycle) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_motorcycle;
    
    } else if(type == StationType::Passenger_Car) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_passengerCar;
    
    } else if(type == StationType::Bus) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_bus;
    
    } else if(type == StationType::Light_Truck) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_lightTruck;
    
    } else if(type == StationType::Heavy_Truck) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_heavyTruck;
    
    } else if(type == StationType::Trailer) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_trailer;
    
    } else if(type == StationType::Special_Vehicle) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_specialVehicle;
    
    } else if(type == StationType::Tram) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_tram;
    
    } else if(type == StationType::RSU) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.otherSubClass = TrafficParticipantType_roadSideUnit;
    
    } else if(type == StationType::Unknown) {
        objectClass.present = ObjectClass_PR_otherSubClass;
        objectClass.choice.otherSubClass = OtherSubClass_unknown;
    } else {
        std::cout << "Object class is not defined for station type " << std::endl;
        throw std::runtime_error("Object class is not defined for station type: " + std::to_string((int)type));
    }

    return objectClass;
}

ObjectIdHandler::ObjectIdHandler()
{
    for(int i = 0; i < 65536; i++) {
        mMap[i] = 0; // TODO: too much computation
        mTimestamps[i] = 0;
    }
}

uint16_t ObjectIdHandler::getOrAssign(int64_t oId, int64_t T_now)
{
    // check if current object is already assigned to an id
    auto it = std::find_if(mMap.begin(), mMap.end(), [oId](const std::pair<int16_t, int64_t>& pair) {
        return pair.second == oId;
    });

    if(it != mMap.end() && (T_now - mTimestamps[it->first]) < mRetentionPeriod) {
        mTimestamps[it->first] = T_now;
        return it->first;
    }

    // if not, assign a new id to the object
    for(int i = 0; i < 65536; i++) {
        if(mTimestamps[i] == 0 || (T_now - mTimestamps[i]) >= mRetentionPeriod) {
            mMap[i] = oId;
            mTimestamps[i] = T_now;
            return i;
        }
    }

    // TODO: handle exception in case a new id cannot be assigned to an object
}

void ObjectIdHandler::setRetentionPeriod(int retentionPeriod)
{
    mRetentionPeriod = retentionPeriod;
}

Define_Module(CpService);

CpService::CpService() :
    mGenCpm { 100, SIMTIME_MS },
    mMaxPerceivedObjects(0)
{
}

void CpService::initialize()
{
    ItsG5Service::initialize();

    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mLocalDynamicMap = &getFacilities().get_mutable<LocalDynamicMap>();

    // avoid unreasonable high elapsed time values for newly inserted vehicles
    mLastCpmTimestamp = simTime();
    mLastSensorInformationTimestamp = mLastCpmTimestamp;

    // generation rate boundaries
    mGenCpmMin = par("T_GenCpmMin");
    mGenCpmMax = par("T_GenCpmMax");
    mAddSensorInformation = par("T_AddSensorInformation");

    // Parameters for data inclusion management
    mMaxPerceptionRegions = par("maxPerceptionRegions");
    mObjectInclusionConfig = par("objectInclusionConfig");

    mObjectPerceptionQualityThreshold = par("objectPerceptionQualityThreshold");
    mMinPositionChangeThreshold = par("minPositionChangeThreshold").doubleValue() * vanetza::units::si::meter;
    mMinGroundSpeedChangeThreshold = par("minGroundSpeedChangeThreshold").doubleValue() * vanetza::units::si::meter_per_second;
    mMinGroundVelocityOrientationChangePriorityThreshold = vanetza::units::Angle { par("minGroundVelocityOrientationChangePriorityThreshold").doubleValue() * vanetza::units::degree };

    // Parameters for message assembly
    mMessageAssemblyConfig = par("messageAssemblyConfig");

    mMinPositionChangePriorityThreshold = par("minPositionChangePriorityThreshold").doubleValue() * vanetza::units::si::meter;
    mMaxPositionChangePriorityThreshold = par("maxPositionChangePriorityThreshold").doubleValue() * vanetza::units::si::meter;
    mMinGroundSpeedChangePriorityThreshold = par("minGroundSpeedChangePriorityThreshold").doubleValue() * vanetza::units::si::meter_per_second;
    mMaxGroundSpeedChangePriorityThreshold = par("maxGroundSpeedChangePriorityThreshold").doubleValue() * vanetza::units::si::meter_per_second;
    mMinGroundVelocityOrientationChangePriorityThreshold = vanetza::units::Angle { par("minGroundVelocityOrientationChangePriorityThreshold").doubleValue() * vanetza::units::degree };
    mMaxGroundVelocityOrientationChangePriorityThreshold = vanetza::units::Angle { par("maxGroundVelocityOrientationChangePriorityThreshold").doubleValue() * vanetza::units::degree };
    mMinLastInclusionTimePriorityThreshold = par("minLastInclusionTimePriorityThreshold");
    mMaxLastInclusionTimePriorityThreshold = par("maxLastInclusionTimePriorityThreshold");

    // Configuration parameter values recommended for sensor and object ID management
    mUnusedSensorIdRetentionPeriod = par("unusedSensorIdRetentionPeriod");
    mUnusedObjectIdRetentionPeriod = par("unusedObjectIdRetentionPeriod");

    // Configuration parameter values recommended for the object perception quality assessment
    mAlpha = par("alpha");
    mOmega_d = par("omega_d");
    mOmega_c = par("omega_c");
    mOmega_oa = par("omega_oa");

    // Configuration parameter values recommended for the object inclusion rate control
    mC_InclusionRateControl = par("C_InclusionRateControl");
    mW_InclusionrateControl = par("W_InclusionRateControl");
    mN_InclusionRateControl = par("N_InclusionRateControl");

    std::cout << "Unused object id retetion period: " << mUnusedObjectIdRetentionPeriod.inUnit(SIMTIME_MS) << std::endl;
    mObjectIdHandler.setRetentionPeriod(mUnusedObjectIdRetentionPeriod.inUnit(SIMTIME_MS));
}

/*
* According to TS 103 324 V2.1.1
* CPM reception management:
*   - Triggering the decoding of the CPM upon receiving an incoming CPM.
*   - Provisioning of the decoded CPM to the LDM and/or ITS applications of the receiving ITS-S.
*   - Optionally, checking the validity of the information of received CPMs.
*/
void CpService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet, const NetworkInterface& interface)
{
    Enter_Method("indicate");

    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
    if(cpm && cpm->validate()) {
        CpObject obj = visitor.shared_wrapper;        
        emit(scSignalCpmReceived, &obj);
        mLocalDynamicMap->updatePerception(obj);
        EV_DETAIL << "Received CPM!" << std::endl;
        // asn_fprint(nullptr, &asn_DEF_CpmPayload, &(*cpm)->payload); 
        asn_fprint(nullptr, &asn_DEF_ConstraintWrappedCpmContainers, &(*cpm)->payload.cpmContainers); 
    }
}

/*
* According to TS 103 324 V2.1.1
* CPM transmission management:
*   - Activation and termination of CPM transmission operation.
*   - Determination of the CPM generation event frequency.
*   - Triggering the generation of the CPM.
*/
void CpService::trigger()
{
    Enter_Method("trigger");

    SimTime T_now = simTime();

    // This is based on the last sent CPM, meaning that there is a line of thought where
    // this could only be called once, only after a CPM is sent.
    // However, because this is dependent on resources_limit defined by MCO_FAC, as stated in the standard,
    // and i currently do not know if that value is constant between CPM generations (probably not), i
    // decided to call it on every trigger.
    updateFrequencyAndContentManagement(T_now);

    // This must be updated on every trigger, because the standard states that the object perception
    // quality must first be computed at t=0, when the object is first perceived.
    // The only way to catch that is to call this method on every trigger.
    updateObjectPerceptionQuality(T_now);

    checkTriggeringConditions(T_now);
}

/*
* According to TS 103 324 V2.1.1
* T_GenCpm, the number of perceived objects and the number of perception regions to be included in the
* CPM are selected based on the VoI of the perceived objects and regions in the last generated CPM. (...)
*/
void CpService::updateFrequencyAndContentManagement(const SimTime& T_now)
{
    // This method will look into the value of information of the perceived objects (and regions)
    // of the last sent CPM. Based on that and the resources_limit value, it will update the
    // mGenCpm and mMaxPerceivedObjects values, in order to maximize the VoI rate.

    // TODO: check how to get the resources_limit value
    // TODO: update mGenCpm and mMaxPerceivedObjects

    // For now, both values are static
    mGenCpm = SimTime{500, SIMTIME_MS};
    mMaxPerceivedObjects = 255;
}

/*
* According to TS 103 324 V2.1.1
* A scalar-value indication about the overall information quality on a perceived object may be provided
* in the component objectPerceptionQuality.
*/
void CpService::updateObjectPerceptionQuality(const SimTime& T_now)
{
    for(auto& objEntry : mLocalDynamicMap->allObjectEntries()) {
        uint32_t objId = objEntry.first;
        std::shared_ptr<const ObjectData> objData = objEntry.second.objectPtr();

        SimTime T_first = objData->firstDetection;
        SimTime T_last = objData->lastDetection;
        SimTime T_age = T_now - T_first;
        SimTime T_elapsed = T_now - T_last;

        /*
        * According to TS 103 324 V2.1.1
        * The object characteristics contributing to the perception quality of an object first 
        * detected at time t=0, where t represents discrete time instants, are defined based on:
        *  - Object age oa in milliseconds.
        *  - Sensor or system specific detection confidence at time t: c_t (between 0 for no detection
        *    confidence and 1 for high detection confidence).
        *  - Detection success, describing the assessment whether a given measurement has successfully
        *    perceived the object (binary assessment) at time t: dt (0 if not detected or 1 if detected)
        */

        // the perception quality will not be affected for oa values greater than 1500 (ms).
        int oa = std::min(T_age.inUnit(SIMTIME_S), (int64_t)std::numeric_limits<int>::max());
        int c = 1; // TODO: compute a more useful value
        int d = T_elapsed.inUnit(SIMTIME_S) < 200 ? 1 : 0; // TODO: compute a more useful value. for now, if object is currently in LEM, d = 1, else d = 0
        
        int perceptionQuality = calcObjectPerceptionQuality(objId, oa, c, d);
        mObjectPerceptionQuality[objId] = perceptionQuality;
    }
}

/*
* According to TS 103 324 V2.1.1
* CPMs are generated during periodic CPM generation events.
* The time elapsed between the triggering of consecutive CPM generation events shall be equal to T_GenCpm.
*/
void CpService::checkTriggeringConditions(const SimTime& T_now)
{
    SimTime& T_GenCpm = mGenCpm;
    const SimTime T_elapsed = T_now - mLastCpmTimestamp;

    if(T_elapsed >= T_GenCpm) {
        sendCpm(T_now);
    }
}

int CpService::calcObjectPerceptionQuality(uint32_t objId, int oa, int c, int d)
{
    if(m_cEMA.find(objId) == m_cEMA.end()) { // t = 0
        m_cEMA[objId] = c;
        m_dEMA[objId] = d;
    } else {
        m_cEMA[objId] = mAlpha * c + (1 - mAlpha) * m_cEMA[objId];
        m_dEMA[objId] = mAlpha * d + (1 - mAlpha) * m_dEMA[objId];
    }

    double r_c = floor(m_cEMA[objId] * 15);
    double r_d = floor(m_dEMA[objId] * 15);

    double r_oa = std::min(floor(oa/100.0), 15.0);

    double w_d = mOmega_d;
    double w_c = mOmega_c;
    double w_oa = mOmega_oa;

    double objectPerceptionQuality = floor((w_d * r_d + w_c * r_c + w_oa * r_oa) / (w_d + w_c + w_oa));

    return (int)objectPerceptionQuality;
}

void CpService::sendCpm(const SimTime& T_now)
{
    const MultiChannelPolicy& mco = getFacilities().get_const<MultiChannelPolicy>();
    const NetworkInterfaceTable& networks = getFacilities().get_const<NetworkInterfaceTable>();

    std::vector<vanetza::asn1::Cpm> messages;

    uint64_t referenceTime = countTaiMilliseconds(mTimer->getTimeFor(T_now));

    /*
    * According to TS 103 324 V2.1.1
    * The Perceived Object Container shall include the component numberOfPerceivedObjects, describing
    * the total number of perceived objects contained in the LDM at the time of generating the CPM
    */
    int numberOfPerceivedObjects = mLocalDynamicMap->allObjectEntries().size(); 

    vanetza::asn1::Cpm cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, referenceTime);

    if(mVehicleDataProvider->getStationType() == vanetza::geonet::StationType::RSU) {
        addOriginatingRsuContainer(cpm);
    } else {
        addOriginatingVehicleContainer(cpm, *mVehicleDataProvider);
    }

    if(mMessageAssemblyConfig == 0) {
        /*
        * If the configuration parameter MessageAssemblyConfig is set to "0", the CPM assembly shall be performed
        * based on an object utility function following clause 6.1.3.2.
        * 
        * If the SensorInformationContainer or the PerceptionRegions need to be transmitted, they shall be added
        * to the first CPM generated for transmission on the preferred CPM channel
        */

        if(checkSensorInformationInclusion(T_now)) {
            addSensorInformationContainer(cpm, *mLocalDynamicMap);
            mLastSensorInformationTimestamp = T_now;
        }

        if(checkPerceptionRegionInclusion(T_now)) {
            // TODO: check if all the perception regions fit into the CPM and if not, remove information from 
            // TODO: the regions with the lowest VoI until they fit.
            addPerceptionRegionContainer(cpm);
        }

        if(checkPerceivedObjectInclusion(T_now)) {
            selectObjectsForTransmission(T_now);
            addPerceivedObjectContainer(cpm, *mVehicleDataProvider, *mLocalDynamicMap, mSelectedObjectsStart, mSelectedObjectsEnd, mObjectIdHandler);
            mSelectedObjects.erase(mSelectedObjectsStart, mSelectedObjectsEnd);
        }
    } else if (mMessageAssemblyConfig == 1) {
        /*
        * If the configuration parameter MessageAssemblyConfig is set to "1", the CPM assembly shall be performed
        * based on an object utility function following clause 6.1.3.3
        */

        // Perception regions are not available with Artery Sensor
        // so this implementation will end up equal to the previous one (with MessageAssemblyConfig = 0)
        // for now, this remains unimplemented
        EV_ERROR << "MessageAssemblyConfig = 1 is not implemented!\n";
    } else {
        EV_ERROR << "Invalid MessageAssemblyConfig value!\n";
    }

    messages.push_back(std::move(cpm));

    mLastCpmTimestamp = T_now;

    for(auto channel : mco.allChannels(vanetza::aid::CP)) {
        if(messages.empty()) {
            break;
        }

        auto network = networks.select(channel);
        if(network) {
            cpm = messages.front();
            messages.erase(messages.begin());

            using namespace vanetza;

            btp::DataRequestB request;
            request.destination_port = host_cast(getPortNumber(channel));
            request.gn.its_aid = aid::CP;
            request.gn.transport_type = geonet::TransportType::SHB;
            request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
            request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
            request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

            asn_fprint(nullptr, &asn_DEF_ConstraintWrappedCpmContainers, &cpm->payload.cpmContainers); 

            CpObject obj(std::move(cpm));
            emit(scSignalCpmSent, &obj);

            using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
            std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
            std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
            payload->layer(OsiLayer::Application) = std::move(buffer);
            this->request(request, std::move(payload));
        } else {
			EV_ERROR << "No network interface available for channel " << channel << "\n";
        }
    }

    if(!messages.empty()) {
        EV_ERROR << "Could not send all CPMs!\n";
    }
}

/************************* Check Inclusions **************************/
bool CpService::checkSensorInformationInclusion(const omnetpp::SimTime& T_now)
{
    return (T_now - mLastSensorInformationTimestamp) >= mAddSensorInformation;
}

bool CpService::checkPerceptionRegionInclusion(const omnetpp::SimTime& T_now)
{
    // The currently available sensors on artery do not provide dynamic perception regions
    // to check differences on shape, confidence and shadowing.
    // This container is never added to the CPM.
    return false;
}

bool CpService::checkPerceivedObjectInclusion(const omnetpp::SimTime& T_now)
{
    /*
    * According to TS 103 324 V2.1.1
    * If ObjectInclusionConfig is set to "0", no rules are defined in the present document.
    * The sender decides based on its own rules about the inclusion of perceived objects.
    */
    if (mObjectInclusionConfig == 0) {

    }

    /*
    * According to TS 103 324 V2.1.1
    * If ObjectInclusionConfig is set to "1", the inclusion rules defined in the present document shall apply.
    */
    if (mObjectInclusionConfig != 1) {
        EV_ERROR << "Invalid ObjectInclusionConfig value!\n";
    }

    mSelectedObjects.clear();

    /*
    * According to TS 103 324 V2.1.1
    * - Type-A objects are objects of class vruSubclass with a VRU profile pedestrian,
    *   bicyclistAndLightVruVehicle or animal or class groupSubclass or otherSubclass.
    * - Type-B objects are objects of any class not included in Type-A.
    */
    std::vector<uint32_t> typeA;
    bool includeAllTypeA = false;

    for(const auto& objEntry : mLocalDynamicMap->allObjectEntries()) {
        uint32_t objId = objEntry.first;
        std::shared_ptr<const ObjectData> objPtr = objEntry.second.objectPtr();

        int perceptionQuality = mObjectPerceptionQuality[objId];
        if (perceptionQuality <= mObjectPerceptionQualityThreshold) {
            continue;
        }

        const SimTime& firstDetection = objPtr->firstDetection;
        const SimTime& lastDectection = objPtr->lastDetection;

        // The object has first been detected by the perception system after the last CPM generation event.
        if (firstDetection > mLastCpmTimestamp) {
            std::cout << "Object " << objId << " selected for inclusion in CPM\n";
            std::cout << "Fist detection: " << firstDetection << std::endl;
            std::cout << "Last CPM timestamp: " << mLastCpmTimestamp << std::endl;

            mSelectedObjects.push_back(objId);
            continue;
        }

        ObjectClass_t objClass = StationType2ObjectClass(objPtr->stationType);
    
        // Type-A objects are objects of class vruSubclass with a VRU profile pedestrian,
        // bicyclistAndLightVruVehicle or animal or class groupSubclass or otherSubclass
        bool isTypeA =
                    (objClass.present == ObjectClass_PR_vruSubClass
                        && (objClass.choice.vruSubClass.present == VruProfileAndSubprofile_PR_pedestrian
                            || objClass.choice.vruSubClass.present == VruProfileAndSubprofile_PR_bicyclistAndLightVruVehicle
                            || objClass.choice.vruSubClass.present == VruProfileAndSubprofile_PR_animal
                           )
                    )
                    || objClass.present == ObjectClass_PR_groupSubClass
                    || objClass.present == ObjectClass_PR_otherSubClass;
    
        if(isTypeA) {
            typeA.push_back(objId);
            
            // an "old" object might not be present on the Maps because it was omitted due to its VoI
            // so this approach selects those objects to be included in the CPM
            if(mLastInclusionTimes.find(objId) == mLastInclusionTimes.end() || (T_now - mLastInclusionTimes[objId]) >= mGenCpmMax/2) {
                includeAllTypeA = true;
            }
            
            continue;
        }

        const Position& position = objPtr->position;
        const vanetza::units::Velocity& speed = objPtr->speed;
        const vanetza::units::Angle& orientation = objPtr->orientation;

        bool distanceExceeded = distance(position, mLastPositions[objId]) > mMinPositionChangeThreshold;
        bool speedExceeded = abs(speed - mLastSpeeds[objId]) > mMinGroundSpeedChangeThreshold;
        bool orientationExceeded = ! vanetza::facilities::similar_heading(orientation, mLastOrientations[objId], mMinGroundVelocityOrientationChangeThreshold);
        bool timeExceeded = (mLastInclusionTimes.find(objId) == mLastInclusionTimes.end()) || (T_now - mLastInclusionTimes[objId]) >= mGenCpmMax;

        if (distanceExceeded || speedExceeded || orientationExceeded || timeExceeded) {
            std::cout << "Distance was exceeded: " << distanceExceeded << std::endl;
            std::cout << "Distance: " << distance(position, mLastPositions[objId]).value() << std::endl;
            std::cout << "Min distance: " << mMinPositionChangeThreshold.value() << std::endl;
            std::cout << "Speed was exceeded: " << speedExceeded << std::endl;
            std::cout << "Speed: " << abs(speed - mLastSpeeds[objId]).value() << std::endl;
            std::cout << "Min speed: " << mMinGroundSpeedChangeThreshold.value() << std::endl;
            std::cout << "Orientation was exceeded: " << orientationExceeded << std::endl;
            std::cout << "Orientation: " << orientation.value() << std::endl;
            std::cout << "Last orientation: " << mLastOrientations[objId].value() << std::endl;
            std::cout << "Min orientation: " << mMinGroundVelocityOrientationChangeThreshold.value() << std::endl;
            std::cout << "Time was exceeded: " << timeExceeded << std::endl;
            std::cout << "Time: " << (T_now - mLastInclusionTimes[objId]).inUnit(SIMTIME_MS) << std::endl;
            std::cout << "Last inclusion time: " << mLastInclusionTimes[objId].inUnit(SIMTIME_MS) << std::endl;
            std::cout << "Min inclusion time: " << mGenCpmMax.inUnit(SIMTIME_MS) << std::endl;
            std::cout << "Object " << objId << " selected for inclusion in CPM\n";
            mSelectedObjects.push_back(objId);
        }
    }

    if(includeAllTypeA) {
        mSelectedObjects.insert(mSelectedObjects.end(), typeA.begin(), typeA.end());
    }

    // The standard defines that objects not selected for transmission may be predicted to the next
    // generation event, for example assuming a constant velocity model. Following this prediction
    // all objects that would then need to be included in a CPM in the next generation event may also
    // be selected for inclusion in the currently generated CPM by including the latest available kinematic
    // and attitude state.
    // TODO: this should be implemented in the future

    return !mSelectedObjects.empty();
}

/***** Object Selection *****/

void CpService::selectObjectsForTransmission(const omnetpp::SimTime& T_now)
{
    /*
    * According to TS 103 324 V2.1.1
    * VoI is used to rank the objects and select the alternative channel where each of them can be included in a CPM.
    */ 
    std::sort(mSelectedObjects.begin(), mSelectedObjects.end(), [this](uint32_t a, uint32_t b) {
        return calcValueOfInformation(a) > calcValueOfInformation(b);
    });

    if (mSelectedObjects.size() > mMaxPerceivedObjects) {
        mSelectedObjectsStart = mSelectedObjects.begin();
        mSelectedObjectsEnd = mSelectedObjects.begin() + mMaxPerceivedObjects;
    } else {
        mSelectedObjectsStart = mSelectedObjects.begin();
        mSelectedObjectsEnd = mSelectedObjects.end();
    }

    /*
    * According to TS 103 324 V2.1.1
    * Perceived objects selected for transmission shall be included in CPMs in a descending priority order following a
    * per-object utility function defined as the sum of the following parameters:
    *  - Object perception quality
    *  - Object position change
    *  - Object speed change
    *  - Object orientation change
    *  - Object last inclusion time
    */
    std::sort(mSelectedObjectsStart, mSelectedObjectsEnd, [T_now,this](uint32_t a, uint32_t b) {
        return calcObjectUtilityFunction(T_now, a) > calcObjectUtilityFunction(T_now, b);
    });
}

int CpService::calcValueOfInformation(uint32_t objId)
{
    return 1;
}

double CpService::calcObjectUtilityFunction(const omnetpp::SimTime& T_now, uint32_t objId)
{
    std::shared_ptr<const ObjectData> objPtr = mLocalDynamicMap->getObjectData(objId);

    // object perception quality
    double p_qual = mObjectPerceptionQuality[objId] / 15.0;

    // object position change
    vanetza::units::Length delta_p = distance(objPtr->position, mLastPositions[objId]);
    double p_pos = 0;
    if (delta_p > mMinPositionChangePriorityThreshold) {
        p_pos = (delta_p - mMinPositionChangePriorityThreshold) / (mMaxPositionChangePriorityThreshold - mMinPositionChangePriorityThreshold); // implicit cast to double
        p_pos = std::min(1.0, p_pos);
    }

    // object speed change
    vanetza::units::Velocity delta_v = abs(objPtr->speed - mLastSpeeds[objId]);
    double p_speed = 0;
    if (delta_v > mMinGroundSpeedChangePriorityThreshold) {
        p_speed = (delta_v - mMinGroundSpeedChangePriorityThreshold) / (mMaxGroundSpeedChangePriorityThreshold - mMinGroundSpeedChangePriorityThreshold); // implicit cast to double
        p_speed = std::min(1.0, p_speed);
    }

    // object orientation change
    vanetza::units::Angle delta_o = abs(objPtr->orientation - mLastOrientations[objId]);
    if(delta_o > vanetza::units::Angle { 180 * vanetza::units::degree }) {
        delta_o = vanetza::units::Angle { 360 * vanetza::units::degree } - delta_o;
    }
    double p_orientation = 0;
    if (delta_o > mMinGroundVelocityOrientationChangePriorityThreshold) {
        p_orientation = (delta_o - mMinGroundVelocityOrientationChangePriorityThreshold) / (mMaxGroundVelocityOrientationChangePriorityThreshold - mMinGroundVelocityOrientationChangePriorityThreshold); // implicit cast to double
        p_orientation = std::min(1.0, p_orientation);
    }

    // object last inclusion time
    SimTime delta_t = T_now - mLastInclusionTimes[objId];
    double p_time = 0;
    if (delta_t > mMinLastInclusionTimePriorityThreshold) {
        p_time = (delta_t - mMinLastInclusionTimePriorityThreshold) / (mMaxLastInclusionTimePriorityThreshold - mMinLastInclusionTimePriorityThreshold); // implicit cast to double
        p_time = std::min(1.0, p_time);
    }

    return p_qual + p_pos + p_speed + p_orientation + p_time;
}




/********************* Message Creation Functions ********************/
vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider& vdp, uint64_t referenceTime)
{
    vanetza::asn1::Cpm message;

    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2; // standard-defined value
    header.messageId = MessageId_cpm; // standard-defined value
    header.stationId = vdp.getStationId(); // unsigned 32-bit value

    CpmPayload_t& payload = (*message).payload;

    CPM_PDU_Descriptions_ManagementContainer_t& mc = payload.managementContainer;
    
    int ret = asn_uint642INTEGER(&mc.referenceTime, referenceTime);
    assert(ret == 0);
    
    // NOTE: The new CPM format uses a deprecated DE from CDD, according to CDD docs (ReferencePosition instead of ReferencePositionWithConfidence)
    //       However, it does make sense, because a vehicle is sure of its own position.

    // latitude and longitude units -> 10^-7 degrees
    mc.referencePosition.latitude = roundToUnit(vdp.latitude(), vanetza::units::degree * boost::units::si::micro) * 10; // extended to unit
    mc.referencePosition.longitude = roundToUnit(vdp.longitude(), vanetza::units::degree * boost::units::si::micro) * 10; // extended to unit

    mc.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    mc.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    mc.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

    mc.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    mc.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    // TODO: optional values

    return message;
}

void addOriginatingVehicleContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp)
{
    CpmPayload& payload = (*message).payload;

    ConstraintWrappedCpmContainers_t& cwccs = payload.cpmContainers;
    // WrappedCpmContainers_t& wccs = cwccs;
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();

    wcc->containerId = CpmContainerId_originatingVehicleContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer;
    OriginatingVehicleContainer_t& ovc = wcc->containerData.choice.OriginatingVehicleContainer;

    // wgs84 unit -> 0.1 degrees
    // TODO: replace heading with orientation angle. see standard
    ovc.orientationAngle.value = roundToUnit(vdp.heading(), vanetza::units::degree * boost::units::si::deci); // between 0..3590
    ovc.orientationAngle.confidence = 10; // equal or within 1 degree (10 decidegrees)
    
    // TODO: Optional values

    int ret = ASN_SEQUENCE_ADD(&(cwccs.list), wcc);
    assert(ret == 0);
}

void addOriginatingRsuContainer(vanetza::asn1::Cpm& message)
{
    CpmPayload& payload = (*message).payload;
    
    ConstraintWrappedCpmContainers_t& cwccs = payload.cpmContainers;
    // WrappedCpmContainers_t& wccs = cwccs;
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    
    wcc->containerId = CpmContainerId_originatingRsuContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_OriginatingRsuContainer;
    OriginatingRsuContainer_t& orc = wcc->containerData.choice.OriginatingRsuContainer;

    // TODO: add optional mapReference value

    int ret = ASN_SEQUENCE_ADD(&(cwccs.list), wcc);
    assert(ret == 0);
}

void addSensorInformationContainer(vanetza::asn1::Cpm& message, const LocalDynamicMap& ldm)
{
    CpmPayload& payload = (*message).payload;
    
    ConstraintWrappedCpmContainers_t& cwccs = payload.cpmContainers;
    // WrappedCpmContainers_t& wccs = cwccs;
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    
    wcc->containerId = CpmContainerId_sensorInformationContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_SensorInformationContainer;
    SensorInformationContainer_t& sic = wcc->containerData.choice.SensorInformationContainer;

    /*
    * According to TS 103 324 V2.1.1
    * SensorInformation shall include the component sensorId. This identifier is in turn utilized in the
    * PerceivedObjectContainer to connect information about an object to the sensor that provided that information. After a
    * pseudonym change the current sensorId of all SensorInformation may be replaced by a new sensorId randomly selected
    * among identifiers which were not used for UnusedSensorIdRetentionPeriod
    */
    // TODO: for now, id is iterated
    uint32_t id = 0;

    for(auto& sensorEntry : ldm.allSensorEntries()) {
        std::shared_ptr<const SensorData> sensorData = sensorEntry.second.objectPtr();

        SensorInformation_t* si = vanetza::asn1::allocate<SensorInformation_t>();
        si->sensorId = id++; // TODO: replace with standard method
        si->sensorType = SensorType_radar; // TODO: replace with standard method
        si->perceptionRegionShape = vanetza::asn1::allocate<Shape_t>();
        si->perceptionRegionShape->present = Shape_PR_radial;

        RadialShape_t& rs = si->perceptionRegionShape->choice.radial;
        rs.range = sensorData->range;
        // TODO: add function to retrieve start and end angles of sensor cone
        // cannot be implemented with vanilla sensors.
        rs.horizontalOpeningAngleStart = CartesianAngleValue_unavailable;
        rs.horizontalOpeningAngleEnd = CartesianAngleValue_unavailable;

        // TODO: Optional values
    
        // TODO: check if this is correctly assigned
        si->perceptionRegionConfidence = vanetza::asn1::allocate<ConfidenceLevel_t>();
        *(si->perceptionRegionConfidence) = 100; // unit -> percentage

        si->shadowingApplies = sensorData->shadowingApplies;

        int ret = ASN_SEQUENCE_ADD(&(sic.list), si);
        assert(ret == 0);
    }

    int ret = ASN_SEQUENCE_ADD(&(cwccs.list), wcc);
    assert(ret == 0);
}

void addPerceptionRegionContainer(vanetza::asn1::Cpm& message)
{
    // The currently available sensors on artery do not provide dynamic perception regions
    // to check differences on shape, confidence and shadowing.
    // This container is never added to the CPM.
}

void addPerceivedObjectContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp, const LocalDynamicMap& ldm, const std::vector<uint32_t>::const_iterator& selectedObjectsStart, const std::vector<uint32_t>::const_iterator& selectedObjectsEnd, ObjectIdHandler& objectIdHandler)
{
    CpmPayload& payload = (*message).payload;

    ConstraintWrappedCpmContainers_t& cwccs = (*message).payload.cpmContainers;
    WrappedCpmContainers_t& wccs = cwccs;
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();

    wcc->containerId = CpmContainerId_perceivedObjectContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
    PerceivedObjectContainer_t& poc = wcc->containerData.choice.PerceivedObjectContainer;

    unsigned totalObjects = ldm.totalObjects(); // unsigned 8-bit value
    assert(totalObjects <= 255); // check if when number is greater than 255, it should clamp or throw error
    poc.numberOfPerceivedObjects = totalObjects; // implicit cast

    PerceivedObjects_t& pos = poc.perceivedObjects;

    // get packet reference time
    const CPM_PDU_Descriptions_ManagementContainer& mc = payload.managementContainer;
    const INTEGER_t& timestampIts = mc.referenceTime;
    uint64_t referenceTime;
    int ret = asn_INTEGER2ulong(&timestampIts, &referenceTime);
    assert(ret == 0);

    const Position& egoPos = vdp.position(); // position in simulation

    auto it = selectedObjectsStart;
    for(; it != selectedObjectsEnd; ++it)  {
        uint32_t objId = *it;
        std::shared_ptr<const ObjectData> objPtr = ldm.getObjectData(objId);

        PerceivedObject_t* po = vanetza::asn1::allocate<PerceivedObject_t>();
        po->objectId = vanetza::asn1::allocate<Identifier2B_t>();
        
        *(po->objectId) = objectIdHandler.getOrAssign(objId, referenceTime);
        
        int64_t deltaTime = objPtr->taiLastDetection- referenceTime;

        // clamp measurementDeltaTime according to cdd documentation
        if(deltaTime < -2048) deltaTime = -2048;
        if(deltaTime > 2046) deltaTime = 2047;
        po->measurementDeltaTime = deltaTime;

        const Position& objectPos = objPtr->position;
        CartesianPosition3dWithConfidence_t& position = po->position;

        position.xCoordinate.value = roundToUnit(objectPos.x - egoPos.x, vanetza::units::si::meter * boost::units::si::centi);
        if(position.xCoordinate.value < CartesianCoordinateLarge_negativeOutOfRange)
            position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
        if(position.xCoordinate.value > CartesianCoordinateLarge_positiveOutOfRange)
            position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
        
        position.xCoordinate.confidence = 1; // TODO
        if(position.xCoordinate.confidence > 4094)
            position.xCoordinate.confidence = CoordinateConfidence_outOfRange;

        position.yCoordinate.value = roundToUnit(objectPos.y - egoPos.y, vanetza::units::si::meter * boost::units::si::centi);
        if(position.yCoordinate.value < CartesianCoordinateLarge_negativeOutOfRange)
            position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
        if(position.yCoordinate.value > CartesianCoordinateLarge_positiveOutOfRange)
            position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
        
        position.yCoordinate.confidence = 1; // TODO
        if(position.yCoordinate.confidence > 4094)
            position.yCoordinate.confidence = CoordinateConfidence_outOfRange;

        ret = ASN_SEQUENCE_ADD(&(pos.list), po);
        assert(ret == 0);
    }

    ret = ASN_SEQUENCE_ADD(&(wccs.list), wcc);
    assert(ret == 0);
}

// void addPerceivedObjectContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp, const LocalDynamicMap& ldm, const std::vector<uint32_t>::const_iterator& selectedObjectsStart, const std::vector<uint32_t>::const_iterator& selectedObjectsEnd)
// {
//     CpmPayload& payload = (*message).payload;

//     ConstraintWrappedCpmContainers_t& cwccs = (*message).payload.cpmContainers;
//     // WrappedCpmContainers_t& wccs = cwccs;
//     WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();

//     wcc->containerId = CpmContainerId_perceivedObjectContainer;
//     wcc->containerData.present = WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
//     PerceivedObjectContainer_t& poc = wcc->containerData.choice.PerceivedObjectContainer;
//     poc.numberOfPerceivedObjects = ldm.allObjectEntries().size(); // unsigned 8-bit value

//     PerceivedObjects_t& pos = poc.perceivedObjects;

//     const CPM_PDU_Descriptions_ManagementContainer& mc = payload.managementContainer;
//     const INTEGER_t& timestampIts = mc.referenceTime;
//     uint64_t referenceTime;
//     int ret = asn_INTEGER2ulong(&timestampIts, &referenceTime);
//     assert(ret == 0);

//     const Position& egoPos = vdp.position();

//     auto it = selectedObjectsStart;
//     for(; it != selectedObjectsEnd; ++it) {
//         uint32_t objId = *it;
//         std::shared_ptr<const ObjectData> objPtr = ldm.getObjectData(objId);

//         PerceivedObject_t* po = vanetza::asn1::allocate<PerceivedObject_t>();
//         po->objectId = vanetza::asn1::allocate<Identifier2B_t>();
//         *(po->objectId) = 1; // TODO: implement object id map system
        
//         int64_t deltaTime = objPtr->taiLastDetection- referenceTime;

//         // clamp measurementDeltaTime according to cdd documentation
//         if(deltaTime < -2048) deltaTime = -2048;
//         if(deltaTime > 2046) deltaTime = 2047;
//         po->measurementDeltaTime = deltaTime;

//         const Position& objectPos = objPtr->position;
//         CartesianPosition3dWithConfidence_t& position = po->position;

//         position.xCoordinate.value = roundToUnit(objectPos.x - egoPos.x, vanetza::units::si::meter * boost::units::si::centi);
//         if(position.xCoordinate.value < CartesianCoordinateLarge_negativeOutOfRange)
//             position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
//         if(position.xCoordinate.value > CartesianCoordinateLarge_positiveOutOfRange)
//             position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
        
//         position.xCoordinate.confidence = 1; // TODO
//         if(position.xCoordinate.confidence > 4094)
//             position.xCoordinate.confidence = CoordinateConfidence_outOfRange;

//         position.yCoordinate.value = roundToUnit(objectPos.y - egoPos.y, vanetza::units::si::meter * boost::units::si::centi);
//         if(position.yCoordinate.value < CartesianCoordinateLarge_negativeOutOfRange)
//             position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
//         if(position.yCoordinate.value > CartesianCoordinateLarge_positiveOutOfRange)
//             position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
        
//         position.yCoordinate.confidence = 1; // TODO
//         if(position.yCoordinate.confidence > 4094)
//             position.yCoordinate.confidence = CoordinateConfidence_outOfRange;

//         ret = ASN_SEQUENCE_ADD(&(pos.list), po);
//         assert(ret == 0);
//     }

//     ret = ASN_SEQUENCE_ADD(&(cwccs.list), wcc);
//     assert(ret == 0);
// }

} // namespace artery
