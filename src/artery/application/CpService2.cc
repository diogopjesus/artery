/*
 * Artery V2X Simulation Framework
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/CpService.h"
#include "artery/application/CpObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/NetworkInterface.h"
#include "artery/utility/InitStages.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/sensor/FovSensor.h"
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <algorithm>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

Define_Module(CpService)

CpService::CpService() :
    mGenCpmMin { 100, SIMTIME_MS },
    mGenCpmMax { 1000, SIMTIME_MS }
    // TODO: Initialize the rest of the members
{
}

int CpService::numInitStages() const
{
    return InitStages::Total;
}

void CpService::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
        // TODO: Read local parameters and look up sibling modules
    } else if (stage == InitStages::Self) {
        // TODO: Initialize module itself
    } else if (stage == InitStages::Propagate) {
        // TODO: Propagate messages, events, and signals
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

    // This is based on the last sent CPM, meaning that there is a line of thought where
    // this could only be called once, only after a CPM is sent.
    // However, because this is dependent on resources_limit defined by MCO_FAC, as stated in the standard,
    // and i currently do not know if that value is constant between CPM generations (probably not), i
    // decided to call it on every trigger.
    updateFrequencyAndContentManagement();
    checkTriggeringConditions(simTime());
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
        emit(scSignalCpmReceived, &obj); // TODO: update envmod with received CPM
        // TODO: implement local dynamic map update
    }
}

/*
* According to TS 103 324 V2.1.1
* T_GenCpm, the number of perceived objects and the number of perception regions to be included in the
* CPM are selected based on the VoI of the perceived objects and regions in the last generated CPM. (...)
*/
void CpService::updateFrequencyAndContentManagement()
{
    // This method will look into the value of information of the perceived objects (and regions)
    // of the last sent CPM. Based on that and the resources_limit value, it will update the
    // mGenCpm and mMaxPerceivedObjects values, in order to maximize the VoI rate.

    // TODO: check how to get the resources_limit value
    // TODO: update mGenCpm and mMaxPerceivedObjects

    // For now, both values are static
    mGenCpm = SimTime{500, SIMTIME_MS};
    mMaxPerceivedObjects = 255; // Arbitrary value, could be other (this value does not mean on only one CPM but in total)
}

/*
* According to TS 103 324 V2.1.1
* CPMs are generated during periodic CPM generation events.
* The time elapsed between the triggering of consecutive CPM generation events shall be equal to T_GenCpm.
*/
//DONE
void CpService::checkTriggeringConditions(const SimTime& T_now)
{
    SimTime& T_GenCpm = mGenCpm;
    const SimTime T_elapsed = T_now - mLastCpmTimestamp;

    if(T_elapsed >= T_GenCpm) {
        sendCpm(T_now);
    }
}

//DONE
bool CpService::checkSensorInformationInclusion(const SimTime& T_now)
{
    return (T_now - mLastSensorInformationTimestamp) >= mAddSensorInformation;
}

//DONE
bool CpService::checkPerceptionRegionInclusion()
{
    // The currently available sensors on artery do not provide dynamic perception regions
    // to check differences on shape, confidence and shadowing.
    // This container is never added to the CPM.
    return false;
}

//DONE
void CpService::checkPerceivedObjects(const SimTime& T_now)
{
    mSelectedObjects.clear();
    mOffloadedObjects.clear();

    /*
    * According to TS 103 324 V2.1.1
    * If ObjectInclusionConfig is set to "0", no rules are defined in the present document.
    * The sender decides based on its own rules about the inclusion of perceived objects.
    */
    if (mObjectInclusionConfig == 0) {
        // NOTE: on this mock example of a custom inclusion management
        //       we will select all perceived objects.

        using TrackedObject = LocalEnvironmentModel::TrackedObject;
        for (const TrackedObject& obj : mEnvironmentModel->allObjects()) {
            const LocalEnvironmentModel::Tracking& tracking = obj.second; 
            if (tracking.expired()) {
                continue;
            }

            mSelectedObjects.push_back(obj);
        }
        
        return;
    }

    /*
    * According to TS 103 324 V2.1.1
    * If ObjectInclusionConfig is set to "1", the inclusion rules defined in the present document shall apply.
    */
    if (mObjectInclusionConfig != 1) {
        throw cRuntimeError("Invalid ObjectInclusionConfig value: %d", mObjectInclusionConfig);
    }

    /*
    * According to TS 103 324 V2.1.1
    * - Type-A objects are objects of class vruSubclass with a VRU profile pedestrian,
    *   bicyclistAndLightVruVehicle or animal or class groupSubclass or otherSubclass.
    * - Type-B objects are objects of any class not included in Type-A.
    */

    std::vector<TrackedObject> typeA;
    bool includeAllTypeA = false;


    using Tracking = LocalEnvironmentModel::Tracking;
    for(const TrackedObject& obj : mEnvironmentModel->allObjects()) {
        const std::weak_ptr<EnvironmentModelObject>& weak = obj.first;
        const Tracking& tracking = obj.second;

        if (weak.expired() ||tracking.expired()) {
            continue;
        }

        // check if perception quality is exceeds threshold
        // in the artery sensors case it is always true
        int perceptionQuality = objectPerceptionQuality(obj, T_now); // TODO: change the way this value is obtained (this should be updated every cycle and only here is only accessed)
        if (perceptionQuality < mObjectPerceptionQualityThreshold) {
            continue;
        }

        SimTime firstDetection = SimTime::getMaxTime();
        for (const auto& sensor : tracking.sensors()) {
            using TrackingTime = LocalEnvironmentModel::TrackingTime;
            const TrackingTime& trackingTime = sensor.second;

            SimTime first = trackingTime.first();
            if (first < firstDetection) {
                firstDetection = first;
            }
        }

        // The object has first been detected by the perception system after the last CPM generation event.
        if (firstDetection > mLastCpmTimestamp) {
            mSelectedObjects.push_back(obj);
            continue;
        }

        // determine the object class
        const VehicleDataProvider& vdp = weak.lock()->getVehicleData();
        ObjectClass_t objClass = StationType2ObjectClass(vdp.getStationType());
        const uint32_t objId = vdp.getStationId();
        
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
            typeA.push_back(obj);
            
            // an "old" object might not be present on the Maps because it was omitted due to its VoI
            // so this approach selects those objects to be included in the CPM
            if(mLastInclusionTimes.find(objId) == mLastInclusionTimes.end() || (T_now - mLastInclusionTimes[objId]) >= mGenCpmMax/2) {
                includeAllTypeA = true;
            }
            
            continue;
        }

        const Position& position = weak.lock()->getCentrePoint();
        
        bool distanceExceeded = distance(position, mLastPositions[objId]) > mMinPositionChangeThreshold;
        bool speedExceeded = abs(vdp.speed() - mLastSpeeds[objId]) > mMinGroundSpeedChangeThreshold;
        bool orientationExceeded = ! vanetza::facilities::similar_heading(vdp.heading(), mLastOrientations[objId], mMinGroundVelocityOrientationChangeThreshold);
        bool timeExceeded = (mLastInclusionTimes.find(objId) == mLastInclusionTimes.end()) || (T_now - mLastInclusionTimes[objId]) >= mGenCpmMax;

        if (distanceExceeded || speedExceeded || orientationExceeded || timeExceeded) {
            mSelectedObjects.push_back(obj);
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

    // If the ITS-S supports MCO, T_GenCpm and the amount of data generated per channel and generation event shall be
    // adjusted by the CPS to satisfy the limits provided by MCO_FAC. To this end, perceived objects selected for
    // transmission in the currently generated CPM that have the lowest Value of Information (VoI) should be omitted.

    // Sort perceived objects by VoI (this must be done to facilitate the removal of the objects and separation to different CPMs)
    // std::sort(mSelectedObjects.begin(), mSelectedObjects.end(), [this](const TrackedObject& a, const TrackedObject& b) {
    //     return valueOfInformation(a) < valueOfInformation(b);
    // });

    // Remove objects with the lowest VoI until the limit of the maximum number of perceived objects is reached
    auto it = mSelectedObjects.begin() + mMaxPerceivedObjects;
    if(it != mSelectedObjects.end()) {
        mOffloadedObjects.insert(mOffloadedObjects.end(), it, mSelectedObjects.end());
        mSelectedObjects.erase(it, mSelectedObjects.end());
    }
}

void CpService::sendCpm(const SimTime& T_now)
{
    int64_t referenceTime = T_now.inUnit(SIMTIME_MS); // TODO: check if last vdp updated time or t_now should be used. 

    const MultiChannelPolicy& mco = getFacilities().get_const<MultiChannelPolicy>();
    const NetworkInterfaceTable& networks = getFacilities().get_const<NetworkInterfaceTable>();

    std::vector<vanetza::asn1::Cpm> messages;

    // TODO: change the way this value is obtained
    int numberOfPerceivedObjects = mEnvironmentModel->allObjects().size();
    
    // create main CPM (first one to be sent, where SIC and PRC must be present if they are to be included)
    auto cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, referenceTime);

    // TODO: check if a vehicle or an RSU
    addOriginatingVehicleContainer(cpm, *mVehicleDataProvider);

    // If the configuration parameter MessageAssemblyConfig is set to "0", the CPM assembly shall be performed
    // based on an object utility function following clause 6.1.3.2.
    if(mMessageAssemblyConfig == 0) {
        // If the SensorInformationContainer or the PerceptionRegions need to be transmitted, they
        // shall be added to the first CPM generated for transmission on the preferred CPM channel

        if(checkSensorInformationInclusion(T_now)) {
            addSensorInformationContainer(cpm, mEnvironmentModel->getSensors());
        }

        if(checkPerceptionRegionInclusion()) {
            // TODO: check if all the perception regions fit into the CPM
            //       and if not, remove information from the regions with the lowest VoI
            //       until they fit.
            // addPerceptionRegionContainer(cpm/*, TODO: define arguments*/);
        }


        checkPerceivedObjects(T_now);
        if(mSelectedObjects.size() > 0) {
            // Perceived objects selected for transmission shall be included in CPMs in a 
            // descending priority order following a per-object utility function
            // TODO: change the way object perception quality is obtained
            using TrackedObject = LocalEnvironmentModel::TrackedObject;
            // std::sort(mSelectedObjects.begin(), mSelectedObjects.end(), [this,T_now](const TrackedObject& a, const TrackedObject& b) {
            //     return objectUtilityFunction(a, T_now) > objectUtilityFunction(b, T_now);
            // });

            addPerceivedObjectContainer(cpm, *mVehicleDataProvider, mSelectedObjects,  numberOfPerceivedObjects);
        }

        // store the main CPM
        messages.push_back(cpm);
        
        while(mOffloadedObjects.size() > 0) {
            // create new CPM
            cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, referenceTime);
            addOriginatingVehicleContainer(cpm, *mVehicleDataProvider);

            while(mSelectedObjects.size() < mMaxPerceivedObjects && mOffloadedObjects.size() > 0) {
                mSelectedObjects.push_back(mOffloadedObjects.back());
                mOffloadedObjects.pop_back();
            }

            // Perceived objects selected for transmission shall be included in CPMs in a 
            // descending priority order following a per-object utility function
            using TrackedObject = LocalEnvironmentModel::TrackedObject;
            // std::sort(mSelectedObjects.begin(), mSelectedObjects.end(), [this,T_now](const TrackedObject& a, const TrackedObject& b) {
            //     return objectUtilityFunction(a, T_now) > objectUtilityFunction(b, T_now);
            // });

            addPerceivedObjectContainer(cpm, *mVehicleDataProvider, mSelectedObjects,  numberOfPerceivedObjects);
            messages.push_back(cpm);
        }
    }

    // If the configuration parameter MessageAssemblyConfig is set to "1", the CPM assembly shall
    // be performed based on the perception regions selected for inclusion following clause 6.1.3.3.
    if (mMessageAssemblyConfig == 1) {
        // Perception regions are not available with Artery Sensor
        // so this implementation will end up equal to the previous one (with MessageAssemblyConfig = 0)
        // for now, this remains unimplemented
    }

    // TODO: send CPMs
    // It is assumed the main channel is the first channel
    // TODO: check if this is true
    for(auto channel : mco.allChannels(vanetza::aid::CP)) {
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
            request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;    /* Provide variable names according to standard */

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

        // check if all messages were sent
        if(messages.size() <= 0) {
            break;
        }
    }
}

double CpService::objectUtilityFunction(const LocalEnvironmentModel::TrackedObject& obj, const SimTime& T_now)
{
    using Length = vanetza::units::Length;
    using Velocity = vanetza::units::Velocity;
    using Angle = vanetza::units::Angle;

    uint32_t objId = obj.first.lock()->getVehicleData().getStationId();
    Position centre = obj.first.lock()->getCentrePoint();
    Velocity speed = obj.first.lock()->getVehicleData().speed();
    Angle orientation = obj.first.lock()->getVehicleData().heading();

    // Object perception quality
    double quality = objectPerceptionQuality(obj, T_now);
    double p_qual = quality / 15.0;

    // Object position change
    Length posDelta = distance(centre, mLastPositions[objId]);
    double p_pos = 0;
    if (posDelta > mMinPositionChangePriorityThreshold) {
        // TODO: repair wrong types
        p_pos = (posDelta - mMinPositionChangePriorityThreshold) / (mMaxPositionChangePriorityThreshold - mMinPositionChangePriorityThreshold);
        p_pos = std::min(1.0, p_pos);
    }

    // Object speed change
    Velocity speedDelta = abs(speed - mLastSpeeds[objId]);
    double p_speed = 0;
    if(speedDelta > mMinGroundSpeedChangePriorityThreshold) {
        // TODO: repair wrong types
        p_speed = (speedDelta - mMinGroundSpeedChangePriorityThreshold) / (mMaxGroundSpeedChangePriorityThreshold - mMinGroundSpeedChangePriorityThreshold);
        p_speed = std::min(1.0, p_speed);
    }

    // Object orientation change
    using boost::math::double_constants::pi;
    static const Angle full_circle = 2.0 * pi * boost::units::si::radian;
    Angle angleDelta = fmod(abs(orientation - mLastOrientations[objId]), full_circle);
    if(angleDelta.value() > pi) {
        angleDelta = full_circle - angleDelta;
    }

    double p_orient = 0;
    if (angleDelta > mMinGroundVelocityOrientationChangePriorityThreshold) {
        // TODO: repair wrong types
        p_orient = (angleDelta - mMinGroundVelocityOrientationChangePriorityThreshold) / (mMaxGroundVelocityOrientationChangePriorityThreshold - mMinGroundVelocityOrientationChangePriorityThreshold);
        p_orient = std::min(1.0, p_orient);
    }

    // Object last inclusion time
    SimTime timeDelta = T_now - mLastInclusionTimes[objId];
    double p_time = 0;
    if(timeDelta > mMinLastInclusionTimePriorityThreshold) {
        // TODO: repair wrong types
        SimTime num = timeDelta - mMinLastInclusionTimePriorityThreshold;
        SimTime den = mMaxLastInclusionTimePriorityThreshold - mMinLastInclusionTimePriorityThreshold;
        p_time = num.dbl() / den.dbl();
    }

    return p_qual + p_pos + p_speed + p_orient + p_time;
}

// TODO: implement this function
int CpService::valueOfInformation(const LocalEnvironmentModel::TrackedObject& obj)
{
    return 1;
}

// TODO: temporary function
//       this function should be implemented with corrected precision (Boost)
//       and should be moved to a more appropriate place, and must keep an history of the last
//       computed EMAs.
int CpService::objectPerceptionQuality(const LocalEnvironmentModel::TrackedObject& obj, const SimTime& T_now)
{
    using Tracking = LocalEnvironmentModel::Tracking;
    const Tracking& tracking = obj.second;

    // Object age in milliseconds.

    SimTime firstDetection = SimTime::getMaxTime();
    SimTime lastDetection = SimTime::ZERO;

    for (const auto& sensor : tracking.sensors()) {
        using TrackingTime = LocalEnvironmentModel::TrackingTime;
        const TrackingTime& trackingTime = sensor.second;

        SimTime first = trackingTime.first();
        if (first < firstDetection) {
            firstDetection = first;
        }

        SimTime last = trackingTime.last();
        if (last > lastDetection) {
            lastDetection = last;
        }
    }

    SimTime objAge = T_now - firstDetection;
    int64_t oa = objAge.inUnit(SIMTIME_MS);
    int64_t t = std::max(0L, oa);

    // Sensor or system specific detection confidence at time 𝑡:
    // c_t (between 0 for no detection confidence and 1 for high detection confidence).
    // int64_t c_t = 1; // in artery sensors case, we consider always 1
    // for this implementation, we consider that every time 100ms passes, the confidence is reduced by 0.1
    SimTime elapsed = T_now - lastDetection; // time elapsed since last detection
    double c_t = std::max(0L, 1000 - elapsed.inUnit(SIMTIME_MS)) / 1000.0;
    
    // Detection success, describing the assessment whether a given measurement has 
    // successfully perceived the object (binary assessment) at time 𝑡:
    // 𝑑_𝑡 (0 if not detected or 1 if detected).
    int d_t = 1; // in artery sensors case, any measurement is successfully detected, so it is always 1.

    // Determine object perception quality

    // Compute the exponential moving average for the system specific
    // detection confidence c_t with factor alpha, 0 <= alpha <= 1  

    double alpha = mAlpha;
    double EMA_t = c_t;
    if(t > 0) {
        double EMA_t_1 = 1; // TODO: get EMA_(t-1) 
        EMA_t = alpha * c_t + (1 - alpha) * EMA_t_1;
    }

    double r_c = floor(EMA_t * 15); // detection confidence rating

    // Compute the exponential moving average for the system specific
    // detection success c_t with factor alpha, 0 <= alpha <= 1  
    
    alpha = mAlpha;
    EMA_t = 1;
    if(t > 0) {
        double EMA_t_1 = 1; // TODO: get EMA_(t-1) 
        EMA_t = alpha * d_t + (1 - alpha) * EMA_t_1;
    }

    double r_d = floor(EMA_t * 15); // detection success rating

    // Compute the object age rating

    double r_oa = std::min(floor(oa/100.0), 15.0); // object age rating

    // Compute object perception quality as the weighted average of 𝑟_oa, r_c, r_d
    double w_d = mOmega_d;
    double w_c = mOmega_c;
    double w_oa = mOmega_oa;

    int objectPerceptionQuality = floor((w_d * r_d + w_c * r_c + w_oa * r_oa) / (w_d + w_c + w_oa));

    return objectPerceptionQuality;
}


/**** static local functions *****/

template<typename T, typename U>
static long roundToUnit(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

/*** message generation functions ***/

vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider& vdp, const uint64_t referenceTime)
{
    vanetza::asn1::Cpm message;

    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2; // standard-defined value
    header.messageId = MessageId_cpm; // standard-defined value
    header.stationId = vdp.station_id(); // unsigned 32-bit value

    CpmPayload_t& payload = (*message).payload;
    
    // Management Container
    CPM_PDU_Descriptions_ManagementContainer_t& mc = payload.managementContainer;
    // referenceTime -> 42-bit integer
    // assert(referenceTime < 2^24 && referenceTime >= 0)
    //memset(&mc.referenceTime, 0, sizeof(INTEGER_t));
    int ret = asn_uint642INTEGER(&mc.referenceTime, referenceTime);

    assert(ret == 0);
    // NOTE: The new CPM format uses a deprecated DE from CDD (ReferencePosition instead of ReferencePositionWithConfidence)
    // latitude and longitude units -> 10^-7 degrees
    mc.referencePosition.latitude = roundToUnit(vdp.latitude(), vanetza::units::degree * boost::units::si::micro) * 10; // extended to unit
    mc.referencePosition.longitude = roundToUnit(vdp.longitude(), vanetza::units::degree * boost::units::si::micro) * 10; // extended to unit
    mc.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    mc.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    mc.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    mc.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    mc.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

    return message;
}

void addOriginatingVehicleContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp)
{
    CpmPayload& payload = (*message).payload;
    ConstraintWrappedCpmContainers_t& cwccs = payload.cpmContainers;
    WrappedCpmContainers_t& wccs = cwccs; // NOTE: temporary solution until asn1 compilation is supported
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    wcc->containerId = CpmContainerId_originatingVehicleContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer;
    OriginatingVehicleContainer_t& ovc = wcc->containerData.choice.OriginatingVehicleContainer;

    // wgs84 unit -> 0.1 degrees
    // TODO: replace heading with orientation angle. see standard
    ovc.orientationAngle.value = roundToUnit(vdp.heading(), vanetza::units::degree * boost::units::si::deci); // between 0..3590
    ovc.orientationAngle.confidence = 10; // equal or within 1 degree (10 decidegrees)
    // TODO: Optional values
    ASN_SEQUENCE_ADD(&(wccs.list), wcc);
}

void addOriginatingRsuContainer(vanetza::asn1::Cpm& message)
{
    CpmPayload& payload = (*message).payload;
    ConstraintWrappedCpmContainers_t& cwccs = payload.cpmContainers;
    WrappedCpmContainers_t& wccs = cwccs; // NOTE: temporary solution until asn1 compilation is supported
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    wcc->containerId = CpmContainerId_originatingRsuContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_OriginatingRsuContainer;
    OriginatingRsuContainer_t& orc = wcc->containerData.choice.OriginatingRsuContainer;

    // TODO: add optional mapReference value

    ASN_SEQUENCE_ADD(&(wccs.list), wcc);
}

void addSensorInformationContainer(vanetza::asn1::Cpm& message, const std::vector<Sensor*>& sensors)
{
    CpmPayload& payload = (*message).payload;
    ConstraintWrappedCpmContainers_t& cwccs = payload.cpmContainers;
    WrappedCpmContainers_t& wccs = cwccs; // NOTE: temporary solution until asn1 compilation is supported
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    wcc->containerId = CpmContainerId_sensorInformationContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_SensorInformationContainer;
    SensorInformationContainer_t& sic = wcc->containerData.choice.SensorInformationContainer;
    
    uint8_t sensorId = 1; // NOTE: temporary solution to attribute sensor ids (vector order)
    for(const Sensor* sensor : sensors)
    {
        // add sensor information only related to FoV sensors
        if (auto fovSensor = dynamic_cast<const FovSensor*>(sensor))
        {
            SensorInformation_t* si = vanetza::asn1::allocate<SensorInformation_t>();
            si->sensorId = sensorId++;
            si->sensorType = SensorType_radar; // NOTE: for now, all FoV sensors in artery are radars

            si->perceptionRegionShape = vanetza::asn1::allocate<Shape_t>();
            si->perceptionRegionShape->present = Shape_PR_radial;
            RadialShape_t& rs = si->perceptionRegionShape->choice.radial;
            const FieldOfView& fov = fovSensor->getFieldOfView();
            rs.range = roundToUnit(fov.range, vanetza::units::si::meter * boost::units::si::deci); // 12-bit size, units -> decimeters
            // angle unit -> decidegree
            // cannot be implemented with vanilla sensors.
            // TODO: add function to retrieve start and end angles of sensor cone
            rs.horizontalOpeningAngleStart = CartesianAngleValue_unavailable;
            rs.horizontalOpeningAngleEnd = CartesianAngleValue_unavailable;
            // TODO: add radial shape optional values

            si->perceptionRegionConfidence = vanetza::asn1::allocate<ConfidenceLevel_t>();
            *(si->perceptionRegionConfidence) = 100; // unit -> percentage | TODO: check if this is correctly assigned

            // TODO: get value from sensor (drawLineOfSight)
            si->shadowingApplies = true; // confidence provided is not valid for shadowed regions

            ASN_SEQUENCE_ADD(&(sic.list), si);
        }
        // TODO: add support for other types of sensors (itssAgregation sensors)
    }

    ASN_SEQUENCE_ADD(&(wccs.list), wcc);
}

void addPerceptionRegionContainer(vanetza::asn1::Cpm& message)
{
    // The currently available sensors on artery do not provide dynamic perception regions
    // to check differences on shape, confidence and shadowing.
    // This container is never added to the CPM.
}

void addPerceivedObjectContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp, const std::vector<LocalEnvironmentModel::TrackedObject>& selectedObjects, const int& numberOfPerceivedObjects)
{
    ConstraintWrappedCpmContainers_t& cwccs = (*message).payload.cpmContainers;
    WrappedCpmContainers_t& wccs = cwccs;
    WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<WrappedCpmContainer_t>();
    wcc->containerId = CpmContainerId_perceivedObjectContainer;
    wcc->containerData.present = WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
    
    PerceivedObjectContainer_t& poc = wcc->containerData.choice.PerceivedObjectContainer;

    poc.numberOfPerceivedObjects = numberOfPerceivedObjects; // 8-bit number

    PerceivedObjects_t& pos = poc.perceivedObjects;

    using TrackedObjectWrapper = std::reference_wrapper<const LocalEnvironmentModel::TrackedObject>;
    using TrackedObject = LocalEnvironmentModel::TrackedObject;
    using Tracking = LocalEnvironmentModel::Tracking;

    // Ego vehicle position
    const Position& egoPos = vdp.position();

    // get timestampIts from management container.
    const CPM_PDU_Descriptions_ManagementContainer_t& mc = (*message).payload.managementContainer;
    const INTEGER_t& timestampIts = mc.referenceTime;
    int64_t referenceTime;
    asn_INTEGER2imax(&timestampIts, &referenceTime);

    for(TrackedObject obj: selectedObjects)
    {
        const Tracking& tracking = obj.second;

        PerceivedObject_t* po = vanetza::asn1::allocate<PerceivedObject_t>();
        po->objectId = vanetza::asn1::allocate<Identifier2B_t>(); // Obligatory
        *(po->objectId) = tracking.id(); // TODO: verify uniqueness and assert min and max values

        // Get last detected time
        SimTime lastDetected = SimTime::ZERO;
        for(const auto& sensor : tracking.sensors())
        {
            if(sensor.first->getSensorCategory() == "FoV")
            {
                if(lastDetected < sensor.second.last())
                {
                    lastDetected = sensor.second.last();
                }
            }
        }

        // a measurement delta time shall be provided for each object as the time difference for the provided measurement
        // information with respect to the reference time of type TimestampIts stated in the Management Container
        // time unit -> milliseconds
        int64_t deltaTime = lastDetected.inUnit(SIMTIME_MS) - referenceTime;
        // clamp measurementDeltaTime according to cdd documentation
        if(deltaTime < -2048) deltaTime = -2048;
        if(deltaTime > 2046) deltaTime = 2047;
        po->measurementDeltaTime = deltaTime;

        const Position& objectPos = obj.first.lock()->getCentrePoint();
        CartesianPosition3dWithConfidence_t& position = po->position;
        
        position.xCoordinate.value = roundToUnit(objectPos.x - egoPos.x, vanetza::units::si::meter * boost::units::si::centi);
        if(position.xCoordinate.value < CartesianCoordinateLarge_negativeOutOfRange)
            position.xCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
        if(position.xCoordinate.value > CartesianCoordinateLarge_positiveOutOfRange)
            position.xCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
        
        position.xCoordinate.confidence = 0;
        if(position.xCoordinate.confidence > 4094)
            position.xCoordinate.confidence = CoordinateConfidence_outOfRange;

        position.yCoordinate.value = roundToUnit(objectPos.y - egoPos.y, vanetza::units::si::meter * boost::units::si::centi);
        if(position.yCoordinate.value < CartesianCoordinateLarge_negativeOutOfRange)
            position.yCoordinate.value = CartesianCoordinateLarge_negativeOutOfRange;
        if(position.yCoordinate.value > CartesianCoordinateLarge_positiveOutOfRange)
            position.yCoordinate.value = CartesianCoordinateLarge_positiveOutOfRange;
        
        position.yCoordinate.confidence = 0;
        if(position.yCoordinate.confidence > 4094)
            position.yCoordinate.confidence = CoordinateConfidence_outOfRange;

        ASN_SEQUENCE_ADD(&(pos.list), po);
    }

    ASN_SEQUENCE_ADD(&(wccs.list), wcc);
}



/**** helper functions ****/

ObjectClass_t StationType2ObjectClass(const vanetza::geonet::StationType& type)
{
    using StationType = vanetza::geonet::StationType;
    ObjectClass_t objectClass;

    if(type == StationType::Pedestrian) {
        // NOTE: add pedestrian as a VRU instead of TrafficParticipant
        objectClass.present = ObjectClass_PR_vruSubClass;
        VruProfileAndSubprofile_t& vruSubClass = objectClass.choice.vruSubClass;
        vruSubClass.present = VruProfileAndSubprofile_PR_pedestrian;
        vruSubClass.choice.pedestrian = VruSubProfilePedestrian_unavailable;
    
    } if(type == StationType::Cyclist) {
        // NOTE: add pedestrian as a VRU instead of TrafficParticipant
        objectClass.present = ObjectClass_PR_vruSubClass;
        VruProfileAndSubprofile_t& vruSubClass = objectClass.choice.vruSubClass;
        vruSubClass.present = VruProfileAndSubprofile_PR_bicyclistAndLightVruVehicle;
        vruSubClass.choice.bicyclistAndLightVruVehicle = VruSubProfileBicyclist_unavailable;

    } if(type == StationType::Moped) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_moped;
    
    } if(type == StationType::Motorcycle) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_motorcycle;
    
    } if(type == StationType::Passenger_Car) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_passengerCar;
    
    } if(type == StationType::Bus) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_bus;
    
    } if(type == StationType::Light_Truck) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_lightTruck;
    
    } if(type == StationType::Heavy_Truck) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_heavyTruck;
    
    } if(type == StationType::Trailer) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_trailer;
    
    } if(type == StationType::Special_Vehicle) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_specialVehicle;
    
    } if(type == StationType::Tram) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.vehicleSubClass = TrafficParticipantType_tram;
    
    } if(type == StationType::RSU) {
        objectClass.present = ObjectClass_PR_vehicleSubClass;
        objectClass.choice.otherSubClass = TrafficParticipantType_roadSideUnit;
    
    } else {
        objectClass.present = ObjectClass_PR_otherSubClass;
        objectClass.choice.otherSubClass = OtherSubClass_unknown;
    }

    return objectClass;
}

} // namespace artery
