//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple CpService like ItsG5Service
{
    parameters:
        @signal[CpmReceived](type=CpObject);
        @signal[CpmSent](type=CpObject);
    
        @statistic[reception](source=CpmReceived;record=vector(cpmStationId)?,vector(cpmReferenceTime)?);
        @statistic[transmission](source=CpmSent;record=vector(cpmStationId)?,vector(cpmReferenceTime)?);
        
        // Configuration parameter values recommended for the CPS data inclusion management
        double T_GenCpmMin @unit(s) = default(0.1s);
        double T_GenCpmMax @unit(s) = default(1.0s);
        double T_AddSensorInformation @unit(s) = default(1.0s);
        
        int maxPerceptionRegions = default(8);
        int objectInclusionConfig @enum(0,1) = default(1);

        double objectPerceptionQualityThreshold = default(3.0);
        double minPositionChangeThreshold @unit(m) = default(4.0m);
        double minGroundSpeedChangeThreshold @unit(mps) = default(0.5mps);
        double minGroundVelocityOrientationChangeThreshold @unit(deg) = default(4.0deg); // TODO: check if 'int' or 'double'

        // Configuration parameter values recommended for the CPM assembly
        int messageAssemblyConfig @enum(0,1) = default(0);
        double minPositionChangePriorityThreshold @unit(m) = default(0.0m);
        double maxPositionChangePriorityThreshold @unit(m) = default(8.0m);
        double minGroundSpeedChangePriorityThreshold @unit(mps) = default(0.0mps);
        double maxGroundSpeedChangePriorityThreshold @unit(mps) = default(1.0mps);
        double minGroundVelocityOrientationChangePriorityThreshold @unit(deg) = default(0.0deg); // TODO: check if 'int' or 'double'
        double maxGroundVelocityOrientationChangePriorityThreshold @unit(deg) = default(8.0deg); // TODO: check if 'int' or 'double'
        double minLastInclusionTimePriorityThreshold @unit(s) = default(0.1s);
        double maxLastInclusionTimePriorityThreshold @unit(ms) = default(1.0s);

        // Configuration parameter values recommended for sensor and object ID management
        double unusedSensorIdRetentionPeriod @unit(s) = default(60.0s);
        double unusedObjectIdRetentionPeriod @unit(s) = default(60.0s);

        // Configuration parameter values recommended for the object perception quality assessment
        double alpha = default(0.5);
        double omega_d = default(1.0);
        double omega_c = default(1.0);
        double omega_oa = default(1.0);

        // Configuration parameter values recommended for the object inclusion rate control
        double C_InclusionRateControl = default(50.0); // percentage
        double W_InclusionRateControl @unit(s) = default(2.0s);
        int N_InclusionRateControl = default(10);
}