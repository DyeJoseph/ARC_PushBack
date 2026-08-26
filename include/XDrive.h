#pragma once

#include "vex.h"
#include "odom.h"
#include "PID.h"
#include "sensorConversion.h"
#include <algorithm>

using namespace vex;

enum MotorSpinType {VOLTS, PERCENTAGE, DPS, RPM};

//Temp motor definitions
vex::motor frontLeft = vex::motor(PORT1, ratio6_1, true);
vex::motor frontRight = vex::motor(PORT1, ratio6_1, true);
vex::motor backLeft = vex::motor(PORT1, ratio6_1, true);
vex::motor backRight = vex::motor(PORT1, ratio6_1, true);

class XDrive{
    private:
        motor_group leftDrive, rightDrive;
        inertial inertialSensor;

        float driveMaxVoltage;
        float turnMaxVoltage;

        float wheelRatio, wheelDiameter;

        float driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime;
        float turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime;
        
        int odomType;
            
    public:
        Odom chassisOdometry;

        /// @brief User controls for x-drive
        void xDriveControls();
        /// @brief X-Drive move to position (will move and turn simultaneously)
        /// @param target a 3-float array of [desired X, desired Y, desired final heading]
        void xMoveToPosition(float *target);
};