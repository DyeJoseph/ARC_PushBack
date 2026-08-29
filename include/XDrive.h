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
        motor_group frontLeft, frontRight, backLeft, backRight;

        float driveMaxVoltage;
        float turnMaxVoltage;

        float wheelRatio, wheelDiameter;

        float driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime;
        float turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime;
        
        int odomType;
            
    public:
        Odom chassisOdometry;

        XDrive(motor_group FL, motor_group FR, motor_group BL, motor_group BR, float wheelDiameter, float wheelRatio, float maxVoltage, int odomType, float odomWheelDiameter, float odomPod1Offset, float odomPod2Offset);

        /// @brief User controls for x-drive
        void xDriveControls();

        /// @brief X-Drive move to position (will move and turn simultaneously)
        /// @param target a 3-float array of [desired X, desired Y, desired final heading]
        void xMoveToPosition(float *target);

        void setDriveMaxVoltage(float maxVoltage);
        void setTurnMaxVoltage(float maxVoltage);
        void setDriveConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime);
        void setTurnConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime);
        void brake(vex::brakeType);
        void movable();
        void updatePostion();


};