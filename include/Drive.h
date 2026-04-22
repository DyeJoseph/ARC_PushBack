#pragma once

#include "vex.h"
#include "odom.h"
#include "PID.h"
#include "sensorConversion.h"

using namespace vex;

enum MotorSpinType {VOLTS, PERCENTAGE, DPS, RPM};

class Drive
{
    private:
        motor_group leftDrive, rightDrive;
        inertial inertialSensor;

        float driveMaxVoltage;
        float turnMaxVoltage;
        float driveSlew = 1.0f; // max voltage increase per 10ms tick

        float wheelRatio, wheelDiameter;

        float driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime;
        float turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime;

        float scurveMaxVel   = 0.0f;  // in/s   — 0 disables the profile
        float scurveMaxAccel = 0.0f;  // in/s^2
        float scurveMaxJerk  = 0.0f;  // in/s^3 — primary slip-reduction knob
        float driveKff       = 0.0f;  // V / (in/s) feedforward gain
        float driveKs        = 0.0f;  // V — static friction compensation (min voltage to move)

        float stallThreshold = 0.5f;  // inches — minimum movement per stallTimeout to not be stalled
        float stallTimeout   = 500.0f;// ms     — 0 disables stall detection
        
        int odomType;
            
    public:
        Odom chassisOdometry;
        float predictedAngle;

        Drive(motor_group leftDrive, motor_group rightDrive, int inertialPORT, float wheelDiameter, float wheelRatio, 
            float maxVoltage, int odomType, float odomWheelDiameter, float odomPod1Offset, float odomPod2Offset);

        void setDriveMaxVoltage(float maxVoltage);
        void setTurnMaxVoltage(float maxVoltage);
        void setDriveSlew(float slew);

        void setDriveConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime);
        void setTurnConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime);
        void setSCurveConstants(float maxVel, float maxAccel, float maxJerk);
        void setDriveKff(float kff);
        void setDriveKs(float ks);
        void setStallDetection(float threshold, float timeout);
        // void setDriveProfileForDistance(float distance, float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime);
        // void setTurnProfileForAngle(float angle, float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime);
        
        void arcade();
        void tank();

        float getCurrentMotorPosition();
        int getOdomType() const {return odomType;}

        void driveMotors(float leftVolts, float rightVolts);
        void driveMotors(float leftVolts, float rightVolts, MotorSpinType spinType);

        void brake();
        void brake(brakeType);
        void brake(bool left, bool right);
        void brake(bool left, bool right, brakeType);

        void driveDistance(float distance);
        void driveDistance(float distance, float maxVoltage);
        void driveDistanceWithOdom(float distance);
        void driveDistanceWithOdom(float distance, float timeLimit);
        void driveDistanceWithOdom(float distance, float timeLimit, float maxVoltage);
        void driveDistanceWithOdom(float distance, float timeLimit, float maxVoltage, float settleTime, float settleError);
            
        void moveable();

        void setMaxVoltage(float volts);
        float getMaxVoltage();

        void turn(float turnDegrees);
        void turn(float turnDegrees, float maxVoltage);

        void turnToAngle(float angle);
        void turnToAngle(float angle, float maxVoltage);
        void turnToAngle(float angle, float timeLimit, float maxVoltage);
        //void turnToAngle(float angle, float maxVoltage, float turnKdUpdate);
        
        void moveToPosition(float, float);
        void turnToPosition(float desX, float desY);

        void bezierTurn(float, float, float, float, float, float, int);

        void updatePosition();
        void setPosition(float x, float y, float heading);
        void movetopos(float x, float y, float angle);

};
