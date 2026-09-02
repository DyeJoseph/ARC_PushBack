#include "XDrive.h"


XDrive::XDrive(motor_group FL, motor_group FR, motor_group BL, motor_group BR, float wheelDiameter, float wheelRatio, float maxVoltage, int odomType, float odomWheelDiameter, float odomPod1Offset, float odomPod2Offset){
    frontLeft = FL;
    frontRight = FR;
    backLeft = BL;
    backRight = BR;

    this->wheelDiameter = wheelDiameter;
    this->wheelRatio = wheelRatio;
    driveMaxVoltage = maxVoltage;
    turnMaxVoltage = maxVoltage;
    this->odomType = odomType;

    switch(odomType){
        case NO_ODOM:
            this->chassisOdometry = Odom(wheelDiameter, wheelDiameter, 0, odomPod1Offset, odomPod2Offset, 0);
            break;
        case HORIZONTAL_AND_VERTICAL:
            this->chassisOdometry = Odom(odomWheelDiameter, odomWheelDiameter, odomPod1Offset, odomPod2Offset);
            break;
        case TWO_VERTICAL:
            // TODO
            break;
        case TWO_AT_45:
            this->chassisOdometry = Odom(odomWheelDiameter, odomPod1Offset, odomPod2Offset);
            break;
    }
}

void XDrive::xDriveControls(){
    //Calculate motor values (do not reverse motors in robot-config, keep all spinning fwd)
    // FL = y + x + r
    float frontLeftVal = Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent) + Controller1.Axis1.position(percent);
    //FR = -y + x + r
    float frontRightVal = -1 * Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent) + Controller1.Axis1.position(percent);
    //BR = -y - x + r
    float backRightVal = -1 * Controller1.Axis3.position(percent) - Controller1.Axis4.position(percent) + Controller1.Axis1.position(percent);
    //BL = y - x + r
    float backLeftVal = Controller1.Axis3.position(percent) - Controller1.Axis4.position(percent) + Controller1.Axis1.position(percent);

    //Find largest value, then divide by that to scale everything down
    float denominator = std::max({fabs(frontLeftVal), fabs(frontRightVal), fabs(backRightVal), fabs(backLeftVal), 100.0});
    frontLeftVal = frontLeftVal / denominator * 100;
    frontRightVal = frontRightVal / denominator * 100;
    backLeftVal = backLeftVal / denominator * 100;
    backRightVal = backRightVal / denominator * 100;

    //Spin motors/motor groups based on calculated values
    frontLeft.spin(fwd, frontLeftVal, percent);
    frontRight.spin(fwd, frontRightVal, percent);
    backLeft.spin(fwd, backLeftVal, percent);
    backRight.spin(fwd, backRightVal, percent);
}
/// @brief Move to pos for XDrive
/// @param target x,y,heading vector for desired position
void XDrive::xMoveToPosition(float *target){
    //Get current positions
    float curX = chassisOdometry.getXPosition();
    float curY = chassisOdometry.getYPosition();
    float curHead = inertial1.heading();

    //Separate vector into variables
    float tarX = target[0];
    float tarY = target[1];
    float tarHead = target[2];

    //Get difference in x and y
    float xDiff = tarX - curX;
    float yDiff = tarY - curY;

    //Calculate distance
    //float distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2));

    //Create PID objects
    PID xPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID yPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);
    
    //Drive until all 3 PIDs have settled
    while(!(xPID.isSettled() && yPID.isSettled() && angularPID.isSettled())){
        //Update position variables
        curX = chassisOdometry.getXPosition();
        curY = chassisOdometry.getYPosition();
        curHead = inertial1.heading();

        //Recalculate x/y based on new position
        xDiff = tarX - curX;
        yDiff = tarY - curY;

        //Get current heading in radians
        float curHeadRad = degToRad(curHead);

        //2D rotation matrix for robot-centric controls
        //[cos(theta) -sin(theta)] [x]
        //[sin(theta) cos(theta) ] [y]
        float robotX = (cos(curHeadRad) * xDiff) - (sin(curHeadRad) * yDiff);
        float robotY = (sin(curHeadRad) * xDiff) + (cos(curHeadRad) * yDiff);

        //Kalman filter stuff here

        //Compute PIDs
        float xOutput = xPID.compute(robotX);
        float yOutput = yPID.compute(robotY);
        float angularOutput = angularPID.compute(degTo180(tarHead - curHead));

        //Clamp PID outputs to 12V to avoid extreme outputs
        xOutput = clamp(xOutput, -driveMaxVoltage, driveMaxVoltage);
        yOutput = clamp(yOutput, -driveMaxVoltage, driveMaxVoltage);
        angularOutput = clamp(angularOutput, -driveMaxVoltage, driveMaxVoltage);

        //Calculate motor powers
        float frontLeftVal = xOutput + yOutput + angularOutput;
        float frontRightVal = xOutput - yOutput + angularOutput;
        float backLeftVal = -1.0 * xOutput + yOutput + angularOutput;
        float backRightVal = -1.0 * xOutput - yOutput + angularOutput;

        //Clamp motors based on largest value
        float denominator = std::max({fabs(frontLeftVal), fabs(frontRightVal), fabs(backLeftVal), fabs(backRightVal), fabs(driveMaxVoltage)});
        frontLeftVal = frontLeftVal / denominator * driveMaxVoltage;
        frontRightVal = frontRightVal / denominator * driveMaxVoltage;
        backLeftVal = backLeftVal / denominator * driveMaxVoltage;
        backRightVal = backRightVal / denominator * driveMaxVoltage;

        //Spin motors
        frontLeft.spin(fwd, frontLeftVal, volt);
        frontRight.spin(fwd, frontRightVal, volt);
        backLeft.spin(fwd, backLeftVal, volt);
        backRight.spin(fwd, backRightVal, volt);

        wait(10, msec); //If changed from 10ms, make sure to change "time" in PID's compute function as well
    }
    //Brake motors

}

void XDrive::setDriveMaxVoltage(float maxVoltage){ driveMaxVoltage = maxVoltage; }
void XDrive::setTurnMaxVoltage(float maxVoltage){ turnMaxVoltage = maxVoltage; }

void XDrive::setDriveConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime){
    driveKp = Kp;
    driveKi = Ki;
    driveKd = Kd;
    driveSettleError = settleError;
    driveTimeToSettle = timeToSettle;
    driveEndTime = endTime;
}

void XDrive::setTurnConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime){
    turnKp = Kp;
    turnKi = Ki;
    turnKd = Kd;
    turnSettleError = settleError;
    turnTimeToSettle = timeToSettle;
    turnEndTime = endTime;
}

void XDrive::brake(vex::brakeType type = hold){
    frontLeft.stop(type);
    frontRight.stop(type);
    backLeft.stop(type);
    backRight.stop(type);
}

void XDrive::movable(){
    while (true) {
        brake(coast);
        updatePostion();
        float x = chassisOdometry.getXPosition();
        float y = chassisOdometry.getYPosition();
        float heading = chassisOdometry.getHeading();

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("X: ");
        Brain.Screen.print(x);
        Brain.Screen.newLine();
        Brain.Screen.print("Y: ");
        Brain.Screen.print(y);
        Brain.Screen.print("Heading: ");
        Brain.Screen.print(heading);
        wait(50, msec); 
    }
}

void XDrive::updatePostion(){
    switch(odomType){
        float left, right, heading;
        case NO_ODOM:
            left = leftDrive.position(degrees);
            right = rightDrive.position(degrees);
            heading = inertial1.heading();
            chassisOdometry.updatePositionTwoForward(right, left, heading);
            break;
        case HORIZONTAL_AND_VERTICAL:
            left = rotation1.position(degrees);
            right = rotation2.position(degrees);
            heading = inertial1.heading();
            chassisOdometry.updatePositionOneForward(left, right, heading);
            break;
        case TWO_VERTICAL:
            left = rotation1.position(degrees);
            right = rotation2.position(degrees);
            heading = inertial1.heading();
            chassisOdometry.updatePositionTwoForward(right, left, heading);
            break;
        case TWO_AT_45:
            left = rotation1.position(degrees);
            right = rotation2.position(degrees);
            heading = inertial1.heading();
            chassisOdometry.updatePositionTwoAt45(left, right, heading);
            break;
    }
}
