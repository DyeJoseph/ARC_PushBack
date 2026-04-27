#include "Drive.h"
#include "SCurveProfile.h"
#include <array>
#include <cmath>

/* ================= */
/* DRIVE CONSTRUCTOR */
/* ================= */



/// @brief Creates a array that holds the distances that the drive PID can be tuned for
/// @param distance The distance to drive in inches
std::array<float, 5> kDistances = {3.0f, 12.0f, 24.0f, 36.0f, 48.0f};

// Tuned drive profiles are stored globally so the tuner can update one bucket at runtime.
// std::array<PID, 5> drivePIDProfiles = {{
//     // PID(Kp, Ki, Kd, settleError, timeToSettle, endTime)
//     PID(1.0f, 0.0000f, 5.0f, 1.0f, 200.0f, 5000.0f),  // 3 inches
//     PID(1.0f, 0.0000f, 5.0f, 1.0f, 200.0f, 5000.0f),  // 12 inches
//     PID(1.0f, 0.0000f, 6.0f, .5f, 200.0f, 5000.0f),  // 24 inches
//     PID(1.0f, 0.0000f, 5.0f, 1.0f, 200.0f, 5000.0f),  // 36 inches
//     PID(.8f, 0.0000f, 5.0f, 1.0f, 200.0f, 5000.0f),  // 48 inches
// }};

/// @brief Selects the PID array index that is closest to the target distance
/// @param distance The distance to drive in inches
/// @return The index of the closest PID profile
int getClosestDistanceProfileIndex(float distance) {
    const float target = std::fabs(distance);
    int closestIndex = 0;
    float bestDelta = std::fabs(target - kDistances[0]);

    for (int i = 1; i < static_cast<int>(kDistances.size()); ++i) {
        const float delta = std::fabs(target - kDistances[i]);
        if (delta < bestDelta) {
            bestDelta = delta;
            closestIndex = i;
        }
    }

    return closestIndex;
}

// /// @brief Retrieves the array with the PID profile for driving distances
// /// @return An array of PID profiles for each distance
// const std::array<PID, 5>& getDrivePIDProfiles() {
//     return drivePIDProfiles;
// }


/// @brief Creates a array that holds the angles that the turn PID can be tuned for
/// @param angle The angle to turn in degrees
std::array<float, 5> kTurnAngles = {5.0f, 30.0f, 45.0f, 90.0f, 180.0f};

// Tuned turn profiles are stored globally so the tuner can update one bucket at runtime.
// std::array<PID, 5> turnPIDProfiles = {{
//     // PID(Kp, Ki, Kd, settleError, timeToSettle, endTime)
//     PID(0.0f, 0.0000f, 0.0f, 0.0f, 0.0f, 0.0f),  // 5 degrees
//     PID(0.0f, 0.0000f, 0.0f, 0.0f, 0.0f, 0.0f),  // 30 degrees
//     PID(0.0f, 0.0000f, 0.0f, 0.0f, 0.0f, 0.0f),  // 45 degrees
//     PID(0.0f, 0.0000f, 0.0f, 0.0f, 0.0f, 0.0f),  // 90 degrees
//     PID(0.0f, 0.0000f, 0.0f, 0.0f, 0.0f, 0.0f),  // 180 degrees
// }};

/// @brief Selects the PID array index that is closest to the target angle
/// @param angle The angle to turn in degrees
/// @return The index of the closest PID profile
int getClosestTurnProfileIndex(float angle) {
    const float target = std::fabs(angle);
    int closestIndex = 0;
    float bestDelta = std::fabs(target - kTurnAngles[0]);

    for (int i = 1; i < static_cast<int>(kTurnAngles.size()); ++i) {
        const float delta = std::fabs(target - kTurnAngles[i]);
        if (delta < bestDelta) {
            bestDelta = delta;
            closestIndex = i;
        }
    }

    return closestIndex;
}

// /// @brief Retrieves the array with the PID profile for turning angles
// /// @return An array of PID profiles for each angle
// const std::array<PID, 5>& getTurnPIDProfiles() {
//     return turnPIDProfiles;
// }

/* ============ */
/*  CONSTRUCTOR */
/* ============ */



/// @brief Constructor
/// @param leftDrive Left side motors of the drive base
/// @param rightDrive Right side motors of the drive base
/// @param inertialPort The Port where the inertial sensor is 
/// @param wheelDiameter The diameter size of the wheel in inches
/// @param wheelRatio   
/// @param max_voltage The maximum amount of the voltage used in the drivebase (1 - 12)
Drive::Drive(motor_group leftDrive, motor_group rightDrive, int inertialPORT, float wheelDiameter, float wheelRatio, float maxVoltage, int odomType, float odomWheelDiameter, float odomPod1Offset, float odomPod2Offset ) : 
leftDrive(leftDrive), 
rightDrive(rightDrive),
inertialSensor(inertial(inertialPORT))
{
    this->wheelDiameter = wheelDiameter;
    this->wheelRatio = wheelRatio;
    this->driveMaxVoltage = maxVoltage;
    this->turnMaxVoltage = maxVoltage;
    this->odomType = odomType;

    switch(odomType){
        case NO_ODOM:
            this->chassisOdometry = Odom(wheelDiameter, wheelDiameter, 0, odomPod1Offset, odomPod2Offset, 0);
            break;
        case HORIZONTAL_AND_VERTICAL:
            this->chassisOdometry = Odom(odomWheelDiameter, odomWheelDiameter, odomPod1Offset, odomPod2Offset);
            break;
        case TWO_VERTICAL:
            // Not yet implemented
            break;
        case TWO_AT_45:
            this->chassisOdometry = Odom(odomWheelDiameter, odomPod1Offset, odomPod2Offset);
            break;
    }
}


/* ============ */
/* SET VOLTAGES */
/* ============ */

/// @brief 
/// @param maxVoltage Maximum voltage for the drive base
void Drive::setDriveMaxVoltage(float maxVoltage)
{
    driveMaxVoltage = maxVoltage;
}

/// @brief 
/// @param maxVoltage Maximum voltage for turning
void Drive::setTurnMaxVoltage(float maxVoltage)
{
    turnMaxVoltage = maxVoltage;
}

void Drive::setDriveSlew(float slew)
{
    driveSlew = slew;
}

void Drive::setSCurveConstants(float maxVel, float maxAccel, float maxJerk)
{
    scurveMaxVel   = maxVel;
    scurveMaxAccel = maxAccel;
    scurveMaxJerk  = maxJerk;
}

void Drive::setDriveKff(float kff)
{
    driveKff = kff;
}

void Drive::setDriveKs(float ks)
{
    driveKs = ks;
}

void Drive::setStallDetection(float threshold, float timeout)
{
    stallThreshold = threshold;
    stallTimeout   = timeout;
}


/* ============= */
/* SET CONSTANTS */
/* ============= */

/// @brief Sets the PID constants for the Drive distance 
/// @param Kp Proportion Constant
/// @param Ki Integral Constant
/// @param Kd Derivative Constant
/// @param settleError The Error reached when settle should start
/// @param timeToSettle The time in milliseconds to settle
/// @param endTime The total run time in milliseconds
void Drive::setDriveConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime)
{
    driveKp = Kp;
    driveKi = Ki;
    driveKd = Kd;
    driveSettleError = settleError;
    driveTimeToSettle = timeToSettle;
    driveEndTime = endTime;
}

/// @brief Sets the PID constants for the turn angle
/// @param Kp Proportion Constant
/// @param Ki Integral Constant
/// @param Kd Derivative Constant
/// @param settleError The Error reached when settle should start
/// @param timeToSettle The time in milliseconds to settle
/// @param endTime The total run time in milliseconds
void Drive::setTurnConstants(float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime)
{
    turnKp = Kp;
    turnKi = Ki;
    turnKd = Kd;
    turnSettleError = settleError;
    turnTimeToSettle = timeToSettle;
    turnEndTime = endTime;

}

// /// @brief Sets a drive PID profile for the closest distance bucket
// void Drive::setDriveProfileForDistance(float distance, float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime)
// {
//     const int profileIndex = getClosestDistanceProfileIndex(distance);
//     drivePIDProfiles[profileIndex].~PID();
//     new (&drivePIDProfiles[profileIndex]) PID(Kp, Ki, Kd, settleError, timeToSettle, endTime);
// }

// /// @brief Sets a turn PID profile for the closest angle bucket
// void Drive::setTurnProfileForAngle(float angle, float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime)
// {
//     const int profileIndex = getClosestTurnProfileIndex(angle);
//     turnPIDProfiles[profileIndex].~PID();
//     new (&turnPIDProfiles[profileIndex]) PID(Kp, Ki, Kd, settleError, timeToSettle, endTime);
// }


/* =========== */
/* DRIVE TYPES */
/* =========== */

/// @brief Arcade drive control, uses the left joystick for forward/backward and the right joystick for turning
void Drive::arcade()
{
    double leftY = 0;
    double rightX = 0;

    if(Controller1.Axis3.position(percent) >= 0)
        leftY = pow(Controller1.Axis3.position(percent),1.5)/10;
    else
        leftY = pow(fabs(Controller1.Axis3.position(percent)),1.5)/-10;
    
    if(Controller1.Axis1.position(percent) >= 0)
        rightX = pow(Controller1.Axis1.position(percent),1.5)/10;
    else
        rightX = pow(fabs(Controller1.Axis1.position(percent)),1.5)/-10;

    leftDrive.spin(forward, leftY+rightX, percent);
    rightDrive.spin(forward, leftY-rightX, percent);
}

void Drive::tank10(){
    int leftY = 0;
    int rightX = 0;
    leftY = Controller1.Axis3.position(percent);
    rightX = Controller1.Axis2.position(percent);

    leftDrive.spin(forward, leftY, percent);
    rightDrive.spin(forward, rightX, percent);
}

void Drive::tank15(){
    int leftY = 0;
    int rightX = 0;
    if(Controller1.Axis3.position(percent) >= 0)
        leftY = pow(fabs(Controller1.Axis3.position(percent)),1.5)/10;
        //leftY = Controller1.Axis3.position(percent);
    else
        leftY = pow(fabs(Controller1.Axis3.position(percent)),1.5)/-10;
        //leftY = Controller1.Axis3.position(percent);
    
    if(Controller1.Axis2.position(percent) >= 0)
        rightX = pow(fabs(Controller1.Axis2.position(percent)),1.5)/10;
        //rightX = Controller1.Axis2.position(percent);
    else
        rightX = pow(fabs(Controller1.Axis2.position(percent)),1.5)/-10;
        //rightX = Controller1.Axis2.position(percent);

    leftDrive.spin(forward, leftY, percent);
    rightDrive.spin(forward, rightX, percent);
}

void Drive::tank20(){
    int leftY = 0;
    int rightX = 0;
    if(Controller1.Axis3.position(percent) >= 0)
        leftY = pow(fabs(Controller1.Axis3.position(percent)),2)/100;
        //leftY = Controller1.Axis3.position(percent);
    else
        leftY = pow(fabs(Controller1.Axis3.position(percent)),2)/-100;
        //leftY = Controller1.Axis3.position(percent);
    
    if(Controller1.Axis2.position(percent) >= 0)
        rightX = pow(fabs(Controller1.Axis2.position(percent)),2)/100;
        //rightX = Controller1.Axis2.position(percent);
    else
        rightX = pow(fabs(Controller1.Axis2.position(percent)),2)/-100;
        //rightX = Controller1.Axis2.position(percent);

    leftDrive.spin(forward, leftY, percent);
    rightDrive.spin(forward, rightX, percent);
}

void Drive::tank25(){
    int leftY = 0;
    int rightX = 0;
    if(Controller1.Axis3.position(percent) >= 0)
        leftY = pow(fabs(Controller1.Axis3.position(percent)),2.5)/1000;
        //leftY = Controller1.Axis3.position(percent);
    else
        leftY = pow(fabs(Controller1.Axis3.position(percent)),2.5)/-1000;
        //leftY = Controller1.Axis3.position(percent);
    
    if(Controller1.Axis2.position(percent) >= 0)
        rightX = pow(fabs(Controller1.Axis2.position(percent)),2.5)/1000;
        //rightX = Controller1.Axis2.position(percent);
    else
        rightX = pow(fabs(Controller1.Axis2.position(percent)),2.5)/-1000;
        //rightX = Controller1.Axis2.position(percent);

    leftDrive.spin(forward, leftY, percent);
    rightDrive.spin(forward, rightX, percent);
}

void Drive::tank30(){
    int leftY = 0;
    int rightX = 0;
        
    leftY = pow(Controller1.Axis3.position(percent),3.0)/10000;
    rightX = pow(Controller1.Axis2.position(percent),3.0)/10000;

    leftDrive.spin(forward, leftY, percent);
    rightDrive.spin(forward, rightX, percent);
}


/* ======== */
/* STOPPING */
/* ======== */

/// @brief Brakes the drivetrain 
void Drive::brake()
{
    brake(true, true);
}

/// @brief Brakes the drivetrain
/// @param type The type of brakeType
void Drive::brake(brakeType type)
{
    brake(true, true, type);
}

/// @brief Brakes individual sides of the drive train using hold by default
/// @param left Left side of the drive train brake
/// @param right Right side of the drive train brake
void Drive::brake(bool left, bool right)
{
    brake(left, right, hold);
}

/// @brief Brakes individual sides of the drive train using brake type
/// @param left Left side of the drive train brake
/// @param right Right side of the drive train brake
/// @param type The type of brakeType
void Drive::brake(bool left, bool right, brakeType type)
{
    if(left)
        leftDrive.stop(type);
    if(right)
        rightDrive.stop(type);
}

/* ======= */
/* TURNING */
/* ======= */

/// @brief Turns the robot a set amount of degrees
/// @param turnDegrees A number in degrees the robot should rotate
void Drive::turn(float turnDegrees){
    turnToAngle(turnDegrees + inertial1.heading());
}

/// @brief Turns the robot a set amount of degrees with a max voltage
/// @param turnDegrees A number in degrees the robot should rotate
/// @param maxVoltage The max amount of voltage used to turn
void Drive::turn(float turnDegrees, float maxVoltage){
    turnToAngle(turnDegrees + inertial1.heading(), maxVoltage);
}

/// @brief Turns to an absolute specific angle
/// @param angle The angle to turn to in degrees (0 - 360)
void Drive::turnToAngle(float angle)
{
    turnToAngle(angle, turnMaxVoltage);
}

/// @brief Turns to an absolute specific angle with a max voltage
/// @param angle The angle to turn to in degrees (0 - 360)
/// @param maxVoltage The max amount of voltage used to turn
void Drive::turnToAngle(float angle, float maxVoltage)
{
    updatePosition();
    angle = inTermsOfNegative180To180(angle);
    PID turnPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);
    do
    {
        float error = inTermsOfNegative180To180(inertial1.heading()-angle);
        float output = turnPID.compute(error);

        float absOutput = fabs(output);
        if      (absOutput < 1.15f) output = copysign(1.0f,  output);
        else                        output = clamp(output, -maxVoltage, maxVoltage);

        driveMotors(-output, output);
        task::sleep(10);
    }while(!turnPID.isSettled());
    brake();
    updatePosition();
}

/// @brief Turns to an absolute specific angle with max voltage and end time
/// @param angle The angle to turn to in degrees (0-360)
/// @param maxVoltage The max amount of voltage used to turn
/// @param endTime The maximum time allowed to turn
void Drive::turnToAngle(float angle, float maxVoltage, float endTime){
    updatePosition();
    angle = inTermsOfNegative180To180(angle);
    PID turnPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, endTime);
    do
    {
        float error = inTermsOfNegative180To180(inertial1.heading()-angle);
        float output = turnPID.compute(error);

        //Minimum output threshold for turning
        if(fabs(output) < 0.85)
            if(output < 0)
                output = -1.4;
            else
                output = 1.4;
        else if (fabs(output) < 1.5)
             if(output < 0)
                output = -3.4;
            else
                output = 3.4;
        else if (fabs(output) < 5)
            if(output < 0)
                output = -5.4;
            else
                output = 5.4;
        else
            output = clamp(output, -maxVoltage, maxVoltage);

        driveMotors(-output, output);
        task::sleep(10);
    }while(!turnPID.isSettled());
    brake();
    updatePosition();
}

/// @brief 
/// @param desX Target X position
/// @param desY Target Y position
void Drive::turnToPosition(float desX, float desY){
    updatePosition();
    float deltaX = desX-chassisOdometry.getXPosition();
    float deltaY = desY-chassisOdometry.getYPosition();
    float angle = atan2(deltaX, deltaY) * (180.0/M_PI);
    turnToAngle(angle);
    updatePosition();
}


/* ================ */
/* STRAIGHT DRIVING */
/* ================ */

/// @brief Spins the drive train motors given the values, this function defaults to using volts
/// @param leftUnit Units of movement in volts for the left side of the drive train
/// @param rightUnit Units of movement in volts for the right side of the drive train
void Drive::driveMotors(float leftUnit, float rightUnit)
{
    driveMotors(leftUnit, rightUnit, VOLTS);
}

/// @brief Spins depending on the spin type with the given values
/// @param leftUnit Units of movement for the left side of the drive train
/// @param rightUnit Units of movement for the right side of the drive train
/// @param spinType The type used to spin the motors, can be: VOLTS, PERCENTAGE, DPS, or RPM
void Drive::driveMotors(float leftUnit, float rightUnit, MotorSpinType spinType)
{
    if(spinType == VOLTS)
    {
        leftDrive.spin(forward, leftUnit, volt);
        rightDrive.spin(forward, rightUnit, volt);
    }
    else if(spinType == PERCENTAGE)
    {
        leftDrive.spin(forward, leftUnit, pct);
        rightDrive.spin(forward, rightUnit, pct);
    }
    else if(spinType == DPS)
    {
        leftDrive.spin(forward, leftUnit, dps);
        rightDrive.spin(forward, rightUnit, dps);
    }
    else if(spinType == RPM)
    {
        leftDrive.spin(forward, leftUnit, rpm);
        rightDrive.spin(forward, rightUnit, rpm);
    }
}

/// @brief Uses the drivetrain to drive the given distance in inches
/// @param distance The distance to drive in inches
void Drive::driveDistance(float distance)
{
    driveDistance(distance, driveMaxVoltage);
}

/// @brief Uses the drivetrain to drive the given distance in inches
/// @param distance The distance to drive in inches
/// @param maxVoltage The max amount of voltage used to drive
void Drive::driveDistance(float distance, float maxVoltage)
{
    // Creates PID objects for linear and angular output
    // float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);


    updatePosition();
    float startPosition = getCurrentMotorPosition();
    float startHeading  = inertial1.heading();

    bool useProfile = (scurveMaxVel > 0.0f && scurveMaxAccel > 0.0f && scurveMaxJerk > 0.0f);
    SCurveProfile profile(distance,
                          useProfile ? scurveMaxVel   : 1.0f,
                          useProfile ? scurveMaxAccel : 1.0f,
                          useProfile ? scurveMaxJerk  : 1.0f);
    float elapsed = 0.0f;
    bool profileActive = useProfile;

    distance += startPosition;  // absolute motor-encoder target

    float prevLinearOutput = 0.0f;

    // Stall detection: track position over time and bail if it hasn't changed.
    float stallRefPos   = startPosition;
    float stallTimer    = 0.0f;
    bool  stallDetect   = (stallTimeout > 0.0f);

    // profileActive keeps the loop running until the profile finishes.
    // !linearPID.isSettled() is short-circuit-skipped while profileActive is true,
    // so isSettled() is only called (and "SETTLED" only printed) after the profile ends.
    while (profileActive || !linearPID.isSettled())
    {
        updatePosition();
        float currentPos   = getCurrentMotorPosition();
        float angularError = degTo180(startHeading - inertial1.heading());

        float pidError, feedforward = 0.0f;

        if (useProfile) {
            SCurveProfile::State state = profile.getState(elapsed);
            elapsed += 0.01f;

            if (profileActive && elapsed >= profile.totalTime()) {
                profileActive = false;
                // Reset settle timer so accumulated tracking time doesn't
                // trigger an immediate false-settle on the final target error.
                linearPID.resetSettle();
            }

            float relActual = currentPos - startPosition;
            pidError    = profileActive
                          ? (state.position - relActual)  // track planned trajectory
                          : (distance - currentPos);      // settle to final target
            if (std::fabs(state.velocity) > 0.01f)
                feedforward = driveKff * state.velocity
                              + driveKs * (state.velocity > 0.0f ? 1.0f : -1.0f);
            else
                feedforward = 0.0f;
        } else {
            pidError = distance - currentPos;
        }

        float linearOutput  = linearPID.compute(pidError);
        float angularOutput = angularPID.compute(angularError);

        if (useProfile) {
            linearOutput += feedforward;
        } else {
            float delta = linearOutput - prevLinearOutput;
            if      (delta >  driveSlew) linearOutput = prevLinearOutput + driveSlew;
            else if (delta < -driveSlew) linearOutput = prevLinearOutput - driveSlew;
            prevLinearOutput = linearOutput;
        }

        linearOutput  = clamp(linearOutput,  -maxVoltage, maxVoltage);
        angularOutput = clamp(angularOutput, -maxVoltage, maxVoltage);

        driveMotors(linearOutput + angularOutput, linearOutput - angularOutput);
        float pidContrib = linearOutput - feedforward;
        //std::cout << "pid:" << pidContrib << " ff:" << feedforward << " total:" << linearOutput << std::endl;

        // Stall detection: reset the reference whenever the robot moves enough,
        // otherwise accumulate time. Exit if stalled too long.
        if (stallDetect) {
            if (std::fabs(currentPos - stallRefPos) >= stallThreshold) {
                stallRefPos = currentPos;
                stallTimer  = 0.0f;
            } else {
                stallTimer += 10.0f;
                if (stallTimer >= stallTimeout) {
                    std::cout << "STALL------------------" << std::endl;
                    break;
                }
            }
        }

        wait(10, msec);
    }

    brake();
    updatePosition();
}

/// @brief Drive a certain distance using the odometry wheels
/// @param distance How far the robot (in inches) needs to drive
void Drive::driveDistanceWithOdom(float distance){
    //Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);

    updatePosition();

    //Starting pose (field coordinates & heading)
    float startHeadingDeg = inertial1.heading();
    float startHeadingRad = degToRad(startHeadingDeg);

    //Unit forward direction based on starting heading
    float dirX = sin(startHeadingRad);
    float dirY = cos(startHeadingRad);

    //Starting position in field coordinates
    float startX = chassisOdometry.getXPosition();
    float startY = chassisOdometry.getYPosition();

    //Target point in field coordinates (distance along starting heading)
    float targetX = startX + dirX * distance;
    float targetY = startY + dirY * distance;

    while (!linearPID.isSettled())
    {
        updatePosition();

        //Odom-based pose
        float curX = chassisOdometry.getXPosition();
        float curY = chassisOdometry.getYPosition();

        float dx = targetX - curX;
        float dy = targetY - curY;

        //Signed error along the original heading:
        float linearError  = dx * dirX + dy * dirY;
        float angularError = degTo180(startHeadingDeg - inertial1.heading());

        float linearOutput  = linearPID.compute(linearError);
        float angularOutput = angularPID.compute(angularError);

        linearOutput  = clamp(linearOutput,  -driveMaxVoltage, driveMaxVoltage);
        angularOutput = clamp(angularOutput, -driveMaxVoltage, driveMaxVoltage);

        driveMotors(linearOutput + angularOutput, linearOutput - angularOutput);

        updatePosition();
        wait(10, msec);
    }

    brake();
    driveMotors(0, 0);
    updatePosition();
}

/// @brief Drive a certain distance using the odometry wheels with a time limit
/// @param distance How far the robot (in inches) needs to drive
/// @param timeLimit The maximum time allowed to drive
void Drive::driveDistanceWithOdom(float distance, float timeLimit){
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);

    updatePosition();

    // --- Starting pose (field coordinates & heading) ---
    float startHeadingDeg = inertial1.heading();
    float startHeadingRad = degToRad(startHeadingDeg);

    // Unit forward direction based on starting heading
    float dirX = sin(startHeadingRad);
    float dirY = cos(startHeadingRad);

    // Starting position in field coordinates
    float startX = chassisOdometry.getXPosition();
    float startY = chassisOdometry.getYPosition();

    // Target point in field coordinates (distance along starting heading)
    float targetX = startX + dirX * distance;
    float targetY = startY + dirY * distance;

    while (!linearPID.isSettled())
    {
        updatePosition();

        // Odom-based pose
        float curX = chassisOdometry.getXPosition();
        float curY = chassisOdometry.getYPosition();

        float dx = targetX - curX;
        float dy = targetY - curY;

        // Signed error along the original heading:
        float linearError  = dx * dirX + dy * dirY;
        float angularError = degTo180(startHeadingDeg - inertial1.heading());

        float linearOutput  = linearPID.compute(linearError);
        float angularOutput = angularPID.compute(angularError);

        linearOutput  = clamp(linearOutput,  -driveMaxVoltage, driveMaxVoltage);
        angularOutput = clamp(angularOutput, -driveMaxVoltage, driveMaxVoltage);

        driveMotors(linearOutput + angularOutput, linearOutput - angularOutput);

        updatePosition();
        wait(10, msec);
    }

    // Make absolutely sure we stop
    brake();
    driveMotors(0, 0);
    updatePosition();
}

/// @brief Drive a certain distance using the odometry wheels with a time limit and max voltage
/// @param distance  How far the robot (in inches) needs to drive
/// @param timeLimit The maximum time allowed to drive
/// @param maxVoltage The maximum voltage the robot can run at
void Drive::driveDistanceWithOdom(float distance, float timeLimit, float maxVoltage){
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);

    updatePosition();

    // --- Starting pose (field coordinates & heading) ---
    float startHeadingDeg = inertial1.heading();
    float startHeadingRad = degToRad(startHeadingDeg);

    // Unit forward direction based on starting heading
    float dirX = sin(startHeadingRad);
    float dirY = cos(startHeadingRad);

    // Starting position in field coordinates
    float startX = chassisOdometry.getXPosition();
    float startY = chassisOdometry.getYPosition();

    // Target point in field coordinates (distance along starting heading)
    float targetX = startX + dirX * distance;
    float targetY = startY + dirY * distance;

    while (!linearPID.isSettled())
    {
        updatePosition();

        // Odom-based pose
        float curX = chassisOdometry.getXPosition();
        float curY = chassisOdometry.getYPosition();

        float dx = targetX - curX;
        float dy = targetY - curY;

        // Signed error along the original heading:
        float linearError  = dx * dirX + dy * dirY;
        float angularError = degTo180(startHeadingDeg - inertial1.heading());

        float linearOutput  = linearPID.compute(linearError);
        float angularOutput = angularPID.compute(angularError);

        linearOutput  = clamp(linearOutput,  -maxVoltage, maxVoltage);
        angularOutput = clamp(angularOutput, -maxVoltage, maxVoltage);

        driveMotors(linearOutput + angularOutput, linearOutput - angularOutput);

        updatePosition();
        wait(10, msec);
    }

    // Make absolutely sure we stop
    brake();
    driveMotors(0, 0);
    updatePosition();
}

/// @brief Drive a certain distance using the odometry wheels with a time limit, max voltage, settle time, and settle error
/// @param distance How far the robot (in inches) needs to drive
/// @param timeLimit The maximum time allowed to drive
/// @param maxVoltage The maximum voltage the robot can run at
/// @param settleTime How long the robot is allowed to settle
/// @param settleError How big the settle error is to allow the robot to settle
void Drive::driveDistanceWithOdom(float distance, float timeLimit, float maxVoltage, float settleTime, float settleError){
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);

    updatePosition();

    // --- Starting pose (field coordinates & heading) ---
    float startHeadingDeg = inertial1.heading();
    float startHeadingRad = degToRad(startHeadingDeg);

    // Unit forward direction based on starting heading
    float dirX = sin(startHeadingRad);
    float dirY = cos(startHeadingRad);

    // Starting position in field coordinates
    float startX = chassisOdometry.getXPosition();
    float startY = chassisOdometry.getYPosition();

    // Target point in field coordinates (distance along starting heading)
    float targetX = startX + dirX * distance;
    float targetY = startY + dirY * distance;

    while (!linearPID.isSettled())
    {
        updatePosition();

        // Odom-based pose
        float curX = chassisOdometry.getXPosition();
        float curY = chassisOdometry.getYPosition();

        float dx = targetX - curX;
        float dy = targetY - curY;

        // Signed error along the original heading:
        float linearError  = dx * dirX + dy * dirY;
        float angularError = degTo180(startHeadingDeg - inertial1.heading());

        float linearOutput  = linearPID.compute(linearError);
        float angularOutput = angularPID.compute(angularError);

        linearOutput  = clamp(linearOutput,  -maxVoltage, maxVoltage);
        angularOutput = clamp(angularOutput, -maxVoltage, maxVoltage);

        driveMotors(linearOutput + angularOutput, linearOutput - angularOutput);

        updatePosition();
        wait(10, msec);
    }

    // Make absolutely sure we stop
    brake();
    driveMotors(0, 0);
    updatePosition();
}


/* ============= */
/* OTHER DRIVING */
/* ============= */

/// @brief Turns sharply to a specific location and moves to it
/// @param desX Desired X position
/// @param desY Desired Y position
void Drive::moveToPosition(float desX, float desY){
    // Calculate the angle to turn to
    float deltaX = desX - chassisOdometry.getXPosition();
    float deltaY = desY - chassisOdometry.getYPosition();

    // Turn to the target angle
    turnToPosition(desX, desY);

    // Calculate the distance to the target position
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    // Drive the calculated distance
    driveDistanceWithOdom(distance);
}

/// @brief Turns along a set curve
/// @param curX The current X position of the robot
/// @param curY The current Y position of the robot
/// @param midX The X position of the middle point of the curve
/// @param midY The Y position of the middle point of the curve
/// @param desX The desired ending X position
/// @param desY The desired ending Y position
/// @param numPts The number of points along the curve to go to
void Drive::bezierTurn(float curX, float curY, float midX, float midY, float desX, float desY, int numPts){
    float* pts = new float[numPts+1];
    float nextX, nextY;
    
    //Populate the t-values (0-1)
    pts[numPts] = 1;
    for(int i=0;i<numPts;i++){
        pts[i] = (1.0/static_cast<float>(numPts+1)) * i;
    }

    //Calculate X and Y values along the curve based on pts[i] and move to that position
    for(int i=0;i<numPts+1;i++){
        nextX = (pow(1-pts[i], 2)*curX) + (2*(1-pts[i])*pts[i]*midX) + (pow(pts[i], 2)*desX);
        nextY = (pow(1-pts[i], 2)*curY) + (2*(1-pts[i])*pts[i]*midY) + (pow(pts[i], 2)*desY);  
        moveToPosition(nextX, nextY);
    }

    delete [] pts;
}

/// @brief Drive to a specific X and Y coordinate and ends at a target heading using "carrot" positions
/// @param x Target X position
/// @param y Target Y position
/// @param targetHeading Target heading position to end facing towards
void Drive::movetopos(float x, float y, float targetHeading) {
    // ===== Tunables =====
    const float lead = 0.15f;            // carrot distance factor
    const float closeRange = 10.0f;      // inches
    const float dt_ms = 10.0f;

    const float maxDrive = driveMaxVoltage;
    const float maxTurn  = turnMaxVoltage;
    const float minTurn  = 1.0f;

    const float settleDist = driveSettleError;
    const float settleAng  = turnSettleError;
    const int   settleTime = driveTimeToSettle;
    const int   timeout_ms = driveEndTime;

    PID drivePID(0.85, 0.0, 2.5, .5, 200, 2500);
    PID turnPID (0.22, 0.0, 1.5, .75,  200, 2500);

    auto sgn = [](float v) { return (v >= 0.0f) ? 1.0f : -1.0f; };

    bool close = false;
    int elapsed = 0;
    int settled = 0;

    while (elapsed < timeout_ms) {
        updatePosition();

        const float rx = chassisOdometry.getXPosition();
        const float ry = chassisOdometry.getYPosition();
        const float rh = chassisOdometry.getHeading(); // deg, 0° = +Y

        // ===== Vector to target =====
        const float dx = x - rx;
        const float dy = y - ry;
        const float dist = hypot(dx, dy);

        if (!close && dist < closeRange)
            close = true;

        // ===== Carrot point =====
        float carrotX = x;
        float carrotY = y;

        if (!close) {
            const float carrotDist = lead * dist;
            const float fx = sin(degToRad(targetHeading));
            const float fy = cos(degToRad(targetHeading));

            carrotX = x - fx * carrotDist;
            carrotY = y - fy * carrotDist;
        }

        // ===== Heading to carrot =====
        const float cdx = carrotX - rx;
        const float cdy = carrotY - ry;
        const float carrotHeading = atan2(cdx, cdy) * 180.0f / M_PI;

        const float travelErr =
            inTermsOfNegative180To180(carrotHeading - rh);
        const float finalErr =
            inTermsOfNegative180To180(targetHeading - rh);

        // ===== Lateral (drive) error =====
        float lateralError;
        if (!close) {
            // Signed distance only (prevents orbiting)
            lateralError = dist * sgn(cos(degToRad(travelErr)));
        } else {
            // True projection onto heading
            lateralError = dist * cos(degToRad(travelErr));

            // Prevent collapse near 90°
            if (fabs(lateralError) < 0.25f)
                lateralError = 0.25f * sgn(lateralError);
        }

        // ===== Angular error =====
        const float angularError = close ? finalErr : travelErr;

        // ===== Settle logic =====
        if (close) {
            const bool distOK = fabs(dist) < settleDist;
            const bool angOK  = fabs(angularError) < settleAng;

            if (distOK && angOK) settled += dt_ms;
            else settled = 0;

            if (settled >= settleTime){
                break;
                std::cout << "\nSettle\n";
            }
        }

        // ===== PID outputs =====
        float driveOut = drivePID.compute(lateralError);
        float turnOut  = turnPID.compute(angularError);

        driveOut = clamp(driveOut, -maxDrive, maxDrive);
        turnOut  = clamp(turnOut,  -maxTurn,  maxTurn);

        // Minimum turn only
        if (fabs(turnOut) > 0 && fabs(turnOut) < minTurn)
            turnOut = sgn(turnOut) * minTurn;

        // Minimum drive ONLY when far
        if (!close && fabs(driveOut) < 1.0f)
            driveOut = sgn(driveOut) * 1.0f;

        // ===== Mix =====
        const float left  = driveOut + turnOut;
        const float right = driveOut - turnOut;

        driveMotors(left, right);
        elapsed += dt_ms;
        if (elapsed > timeout_ms){
            std::cout << "\nTimeout\n";
            break;
        }

        vex::task::sleep(dt_ms);
    }

    brake();
}


/* =========== */
/* POSITIONING */
/* =========== */

/// @brief Gets the current position of the drive base
/// @return Returns the position in inches
float Drive::getCurrentMotorPosition()
{
    float leftPosition = degToInches(leftDrive.position(degrees), wheelDiameter);
    float rightPosition = degToInches(rightDrive.position(degrees), wheelDiameter);

    return (leftPosition + rightPosition) / 2;
}

// float Drive::getCurrentMotorPosition(){
//     float leftPosition = degToInches(getMotorEncoderPosition(LT1, LT2, LT3, LT4, LT5), wheelDiameter);
//     float rightPosition = degToInches(getMotorEncoderPosition(RT1, RT2, RT3, RT4, RT5), wheelDiameter);

//     return (leftPosition + rightPosition) / 2;
// }

/// @brief Sets brakes to coast and displays position location to the robot brain
void Drive::moveable(){
    //updates odom and printx x and y position
    while (true) {
        brake(coast);
        updatePosition();
        float x = chassisOdometry.getXPosition();
        float y = chassisOdometry.getYPosition();
        float heading = chassisOdometry.getHeading();
        // float x = rotation1.position(degrees);
        // float y = rotation2.position(degrees);
        // std::cout << "X: " << x << ", Y: " << y << std::endl;
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("X: ");
        Brain.Screen.print(x);
        Brain.Screen.newLine();
        Brain.Screen.print("Y: ");
        Brain.Screen.print(y);
        Brain.Screen.print(heading);
        //std::cout << "\nHeading: " << chassisOdometry.getHeading();
        //std::cout << "\nx: " << x;
        //std::cout << "\ny: " << y;
        wait(50, msec); 
    }
}

/// @brief Updates the position of the robot
void Drive::updatePosition(){
    switch(odomType){
        float left, right, heading;
        case NO_ODOM:
            left = leftDrive.position(degrees);
            right = rightDrive.position(degrees);
            heading = inertial1.heading();
            chassisOdometry.updatePositionTwoForward(right, left, heading);
            break;
        case HORIZONTAL_AND_VERTICAL:
            // left = rotation1.position(degrees);
            // right = rotation2.position(degrees);
            // heading = inertial1.heading();
            // chassisOdometry.updatePositionOneForward(left, right, heading);
            break;
        case TWO_VERTICAL:
            // left = rotation1.position(degrees);
            // right = rotation2.position(degrees);
            // heading = inertial1.heading();
            // chassisOdometry.updatePositionTwoForward(right, left, heading);
            break;
        case TWO_AT_45:
            // left = rotation1.position(degrees);
            // right = rotation2.position(degrees);
            // heading = inertial1.heading();
            // chassisOdometry.updatePositionTwoAt45(left, right, heading);
            break;
    }
}

/// @brief Sets the current location of the robot
/// @param x Current X location
/// @param y Current Y location
/// @param heading Current heading
void Drive::setPosition(float x, float y, float heading){
    // Reset odom pose
    chassisOdometry.setPosition(x, y, heading);
    inertial1.setHeading(heading, degrees);

    // Sync odom encoder baselines with the actual sensors
    switch (odomType) {
        case NO_ODOM:
            // Using drive motors as odom
            chassisOdometry.setForwardRightDegrees(rightDrive.position(degrees));
            chassisOdometry.setForwardLeftDegrees(leftDrive.position(degrees));
            chassisOdometry.setLateralDegrees(0);
            break;

        case HORIZONTAL_AND_VERTICAL:
            // Using rotation1 and rotation2
            // chassisOdometry.setForwardLeftDegrees(rotation1.position(degrees));
            // chassisOdometry.setLateralDegrees(rotation2.position(degrees));
            // If you have a second forward sensor, set it here too
            break;

        case TWO_AT_45:
            // Whatever sensors you're using in this mode
            // Example (if both are vertical tracking wheels):
            // chassisOdometry.setForwardRightDegrees(rotation1.position(degrees));
            // chassisOdometry.setForwardLeftDegrees(rotation2.position(degrees));
            chassisOdometry.setLateralDegrees(0);
            break;

        default:
            break;
    }
}