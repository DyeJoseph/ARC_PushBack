#include "Drive.h"

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

    // this->chassisOdometry = Odom(2, -1.0, -1.0);

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

void Drive::setDriveMaxVoltage(float maxVoltage)
{
    driveMaxVoltage = maxVoltage;
}

void Drive::setTurnMaxVoltage(float maxVoltage)
{
    turnMaxVoltage = maxVoltage;
}

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


void Drive::arcade()
{
    int leftY = 0;
    int rightX = 0;
    if(Controller1.Axis3.position(percent) >= 0)
        leftY = pow(Controller1.Axis3.position(percent),2)/100;
    else
        leftY = pow(Controller1.Axis3.position(percent),2)/-100;
    
    if(Controller1.Axis1.position(percent) >= 0)
        rightX = pow(Controller1.Axis1.position(percent),2)/100;
    else
        rightX = pow(Controller1.Axis1.position(percent),2)/-100;

    leftDrive.spin(forward, leftY+rightX, percent);
    rightDrive.spin(forward, leftY-rightX, percent);
}


void Drive::tank(){
    int leftY = 0;
    int rightX = 0;
    if(Controller1.Axis3.position(percent) >= 0)
        leftY = pow(Controller1.Axis3.position(percent),2)/100;
        //leftY = Controller1.Axis3.position(percent);
    else
        leftY = pow(Controller1.Axis3.position(percent),2)/-100;
        //leftY = Controller1.Axis3.position(percent);
    
    if(Controller1.Axis2.position(percent) >= 0)
        rightX = pow(Controller1.Axis2.position(percent),2)/100;
        //rightX = Controller1.Axis2.position(percent);
    else
        rightX = pow(Controller1.Axis2.position(percent),2)/-100;
        //rightX = Controller1.Axis2.position(percent);

    leftDrive.spin(forward, leftY, percent);
    rightDrive.spin(forward, rightX, percent);
}

/// @brief Gets the current position of the drive base
/// @return Returns the position in inches
float Drive::getCurrentMotorPosition()
{
    float leftPosition = degToInches(leftDrive.position(degrees), wheelDiameter);
    float rightPosition = degToInches(rightDrive.position(degrees), wheelDiameter);

    return (leftPosition + rightPosition) / 2;
}

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
    //float Kp, float Ki, float Kd, float settleError, float timeToSettle, float endTime
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, driveEndTime);
    PID angularPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, turnEndTime);
    
    updatePosition();
    // Sets the starting variables for the Position and Heading
    float startPosition = getCurrentMotorPosition();
    float startHeading = inertial1.heading();

    // Updates the distance to match the current position of the robot
    distance += startPosition;

    //  Loops while the linear PID has not yet settled
    while(!linearPID.isSettled())
    {
        updatePosition();
        // Updates the Error for the linear values and the angular values
        float linearError = distance - getCurrentMotorPosition();
        float angularError = degTo180(startHeading - inertial1.heading());

        // Sets the linear output and angular output to the output of the error passed through the PID compute functions
        float linearOutput = linearPID.compute(linearError);
        float angularOutput = angularPID.compute(angularError);

        // Clamps the values of the output to fit within the -12 to 12 volt limit of the vex motors
        linearOutput = clamp(linearOutput, -maxVoltage, maxVoltage);
        angularOutput = clamp(angularOutput, -maxVoltage, maxVoltage);

        // Drives motors according to the linear Output and includes the linear Output to keep the robot in a straight path relative to is start heading
        driveMotors(linearOutput + angularOutput, linearOutput - angularOutput);
        wait(10, msec);
    }

    
    // Stops the motors once PID has settled
    //brake();
    updatePosition();
}

/// @brief Turns the robot a set amount of degrees
/// @param turnDegrees A number in degrees the robot should rotate
void Drive::turn(float turnDegrees){
    turnToAngle(turnDegrees + inertial1.heading());
}

/// @brief Turns the robot a set amount of degrees
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

/// @brief Turns to an absolute specific angle
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

        //Minimum output threshold for turning
        if(fabs(output) < 1.55)
            if(output < 0)
                output = -1.55;
            else
                output = 1.55;
        else
            output = clamp(output, -maxVoltage, maxVoltage);

        driveMotors(-output, output);
        task::sleep(10);
    }while(!turnPID.isSettled());
    brake();
    updatePosition();
}

void Drive::turnToAngleD(float angle, float maxVoltage, float turnKdUpdate)
{
    updatePosition();
    angle = inTermsOfNegative180To180(angle);
    PID turnPID(turnKp, turnKi, turnKdUpdate, turnSettleError, turnTimeToSettle, turnEndTime);
    do
    {
        float error = inTermsOfNegative180To180(inertial1.heading()-angle);
        float output = turnPID.compute(error);

        //Minimum output threshold for turning
        if(fabs(output) < 2)
            if(output < 0)
                output = -2.5;
            else
                output = 2.5;
        else
            output = clamp(output, -maxVoltage, maxVoltage);

        driveMotors(-output, output);
        task::sleep(10);
    }while(!turnPID.isSettled());
    brake();
    updatePosition();
}

void Drive::turnToAngleTime(float angle, float timeLimit, float maxVoltage)
{
    updatePosition();
    angle = inTermsOfNegative180To180(angle);
    PID turnPID(turnKp, turnKi, turnKd, turnSettleError, turnTimeToSettle, timeLimit);
    do
    {
        float error = inTermsOfNegative180To180(inertial1.heading()-angle);
        float output = turnPID.compute(error);

        //Minimum output threshold for turning
        if(fabs(output) < 2)
            if(output < 0)
                output = -2.5;
            else
                output = 2.5;
        else
            output = clamp(output, -maxVoltage, maxVoltage);

        driveMotors(-output, output);
        task::sleep(10);
    }while(!turnPID.isSettled());
    brake();
    updatePosition();
}

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



void Drive::driveDistanceWithOdom(float distance){
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

void Drive::driveDistanceWithOdomSettle(float distance, float settleTime, float settleError){
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, settleError, settleTime, driveEndTime);
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

void Drive::driveDistanceWithOdomTime(float distance, float timeLimit){
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, timeLimit);
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


void Drive::driveDistanceWithOdomTime(float distance, float timeLimit, float maxVoltage){
    // Creates PID objects for linear and angular output
    PID linearPID(driveKp, driveKi, driveKd, driveSettleError, driveTimeToSettle, timeLimit);
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



void Drive::turnToPosition(float desX, float desY){
    updatePosition();
    float deltaX = desX-chassisOdometry.getXPosition();
    float deltaY = desY-chassisOdometry.getYPosition();
    float angle = atan2(deltaX, deltaY) * (180.0/M_PI);
    turnToAngle(angle);
    updatePosition();
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

// void Drive::setPosition(float x, float y, float heading){
//     chassisOdometry.setPosition(x, y, heading);
// }


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
            chassisOdometry.setForwardLeftDegrees(rotation1.position(degrees));
            chassisOdometry.setLateralDegrees(rotation2.position(degrees));
            // If you have a second forward sensor, set it here too
            break;

        case TWO_AT_45:
            // Whatever sensors you're using in this mode
            // Example (if both are vertical tracking wheels):
            chassisOdometry.setForwardRightDegrees(rotation1.position(degrees));
            chassisOdometry.setForwardLeftDegrees(rotation2.position(degrees));
            chassisOdometry.setLateralDegrees(0);
            break;

        default:
            break;
    }
}

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

    PID drivePID(0.7, 0.0001, 1.7, settleDist, settleTime, timeout_ms);
    PID turnPID (0.3, 0.0001, 1.5, settleAng,  settleTime, timeout_ms);

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
        std::cout << "\ncdx: " << cdx << " cdy: " << cdy;
        std::cout << "\ndx: " << dx << " dy: " << dy << ", Dist: " << dist;
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
/*
void Drive::movetopos(float x, float y, float targetHeading) {
    // ===== Tunables =====
    const float approachOffset = 7.0f;          // inches behind target
    const float offsetRadius   = 3.0f;
    const float finalRadius    = driveSettleError;
    const float headingTol     = turnSettleError;

    const float turnOnlyAngle  = 100.0f;        // deg

    const float dt_ms = 10.0f;
    const int   timeout_ms = driveEndTime;

    const float maxDrive = driveMaxVoltage;
    const float maxTurn  = turnMaxVoltage;
    const float minTurn  = 1.0f;

    PID drivePID(0.7, 0.0, 1.4);
    PID turnPID (0.35, 0.0, 1.6);

    auto sgn = [](float v) { return (v >= 0.0f) ? 1.0f : -1.0f; };

    int elapsed = 0;

    while (elapsed < timeout_ms) {
        updatePosition();

        const float rx = chassisOdometry.getXPosition();
        const float ry = chassisOdometry.getYPosition();
        const float rh = chassisOdometry.getHeading(); // deg, 0° = +Y

        // ===== Offset point =====
        const float fx = sin(degToRad(targetHeading));
        const float fy = cos(degToRad(targetHeading));

        const float ox = x - fx * approachOffset;
        const float oy = y - fy * approachOffset;

        // ===== Errors =====
        const float dxO = ox - rx;
        const float dyO = oy - ry;
        const float distToOffset = hypot(dxO, dyO);

        const float dxF = x - rx;
        const float dyF = y - ry;
        const float distToFinal = hypot(dxF, dyF);

        const float offsetHeading =
            atan2(dxO, dyO) * 180.0f / M_PI;

        const float offsetHeadingErr =
            inTermsOfNegative180To180(offsetHeading - rh);

        const float finalHeadingErr =
            inTermsOfNegative180To180(targetHeading - rh);

        // ===== SETTLE =====
        if (distToFinal < finalRadius &&
            fabs(finalHeadingErr) < headingTol) {

            std::cout << "\n[MoveToPos] Settled"
                      << " | Dist: " << distToFinal
                      << " | HeadErr: " << finalHeadingErr
                      << " | Time: " << elapsed << "ms";

            driveMotors(0, 0);
            break;
        }

        float driveOut = 0;
        float turnOut  = 0;

        // ===== Final heading lock =====
        if (distToOffset < offsetRadius &&
            fabs(finalHeadingErr) > headingTol) {

            turnOut = turnPID.compute(finalHeadingErr);
        }

        // ===== Turn-only if backwards =====
        else if (distToOffset > offsetRadius &&
                 fabs(offsetHeadingErr) > turnOnlyAngle) {

            turnOut = turnPID.compute(offsetHeadingErr);
        }

        // ===== Final precision drive =====
        else if (distToOffset < offsetRadius &&
                 fabs(finalHeadingErr) < headingTol) {

            float forward = dxF * fx + dyF * fy;

            driveOut = drivePID.compute(forward) * 0.6f;
            turnOut  = turnPID.compute(finalHeadingErr) * 0.6f;
        }

        // ===== Main drive to offset =====
        else {
            float lateralError =
                distToOffset * cos(degToRad(offsetHeadingErr));

            driveOut = drivePID.compute(lateralError);
            turnOut  = turnPID.compute(offsetHeadingErr);
        }

        // ===== Clamp =====
        driveOut = clamp(driveOut, -maxDrive, maxDrive);
        turnOut  = clamp(turnOut,  -maxTurn,  maxTurn);

        if (fabs(turnOut) > 0 && fabs(turnOut) < minTurn)
            turnOut = sgn(turnOut) * minTurn;

        // ===== Mix =====
        const float left  = driveOut + turnOut;
        const float right = driveOut - turnOut;

        driveMotors(left, right);

        elapsed += dt_ms;
        vex::task::sleep(dt_ms);
    }

    // ===== TIMEOUT PRINT =====
    if (elapsed >= timeout_ms) {
        std::cout << "\n[MoveToPos] TIMEOUT"
                  << " | Target(" << x << ", " << y << ")"
                  << " | FinalHeading: " << targetHeading;
    }

    brake();
}*/

void Drive::moveToTarget(float x, float y, float targetHeading){
    const float approachOffset = 10.0f; //How far away the robot goes from the final point to align
    const float preciseOffset =1.0f; //When the robot switches to precise mode

    const float settleDist = driveSettleError; //Drive error
    const float settleAng  = turnSettleError; //Turn error

    const float turnOnlyAngle = 35.0f; //If robot is more than ___ deg away from angle it will turn first, prevent backward arching

    const float dt_ms = 10.0f;                      //Loop Timing
    const int   settleTime = driveTimeToSettle;      //Settle time
    const int timeout_ms = driveEndTime;            //Timeout Time

    const float maxDrive = driveMaxVoltage;         //Max Drive Volt
    const float maxTurn  = turnMaxVoltage;          //Max Turn Volt
    const float minTurn  = 1.0f;                    //Min Turn Volt
    const float minDrive = 1.0f;                    //Min Drive Volt

    bool precise = false;

    auto sgn = [](float v) { return (v >= 0.0f) ? 1.0f : -1.0f; }; //Returns if a number is positive or negative)

    PID drivePID(0.7, 0.0001, 1.7, 3, settleTime, 5000);
    PID turnPID (0.3, 0.0001, 1.5, settleAng,  settleTime, 5000);

    int elapsed = 0;
    //Main Loop
    while(elapsed < timeout_ms){
        updatePosition();

        const float currX = chassisOdometry.getXPosition();
        const float currY = chassisOdometry.getYPosition();
        const float currHeading = chassisOdometry.getHeading();

        const float offsetX = x;
        const float offsetY = y;
        //offset
        if (!precise){
            const float finalX = sin(degToRad(targetHeading));              //calculates where the offset position is
            const float finalY = cos(degToRad(targetHeading));
            const float offsetX = x - finalX * approachOffset;
            const float offsetY = y - finalY * approachOffset;
        }

        //error
        const float distOffsetX = offsetX - currX;                      //calculates distance to offset position
        const float distOffsetY = offsetY - currY;
        const float distToOffset = hypot(distOffsetX, distOffsetY);

        const float distFinalX = x - currX;                             //calculates distance to final position
        const float distFinalY = y - currY;
        const float distToFinal = hypot(distFinalX, distFinalY);

        const float offsetHeading = atan2(distOffsetX, distOffsetY) * 180.0f / M_PI;            //General direction a robot must go        
        const float offsetHeadingErr = inTermsOfNegative180To180(offsetHeading - currHeading);  //Shortest path to turn robot to offset
        const float finalHeadingErr = inTermsOfNegative180To180(targetHeading - currHeading);   //Turns the robot to the final heading when it gets close

        if (!precise && distToOffset < preciseOffset && fabs(offsetHeadingErr) < .75f) {
            precise = true;
            std::cout << "\n\nSWITCH\n\n";
        }

        float driveOut = 0;
        float turnOut  = 0;

        if (distToFinal < settleDist && fabs(finalHeadingErr) < settleAng) {
                std::cout << "MoveToTarget Settled"
                        << " , Dist: " << distToFinal
                        << " , HeadErr: " << finalHeadingErr
                        << " , Time: " << elapsed << "ms" << std::endl;
                driveMotors(0, 0);
                break;
            }

        // Precise Mode (When the distance left is less than precise offset and heading is less than settle angle error)
        if (precise){

            float forward = distFinalX * cos(degToRad(currHeading)) + distFinalY * sin(degToRad(currHeading));
            

            driveOut = drivePID.compute(forward) * 0.6f;    //Drop drive and turn to 60% power to be "precise"
            turnOut  = turnPID.compute(finalHeadingErr) * 0.6f;

        }else{
            
            //Final Heading Set (When Distance left is less than our precise value, and my heading is greater than my settle error)
            if (distToOffset < preciseOffset && fabs(finalHeadingErr) > settleAng) {
                driveOut = 0;
                turnOut = turnPID.compute(finalHeadingErr);
            }

            // Turn Only (If distance left is greater than my precise offset distance and offset heading is greater than the turnonly value you gave)
            else if (distToOffset > preciseOffset && fabs(offsetHeadingErr) > turnOnlyAngle) {
                turnOut = turnPID.compute(offsetHeadingErr);
            }
            else{
                // Main Drive (If no other if statements catch)
            float forwardError = distToOffset * cos(degToRad(offsetHeadingErr));    //adjust hows much the drive power is based on angle of robot
            //float speedScale = clamp(distToOffset / 18.0f, 0.25f, 1.0f);
            driveOut = drivePID.compute(forwardError);
            turnOut  = turnPID.compute(offsetHeadingErr);
            }

        }


        //Clamping 
        driveOut = clamp(driveOut, -maxDrive, maxDrive);
        turnOut  = clamp(turnOut,  -maxTurn,  maxTurn);
        if (fabs(driveOut) > 0 && fabs(driveOut) < minDrive)
            driveOut = sgn(driveOut) * minDrive;
        if (fabs(turnOut) > 0 && fabs(turnOut) < minTurn)       //Prevents deadzone of too low voltage
            turnOut = sgn(turnOut) * minTurn;

        // ===== Mix =====
        const float left  = driveOut + turnOut;                 //Adjusts how robot will move, left, right, straight...
        const float right = driveOut - turnOut;

        driveMotors(left, right);                               //Actually drives the motors of the robot
        std::cout << "\nValues "
                  << " , CurrPos(" << chassisOdometry.getXPosition() << ", " << chassisOdometry.getYPosition() << ")"
                  << " , Target: " << distToFinal
                  << " , DriveVolts(" << driveOut << ", " << turnOut << ")"
                  << " , Heading: " << chassisOdometry.getHeading()
                  << " , Precise: " << precise
                  << " , distToOffset: " << distToOffset
                << " , CurrHeading: " << currHeading
                << " , Err: " << offsetHeadingErr << "\n";

            //timeout
        if (elapsed >= timeout_ms) {
            std::cout << "\nMoveToTarget TIMEOUT"
                    << " , Target(" << x << ", " << y << ")"
                    << " , Actual(" << chassisOdometry.getXPosition() << ", " << chassisOdometry.getYPosition() << ")"
                    << " , FinalHeading: " << targetHeading
                    << " , ActualHeading: " << chassisOdometry.getHeading();

        }
        elapsed += dt_ms;                                       //Adds to elapsed and waits dt_ms to run again
        vex::task::sleep(dt_ms);

    }
    brake();
}