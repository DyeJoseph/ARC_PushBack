/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Organization:       Autonomous Robotics Club (ARC)                      */
/*    Authors:            Coby Smith and Joseph Dye                           */
/*    Created:            9/9/2024                                            */
/*    Description:        ARC Template                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "screen.h"
#include "util.h"
#include "Drive.h"
#include "autoPIDTuner.h"
#include "semiPIDTuner.h"
#include "images.h"


using namespace vex;

////////////////////////// GLOBAL VARIABLES //////////////////////////

  // Competition Instance
  competition Competition;

  int odomType = TWO_AT_45;

  bool isColorSorting = false;
  bool odomDebugEnabled = true;

  bool isInAuton = false;
  int lastPressed = 0;
  int teamColor = 0; //red = 0, blue = 1
  int driver = 0; //Elliot = 0, Jacob = 1

  // Define Values for the Chassis here:
  Drive chassis
  (
    motor_group(LFT, LFB, LBB, LBT), // Left drive train motors
    motor_group(RFT, RFB, RBB, RBT), // Right drive train motors
    PORT20,               // Inertial Sensor Port
    2.66,              // The diameter size of the wheel in inches 2.66
    1,                   // 
    12,                   // The maximum amount of the voltage used in the drivebase (1 - 12)
    odomType,
    1.955,                  //Odometry wheel diameter (set to zero if no odom) (1.96 robot behind by .2)
    -1.280,               //Odom pod1 offset -3.867
    -1.280                //Odom pod2 offset -3.867
  );

//////////////////////////////////////////////////////////////////////

///////////////////////// Prototypes /////////////////////////////////

void setDriveTrainConstants();
void Auton_1();
void Auton_2();
void Auton_3();
void Auton_4();
void Auton_5();
void Auton_6();
void Auton_7();
void Auton_8();
int odomDebugThread();
void PAutonTest();
void autoPIDTest();
void semiPIDTest();
void runStepResponseTest(double voltage, int runID);

void toggleLift();
void toggleIntakeFlap();
void slowIntake();
void toggleColorSort();
void toggleDropDown();

//////////////////////////////////////////////////////////////////////


/// @brief Runs before the competition starts
void preAuton() 
{
  setDriveTrainConstants();

  chassis.brake(coast);       // make sure they aren’t holding weirdly
  chassis.driveMotors(0, 0);  

  enum preAutonStates{START_SCREEN = 0, SELECTION_SCREEN = 1};
  int currentScreen = START_SCREEN;

  // Calibrates/Resets the Brains sensors before the competition
  inertial1.calibrate();
  rotation1.resetPosition();
  rotation2.resetPosition();

  vex::color colors[8] = {vex::color::red, vex::color::red, vex::color::red, vex::color::red, 
                          vex::color::blue, vex::color::blue, vex::color::blue, vex::color::blue};
  std::string names[8] = {"NONE", "UCF", "1mSkill", "TopDef3", 
                          "Park5", "2xTop7", "TopMid7", "LowMid7"};
  Button buttons[9];
  createAutonButtons(colors, names, buttons);
  buttons[0].setChosen(true);

  Text selectionLabel;
  Text configLabel;
  Button startScreenButtons[5];
  createPreAutonScreen(startScreenButtons, selectionLabel, configLabel);
  
  int temp;

  Controller1.Screen.print(buttons[lastPressed].getName().c_str());

  while(!isInAuton){
    showPreAutonScreen(startScreenButtons, selectionLabel, configLabel, buttons[lastPressed].getName(), teamColor, driver);
    while(currentScreen == START_SCREEN){
      if(Brain.Screen.pressing()){
        if(checkPreAutonButtons(startScreenButtons, teamColor, driver, configLabel)){
          currentScreen = SELECTION_SCREEN;
        }
        Controller1.Screen.clearLine();
        Controller1.Screen.setCursor(1, 1);
        std::string colorString = teamColor ? "Blue" : "Red";
        std::string driverString = driver ? "Jacob" : "Elliot";
        std::string controllerPrint = buttons[lastPressed].getName() + " - " + colorString + " - " + driverString;
        Controller1.Screen.print(controllerPrint.c_str());
      }
      wait(10, msec);
    }

    showAutonSelectionScreen(buttons);
    while(currentScreen == SELECTION_SCREEN){
      if(Brain.Screen.pressing()){
        temp = checkButtonsPress(buttons);
        if(temp >= 0 && temp < 8){
          lastPressed = temp;
          Controller1.Screen.clearLine();
          Controller1.Screen.setCursor(1, 1);
          std::string colorString = teamColor ? "Blue" : "Red";
          std::string driverString = driver ? "Jacob" : "Elliot";
          std::string controllerPrint = buttons[lastPressed].getName() + " - " + colorString + " - " + driverString;
          Controller1.Screen.print(controllerPrint.c_str());
        }
      }
      if(temp == 8)
        currentScreen = START_SCREEN;
      wait(10, msec);
    }
    wait(10, msec);
  }
}

/// @brief Runs during the Autonomous Section of the Competition
void autonomous() 
{  
  drawSponsors();
  isInAuton = true;
  rotation1.resetPosition();
  rotation2.resetPosition();
  inertial1.resetHeading();
  wait(100, msec);

  // setDriveTrainConstants();

  // chassis.setPosition(0, 0, 0);
  // chassis.moveable();

  // switch (lastPressed) 
  // {
  //   case 0:
  //     Auton_1();
  //     break;
  //   case 1:
  //     Auton_2();
  //     break;
  //   case 2:
  //     Auton_3();
  //     break;
  //   case 3:
  //     Auton_4();
  //     break;
  //   case 4:
  //     Auton_5();
  //     break;
  //   case 5:
  //     Auton_6();
  //     break;
  //   case 6:
  //     Auton_7();
  //     break;
  //   case 7:
  //     Auton_8();
  //     break;
  //   default:
  //     break;
  // }

  //autoPIDTest();
    // runStepResponseTest(12.0, 1);
    // vex::wait(5000, vex::msec); // let it settle between runs
    // runStepResponseTest(8.0,  2);
    // vex::wait(5000, vex::msec);
    // runStepResponseTest(4.0,  3);
}

/// @brief Runs during the UserControl section of the competition
void usercontrol() 
{
  semiPIDTest();
  drawSponsors();

  // User control code here, inside the loop
  bool flapState = false;
  int lastSeen = teamColor;
  

  chassis.brake(coast);
  mainIntake.setStopping(coast);

  mainIntake.setVelocity(85, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);

  Controller1.ButtonL1.pressed(toggleLift);
  // Controller1.ButtonUp.pressed(toggleIntakeFlap);
  // Controller1.ButtonDown.pressed(slowIntake);
  Controller1.ButtonRight.pressed(toggleColorSort);
  Controller1.ButtonL2.pressed(toggleDropDown);
  Controller1.ButtonL2.released(toggleDropDown);

  //For Skills Auton

  bottomColorSort.setLight(ledState::on);
  bottomColorSort.integrationTime(20);
  while (1) {

      if(driver)
        chassis.tank();
      else
        chassis.arcade();

      if(bottomColorSort.color() == vex::color::red){
        lastSeen = 0;
      }else if(bottomColorSort.color() == vex::color::blue){
        lastSeen = 1;
      }

      if(Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward, 50, percent);
        flapState = false;
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else{
          colorSort.spin(reverse);
        }
      }else if(Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
        mainIntake.spin(reverse);
        topStage.spin(reverse);
        colorSort.spin(forward, 25, percent);
      }else if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward);
        flapState = true;
        colorSort.spin(fwd);
      }else if(Controller1.ButtonDown.pressing()){
        mainIntake.spin(reverse, 35, percent);
        topStage.spin(reverse);
        colorSort.spin(forward, 20, percent);
      }else if(Controller1.ButtonUp.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward, 35, percent);
        flapState = true;
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else{
          colorSort.spin(reverse);
        }
      }else{
        mainIntake.stop();
        colorSort.stop();
        topStage.stop();
      }
      if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonDown.pressing() && !Controller1.ButtonUp.pressing()){
        flapState = false;
      }

      if(Controller1.ButtonA.pressing()){
        matchLoad.set(true);
        mainIntake.spin(forward);
        topStage.spin(forward, 50, percent);
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else
          colorSort.spin(reverse);
      }else{
        matchLoad.set(false);
        if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonDown.pressing() && !Controller1.ButtonR2.pressing() && !Controller1.ButtonUp.pressing()){
          mainIntake.stop();
          colorSort.stop();
          topStage.stop();
        }
      }
      intakeFlap.set(flapState);
    wait(20, msec);
  }
    
}

void toggleLift(){
  static bool liftState = false;
  liftState = !liftState;
  intakeLift.set(liftState);
}

void toggleIntakeFlap(){
  static bool staticFlap = false;
  staticFlap = !staticFlap;
  intakeFlap.set(staticFlap);
}

void toggleDropDown(){

  static bool staticDrop = false;
  staticDrop = !staticDrop;
  dropDown.set(staticDrop);

}

void slowIntake(){
  static bool isSlowed = false;
  isSlowed = !isSlowed;
  if(isSlowed){
    mainIntake.setVelocity(50, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(50, percent);
  }else{
    mainIntake.setVelocity(85, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
  }
}

void toggleColorSort(){
  isColorSorting = !isColorSorting;
}

int main() 
{

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  preAuton();

  // Prevent main from exiting with an infinite loop.
  while (true) 
  {
    wait(100, msec);
  }
}

/// @brief Sets the PID values for the DriveTrain
void setDriveTrainConstants()
{
    // Set the Drive PID values for the DriveTrain
    chassis.setDriveConstants(
        .9,  // Kp - Proportion Constant
        0.0, // Ki - Integral Constant
        2.5, // Kd - Derivative Constant
        .5, // Settle Error
        200, // Time to Settle
        2500 // End Time 5000
    );  

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        0.22,    // Kp - Proportion Constant
        0.0,      // Ki - Integral Constant
        1.5,      // Kd - Derivative Constant 
        .75, //1.25    // Settle Error
        200,    // Time to Settle
        2500    // End Time
    );
    
}

//Auton Route Functions
/// @brief Auton Slot 1 - Write code for route within this function.
void Auton_1() //EMPTY (UPDATE WHEN CHANGED)
{   

}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2() // UCF Route
{

}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3() //1 MINUTE SKI
{   

}

void Auton_4()
{

}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5() //PARK
{

}

/// @brief Auton Slot 6 - Write code for route within this function.
void Auton_6() //DOUBLE LOAD TOP
{
 
}

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7() //SCORES TOP MIDDLE REAL
{
}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8() //SCORES LOW MIDDLE REAL
{

}

int odomDebugThread() {
  while (odomDebugEnabled) {
    std::cout << "X POS: " << chassis.chassisOdometry.getXPosition()
              << " Y POS: " << chassis.chassisOdometry.getYPosition()
              << " HEADING: " << chassis.chassisOdometry.getHeading()
              << std::endl;

    vex::this_thread::sleep_for(100);
  }
  return 0;
}

/*
  too low p = undershoot
*/

/// @brief Moves the robot in straight lines along both X and Y axis recording error alongside a changing P value
void PAutonTest(){
  std::string filename = "PID_Tuning.csv";
  std::string headerName = "";
  float pValues[] ={0.1, 0.2, 0.4, 0.6, 0.8, 1.0};
  int numPValues = 6;

  chassis.setPosition(0,0,0);
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(10);

  //Set PID Values
  for(int k=0; k<numPValues;k++){
    chassis.setPosition(0,0,0);
    std::cout << "p: " << pValues[k] << std::endl;
    
    chassis.setDriveConstants(pValues[k], 0, 0, 0.75, 200, 2500);
    chassis.setTurnConstants(0.25, 0, 0, 2, 200, 2500);

    headerName = "P:";
    writeToCard(filename, headerName);
    writeToCard(filename, pValues[k]);
    headerName = "; I:0; D:0";
    writeToCard(filename, headerName);
    
    writeNewLineToCard(filename);

    for(int i=0;i<20;i++){
      //Test Y
      std::cout << "i1: " << i << std::endl;
      chassis.turnToAngle(0);
      chassis.setPosition(0,0,0);
      chassis.driveDistanceWithOdom(24);
      writeToCard(filename,chassis.chassisOdometry.getYPosition()-24);
      writeCommaToCard(filename);
      chassis.driveDistanceWithOdom(-24);
    }
    writeNewLineToCard(filename);
    for(int i=0;i<20;i++){
      //Turn and test X
      std::cout << "i2: " << i << std::endl;
      chassis.turnToAngle(90);
      chassis.setPosition(0,0,90);
      chassis.driveDistanceWithOdom(24);
      writeToCard(filename,chassis.chassisOdometry.getXPosition()-24);
      writeCommaToCard(filename);
      chassis.driveDistanceWithOdom(-24);
    }

    chassis.turnToAngle(0);
    writeNewLineToCard(filename);

  }
  
}

/// @brief Automatically computes the best P value given a certain distance
/// @param distance distance to cover
/*void PAutonGenerator(float distance){
  //P value
  float pValue = 0.1;
  
  //PID constants
  float iValue = 0.0;
  float dValue = 0.0;
  float settleError = 0.75;
  float timeToSettle = 200;
  float endTime = 2500;

  //Storage
  std::vector<float> yErrors;
  std::vector<float> xErrors;
  float yAvgError;
  float xAvgError;
  int crossCountY = 0;
  int crossCountX = 0;
  float prevError = 0.0;
  bool hasPrevError = false;

  //Number of times for loops to iterate
  int numIterations = 10;

  //Ratios/Conditionals for P
  float incrementRatio = 1.5;
  float decrementRatio = 0.8;
  float acceptableError = 0.01 * distance;

  //SD Card variables
  std::string filename = "PTest.csv";
  std::ostringstream oss;

  float actualError = acceptableError+1; //Set higher than acceptableError so while loop will initally run

  chassis.setPosition(0, 0, 0);
  
  while(fabs(actualError) > acceptableError){
    chassis.setDriveConstants(pValue, iValue, dValue, settleError, timeToSettle, endTime);

    oss << "Current P: " << pValue;
    writeToCard(filename, oss.str());
    oss.str("");
    oss.clear();
    writeNewLineToCard(filename);

    //Test on Y-axis
    for(int i=0; i<numIterations;i++){
      chassis.turnToAngle(0);
      chassis.setPosition(0, 0, 0);
      chassis.driveDistanceWithOdom(distance);
      writeToCard(filename, chassis.chassisOdometry.getYPosition()-distance);
      writeCommaToCard(filename);
      yErrors.push_back(chassis.chassisOdometry.getYPosition()-distance);
      if(hasPrevError && prevError * yErrors.at(yErrors.size()-1) < 0){
        crossCountY++;
      }
      prevError = yErrors.at(yErrors.size()-1);
      hasPrevError = true;

      chassis.driveDistanceWithOdom(-distance);
    }
    writeNewLineToCard(filename);

    //Average errors
    yAvgError = 0.0;
    for(int i=0;i<yErrors.size();i++){
      yAvgError += fabs(yErrors.at(i));
    }
    yAvgError /= yErrors.size();

    oss << "AVG ERR FOR P=" << pValue << " ON Y-AXIS: " << yAvgError;
    writeToCard(filename, oss.str());
    oss.str("");
    oss.clear();
    writeNewLineToCard(filename);

    //-----------------------------------------------------------//

    hasPrevError = false;
    prevError = 0.0;
    //Test on X-axis
    for(int i=0; i<numIterations;i++){
      chassis.turnToAngle(90);
      chassis.setPosition(0, 0, 90);
      chassis.driveDistanceWithOdom(distance);
      writeToCard(filename, chassis.chassisOdometry.getXPosition()-distance);
      writeCommaToCard(filename);
      xErrors.push_back(chassis.chassisOdometry.getXPosition()-distance);
      if(hasPrevError && prevError * xErrors.at(xErrors.size()-1) < 0){
        crossCountX++;
      }
      prevError = xErrors.at(xErrors.size()-1);
      hasPrevError = true;

      chassis.driveDistanceWithOdom(-distance);
    }
    writeNewLineToCard(filename);

    //Average errors
    xAvgError = 0.0;
    for(int i=0;i<xErrors.size();i++){
      xAvgError += fabs(xErrors.at(i));
    }
    xAvgError /= xErrors.size();

    oss << "AVG ERR FOR P=" << pValue << " ON X-AXIS: " << xAvgError;
    writeToCard(filename, oss.str());
    oss.str("");
    oss.clear();
    writeNewLineToCard(filename);

    //If avgError is more than the acceptable, decrease P
    if(yAvgError > acceptableError && xAvgError > acceptableError){
      if(crossCountY > numIterations/3 && crossCountX > numIterations/3){
        std::cout << "YAVG: " << yAvgError << ", XAVG: " << xAvgError << "; DECREASING P" << std::endl;
        pValue *= decrementRatio;
      }else{
        std::cout << "YAVG: " << yAvgError << ", XAVG: " << xAvgError << "; INCREASING P" << std::endl;
        pValue *= incrementRatio;
      }
    }

    yErrors.clear();
    xErrors.clear();
    crossCountY = 0;
    crossCountX = 0;
    hasPrevError = false;
    prevError = 0.0;

    actualError = std::max(yAvgError, xAvgError);
  }
}*/

void autoPIDTest(){
  chassis.setPosition(0,0,0);
  std::cout << "Auto tuner" << std::endl;

  tunerConfig cfg;
  cfg.stepVoltage      = 3.0;    // lower if you have limited space
  cfg.stepDuration     = 2500;   // ms per run
  cfg.sampleIntervalMs = 10;     // ms between samples
  cfg.maxDistanceIn    = 30.0;   // safety cutoff
  cfg.method           = "ITAE"; // "ITAE" = smooth, "ZN" = aggressive
  AutoPIDTuner tuner(chassis, cfg);
  tuner.tuneDrive();
}

void semiPIDTest(){
  PIDTuner tuner(chassis);
  tuner.run();
}

void runStepResponseTest(double voltage, int runID) {
    std::ofstream logFile;
    logFile.open("pid_log2.csv", std::ios::app); // append mode

    // Write run header
    logFile << "=== RUN " << runID << " | " << voltage << "V ===\n";
    logFile << "timestamp_ms,voltage,left_velocity,right_velocity\n";

    // Reset motors and wait for settle
    leftDrive.stop(vex::brakeType::hold);
    rightDrive.stop(vex::brakeType::hold);
    vex::wait(500, vex::msec);

    double startTime = Brain.Timer.time(vex::msec);
    double duration  = 2000; // run for 2 seconds

    while (Brain.Timer.time(vex::msec) - startTime < duration) {
        double t    = Brain.Timer.time(vex::msec) - startTime;
        double lVel = leftDrive.velocity(vex::velocityUnits::rpm);
        double rVel = rightDrive.velocity(vex::velocityUnits::rpm);

        // Log to file
        logFile << t << "," << voltage << "," << lVel << "," << rVel << "\n";

        // Apply voltage
        leftDrive.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
        rightDrive.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);

        vex::wait(10, vex::msec); // 100hz logging rate
    }

    // Stop motors between runs
    leftDrive.stop(vex::brakeType::coast);
    rightDrive.stop(vex::brakeType::coast);
    logFile << "\n";
    logFile.close();
}