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
  Brain.Screen.clearScreen();
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

  setDriveTrainConstants();

  /*switch (lastPressed) 
  {
    case 0:
      Auton_1();
      break;
    case 1:
      Auton_2();
      break;
    case 2:
      Auton_3();
      break;
    case 3:
      Auton_4();
      break;
    case 4:
      Auton_5();
      break;
    case 5:
      Auton_6();
      break;
    case 6:
      Auton_7();
      break;
    case 7:
      Auton_8();
      break;
    default:
      break;
  }*/
  Auton_3();
}

/// @brief Runs during the UserControl section of the competition
void usercontrol() 
{
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


    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(0,0,0);
    chassis.setDriveMaxVoltage(12);
    chassis.setTurnMaxVoltage(12);

    chassis.driveDistanceWithOdom(24);
    std::cout << "\nHIT 1, X: " << chassis.chassisOdometry.getXPosition() 
              << ", Y: " << chassis.chassisOdometry.getYPosition() 
              << ", H: " << chassis.chassisOdometry.getHeading();
    

    //DRIVE TEST 1
    // std::cout << "\nSTARTING AUTON\n";
    // chassis.driveDistanceWithOdom(6);     // pos: 6
    // chassis.driveDistanceWithOdom(5);     // pos: 11
    // chassis.driveDistanceWithOdom(18);    // pos: 29
    // chassis.driveDistanceWithOdom(-7);    // pos: 22
    // chassis.driveDistanceWithOdom(20);    // pos: 42
    // chassis.driveDistanceWithOdom(-6);    // pos: 36
    // chassis.driveDistanceWithOdom(-25);   // pos: 11
    // chassis.driveDistanceWithOdom(7);     // pos: 18
    // chassis.driveDistanceWithOdom(30);    // pos: 48 
    // chassis.driveDistanceWithOdom(-6);    // pos: 42
    // chassis.driveDistanceWithOdom(-35);   // pos: 7
    // chassis.driveDistanceWithOdom(-7);    // pos: 0
    // chassis.driveDistanceWithOdom(10);    // pos: 10
    // chassis.driveDistanceWithOdom(-5);    // pos: 5
    // chassis.driveDistanceWithOdom(15);    // pos: 20
    // chassis.driveDistanceWithOdom(-10);   // pos: 10
    // chassis.driveDistanceWithOdom(-5);    // pos: 5
    // chassis.driveDistanceWithOdom(5);     // pos: 10
    // chassis.driveDistanceWithOdom(-10);   // pos: 0

    //DRIVE TEST 2
    // std::cout << "\nSTARTING AUTON\n";
    // chassis.driveDistanceWithOdom(8);     // pos: 8
    // chassis.driveDistanceWithOdom(12);    // pos: 20
    // chassis.driveDistanceWithOdom(15);    // pos: 35
    // chassis.driveDistanceWithOdom(-10);   // pos: 25
    // chassis.driveDistanceWithOdom(18);    // pos: 43
    // chassis.driveDistanceWithOdom(-8);    // pos: 35
    // chassis.driveDistanceWithOdom(-20);   // pos: 15
    // chassis.driveDistanceWithOdom(10);    // pos: 25
    // chassis.driveDistanceWithOdom(23);    // pos: 48
    // chassis.driveDistanceWithOdom(-12);   // pos: 36
    // chassis.driveDistanceWithOdom(-30);   // pos: 6
    // chassis.driveDistanceWithOdom(-6);    // pos: 0
    // chassis.driveDistanceWithOdom(14);    // pos: 14
    // chassis.driveDistanceWithOdom(-9);    // pos: 5
    // chassis.driveDistanceWithOdom(20);    // pos: 25
    // chassis.driveDistanceWithOdom(-15);   // pos: 10
    // chassis.driveDistanceWithOdom(-5);    // pos: 5
    // chassis.driveDistanceWithOdom(-5);    // pos: 0

    //DRIVE TEST 3
    // std::cout << "\nSTARTING AUTON\n";
    // chassis.driveDistanceWithOdom(10);    // pos: 10
    // chassis.driveDistanceWithOdom(15);    // pos: 25
    // chassis.driveDistanceWithOdom(12);    // pos: 37
    // chassis.driveDistanceWithOdom(-9);    // pos: 28
    // chassis.driveDistanceWithOdom(18);    // pos: 46
    // chassis.driveDistanceWithOdom(-11);   // pos: 35
    // chassis.driveDistanceWithOdom(-25);   // pos: 10
    // chassis.driveDistanceWithOdom(8);     // pos: 18
    // chassis.driveDistanceWithOdom(30);    // pos: 48
    // chassis.driveDistanceWithOdom(-14);   // pos: 34
    // chassis.driveDistanceWithOdom(-24);   // pos: 10
    // chassis.driveDistanceWithOdom(-10);   // pos: 0
    // chassis.driveDistanceWithOdom(16);    // pos: 16
    // chassis.driveDistanceWithOdom(-6);    // pos: 10
    // chassis.driveDistanceWithOdom(20);    // pos: 30
    // chassis.driveDistanceWithOdom(-15);   // pos: 15
    // chassis.driveDistanceWithOdom(-10);   // pos: 5
    // chassis.driveDistanceWithOdom(-5);    // pos: 0

    //TURN TEST 1
    // std::cout << "\nSTARTING AUTON\n";
    // chassis.turnToAngle(45);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(120);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(30);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(275);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(50);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(310);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(180);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(90);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(350);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(60);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(200);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(10);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(330);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(25);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(140);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(5);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(300);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(15);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(0);
    // std::cout << chassis.chassisOdometry.getHeading() << std::endl;
    // chassis.turnToAngle(0);

    //TURN TEST 2
    // std::cout << "\nSTARTING AUTON\n";
    // chassis.turnToAngle(60);
    // chassis.turnToAngle(180);
    // chassis.turnToAngle(5);
    // chassis.turnToAngle(270);
    // chassis.turnToAngle(90);
    // chassis.turnToAngle(300);
    // chassis.turnToAngle(15);
    // chassis.turnToAngle(225);
    // chassis.turnToAngle(350);
    // chassis.turnToAngle(40);
    // chassis.turnToAngle(310);
    // chassis.turnToAngle(50);
    // chassis.turnToAngle(200);
    // chassis.turnToAngle(75);
    // chassis.turnToAngle(330);
    // chassis.turnToAngle(120);
    // chassis.turnToAngle(10);
    // chassis.turnToAngle(280);
    // chassis.turnToAngle(140);
    // chassis.turnToAngle(0);

    //TURN TEST 3
    // std::cout << "\nSTARTING AUTON\n";
    // chassis.turnToAngle(30);
    // chassis.turnToAngle(150);
    // chassis.turnToAngle(5);
    // chassis.turnToAngle(270);
    // chassis.turnToAngle(45);
    // chassis.turnToAngle(320);
    // chassis.turnToAngle(60);
    // chassis.turnToAngle(210);
    // chassis.turnToAngle(10);
    // chassis.turnToAngle(340);
    // chassis.turnToAngle(25);
    // chassis.turnToAngle(190);
    // chassis.turnToAngle(80);
    // chassis.turnToAngle(355);
    // chassis.turnToAngle(100);
    // chassis.turnToAngle(5);
    // chassis.turnToAngle(270);
    // chassis.turnToAngle(60);
    // chassis.turnToAngle(350);
    // chassis.turnToAngle(0);



      std::cout << "X: " << chassis.chassisOdometry.getXPosition() 
        << " Y: " << chassis.chassisOdometry.getYPosition() 
        << " Heading: " << chassis.chassisOdometry.getHeading() 
        << std::endl;
}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2() // UCF Route
{

  //thread odomThread(odomDebugThread);
  // SETTING UP
  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);
  bottomStage.setVelocity(100, percent);

  chassis.setPosition(-46,-8,0);
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(10);

  toggleLift();
  wait(0.2, sec);
  toggleLift();

  // GRAB MATCH LOADS
  matchLoad.set(true);
  mainIntake.spin(forward);

  // BLOCKING TOP MIDDLE
  chassis.driveDistanceWithOdomTime(-39,1400);
  chassis.turnToAngleTime(60,1400,10);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(13.5, -15.5, 60);
  // wait(0.1, sec);


  // GRABS 6 FROM MATCH LOAD
  chassis.driveDistanceWithOdomTime(-15,700); // -67.5
  // wait(0.1, sec);
  // chassis.turnToAngle(240);
  chassis.turnToAngleTime(150,500,10);
  chassis.turnToAngle(240);
  // chassis.movetopos(-46,-47,240); //-46,-47
  chassis.movetopos(-53,-49,240); //-53,-53,270
  chassis.turnToAngleTime(270,1200,10);
  chassis.driveDistanceWithOdomTime(8, 400);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(1.5, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop(); 


  // OUTTAKE 3 BLOCKS
  chassis.driveDistanceWithOdomTime(-5,400);
  chassis.turnToAngleTime(145,500,10);
  toggleLift(); // up
  mainIntake.spin(reverse, 50, percent);
  wait(.6, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  // SCORE 5 INTO HIGH GOAL
  chassis.turnToAngleTime(180,500,10);
  chassis.turnToAngle(90);
  // wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.driveDistanceWithOdomTime(22,750);
  // chassis.movetopos(-40,-33,90); // -33.5,-47,90
  toggleIntakeFlap(); // open
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(.8, sec);
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.15, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(.7, sec);
  toggleIntakeFlap(); // close

  // GRAB 8 FROM MATCH LOAD
  chassis.driveDistanceWithOdomTime(-5,400);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); // down
  chassis.turnToAngleTime(180,500,10); 
  chassis.turnToAngleTime(270,1000,10);
  chassis.movetopos(-53, -45, 270); //-57
  chassis.driveDistanceWithOdomTime(8,400);
  matchLoad.set(true);
  colorSort.spin(forward);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(2.7,sec); //3.5
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  // SCORE 8 INTO HIGH GOAL
  chassis.driveDistanceWithOdomTime(-5, 500);
  // chassis.turnToAngle(90);
  matchLoad.set(false);
  toggleLift(); // up
  chassis.turnToAngleTime(180,600,10);
  chassis.turnToAngle(90); 
  // wait(.1, sec);
  // chassis.movetopos(-40,-33, 90); // -32,-47,90
  chassis.driveDistanceWithOdomTime(22,750);
  toggleIntakeFlap(); // open
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(1.2, sec); // 1
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.2, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(1.5, sec); // 1.7
  toggleIntakeFlap(); // close

  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3() //1 MINUTE SKI
{   

    vex::timer myTimer;
    myTimer.reset();

    //thread odomThread(odomDebugThread);
    //SETUP
    std::cout << "\n\n\nSTART";
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(-46,0,270);
    chassis.setDriveMaxVoltage(12);
    chassis.setTurnMaxVoltage(12);

    //GRAB 4 BALLS
    toggleLift(); //UP
    wait(0.1, sec);
    toggleDropDown(); //DOWN
    wait(.3, sec); // .5
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(-15);
    std::cout << "\nHIT 1, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.driveDistanceWithOdom(5);
    matchLoad.set(true);
    std::cout << "\nHIT 2, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    colorSort.stop();
    topStage.stop();

    //GRAB 2 BLUE ON WALL
    chassis.turnToAngle(194);
    chassis.driveDistanceWithOdom(45);
    matchLoad.set(false);
    std::cout << "\nHIT 3, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(180);
    toggleDropDown(); //UP
    bottomStage.spin(forward);
    chassis.driveDistanceWithOdom(17);
    std::cout << "\nHIT 4, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.setDriveMaxVoltage(6);
    std::cout <<  "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(4, 200);
    matchLoad.set(true); //OPEN
    chassis.driveDistanceWithOdom(-13.5);
    std::cout << "\nHIT 5, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.setDriveMaxVoltage(12);
    matchLoad.set(false); //CLOSE
    colorSort.stop();
    topStage.stop();

    //LOAD 6 BLUE INTO LONG GOAL
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdom(15);
    std::cout << "\nHIT 7, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    toggleIntakeFlap(); //OPEN
    std::cout <<  "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(5, 200);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward, 100, percent);
    wait(1.7, sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();

    //GRAB 6 MATCH LOAD
    chassis.driveDistanceWithOdom(-5);
    toggleIntakeFlap(); //CLOSE
    std::cout << "\nHIT 8, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(278.5);
    chassis.driveDistanceWithOdom(21);
    chassis.turnToAngle(270);
    std::cout << "\nHIT 9, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    std::cout <<  "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(6, 200);
    matchLoad.set(true); //OPEN
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec); //1.7

    //LOAD 5 INTO LONG GOAL
    chassis.driveDistanceWithOdom(-5);
    std::cout << "\nHIT 10, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(97);
    matchLoad.set(false); //CLOSE
    chassis.driveDistanceWithOdom(21);
    std::cout << "\nHIT 11, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngleTime(90, 500, 10);
    std::cout << "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(6, 200);
    toggleIntakeFlap(); //OPEN
    mainIntake.spin(reverse);
    colorSort.spin(reverse);
    topStage.spin(reverse);
    wait(.1, sec);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward, 100, percent);
    wait(1.4, sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleIntakeFlap(); //CLOSE


    //GRAB 6 FAR MATCH LOADER
    chassis.driveDistanceWithOdom(-12);
    std::cout << "\nHIT 12, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(132);
    chassis.driveDistanceWithOdom(20.6);
    std::cout << "\nHIT 13, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(92);
    chassis.driveDistanceWithOdom(60);
    std::cout << "\nHIT 14, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(51);
    chassis.driveDistanceWithOdom(18);
    //chassis.movetopos(49, -51, 51);
    std::cout << "\nHIT 15, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(94);
    chassis.driveDistanceWithOdom(8);
    std::cout << "\nHIT 16, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    std::cout << "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(5, 300);
    matchLoad.set(true); //OPEN
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec); //1.7

    //LOAD 4 INTO LONG GOAL
    chassis.driveDistanceWithOdom(-5);
    std::cout << "\nHIT 17, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false); //CLOSE
    chassis.turnToAngle(274);
    chassis.driveDistanceWithOdom(22);
    chassis.turnToAngleTime(270, 200, 10);
    std::cout << "\nHIT 18, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    toggleIntakeFlap(); //OPEN
    std::cout << "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(5, 100);
    mainIntake.spin(reverse);
    colorSort.spin(reverse);
    topStage.spin(reverse);
    wait(.1, sec);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward, 100, percent);
    wait(.9, sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleIntakeFlap(); //CLOSE
    wait(.1, sec);

    //GRAB 2 RED ON WALL
    chassis.driveDistanceWithOdom(-15);
    std::cout << "\nHIT 19, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.turnToAngle(180);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(14);
    std::cout << "\nHIT 20, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.setDriveMaxVoltage(6);
    std::cout <<  "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(5, 100);
    matchLoad.set(true); //OPEN
    chassis.driveDistanceWithOdom(-5);
    std::cout << "\nHIT 21, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    chassis.setDriveMaxVoltage(12);

    //OPTION 1 GRAB 2 RED FROM MIDDLE LINE AND LOAD 7 INTO TOP MID
    //OPTION 2 LOAD 5 RED IN 
    chassis.driveDistanceWithOdom(-11);
    std::cout << "\nHIT 22, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    matchLoad.set(false); //CLOSE
    colorSort.stop();
    topStage.stop();
    chassis.turnToAngle(316);
    toggleLift(); //DOWN
    toggleDropDown(); //DOWN
    chassis.driveDistanceWithOdom(42.5);
    toggleIntakeFlap();
    std::cout << "\nHIT 23, X: " << chassis.chassisOdometry.getXPosition() << ", Y: " << chassis.chassisOdometry.getYPosition() << ", H: " << chassis.chassisOdometry.getHeading();
    std::cout <<  "\nMy Timeout: ";
    chassis.turnToAngle(315);
    chassis.driveDistanceWithOdomTime(6, 200);
    std::cout <<  "\nMy Timeout: ";
    chassis.driveDistanceWithOdomTime(-4, 100);
    mainIntake.spin(reverse);
    colorSort.spin(reverse);
    topStage.spin(reverse);
    wait(.1, sec);
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 70, percent);
    wait(2, sec);

    chassis.driveDistanceWithOdom(-20);
    colorSort.stop();
    topStage.stop();
    toggleDropDown(); //UP
    chassis.turnToAngle(270);
    chassis.driveDistanceWithOdom(87);
    chassis.turnToAngle(350);
    chassis.driveDistanceWithOdom(-6);
    while (myTimer.time(sec) <= 58){
      wait(10, msec);
    }
    chassis.setDriveMaxVoltage(12);
    chassis.driveDistanceWithOdom(100);




    //PARK
    //toggleDropDown(); //UP


    /*//GRAB 4 BLUE BALLS
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 1\n";
    toggleLift(); //UP
    wait(0.1, sec);
    toggleDropDown(); // down
    wait(.3, sec); // .5
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(-15);
    matchLoad.set(true);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 2\n";
    //chassis.driveDistanceWithOdom(5); //MOVETOPOS
    chassis.movetopos(-36, 0, 270);
    chassis.chassisOdometry.setPosition(-36, 0, 270);

    //GRAB 2 BLUE WALL BALLS
    chassis.turnToAngle(192);
    //chassis.setDriveMaxVoltage(10);
    chassis.movetopos(-46, -47, 192);
    chassis.chassisOdometry.setPosition(-46, -47, 192);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 3\n";
    bottomStage.spin(forward);
    mainIntake.spin(forward);
    //chassis.setDriveMaxVoltage(8);
    chassis.turnToAngleTime(180, 200, 8);
    matchLoad.set(false);
    //mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleDropDown(); // up
    chassis.movetopos(-46, -60, 180);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 4\n";

    mainIntake.spin(forward);
    colorSort.spin(forward);
    matchLoad.set(true);
    chassis.driveDistanceWithOdomTime(-5, 400); 
    

    //GRAB 1 BLUE BALL
    chassis.turnToAngle(41);
    //chassis.setDriveMaxVoltage(10);
    chassis.movetopos(-14, -23, 41);
    chassis.chassisOdometry.setPosition(-14, -23, 41);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 5\n";
    //chassis.setDriveMaxVoltage(8);
    mainIntake.stop();
    colorSort.stop();
    chassis.turnToAngle(90);
    matchLoad.set(false);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    //chassis.setDriveMaxVoltage(10);
    chassis.movetopos(18,-22, 90);
    chassis.chassisOdometry.setPosition(18, -22, 90);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 6\n";
    //chassis.setDriveMaxVoltage(8);
    toggleLift(); //DOWN
    wait(.1, sec);

    //LOAD 7 BALLS
    matchLoad.set(true);
    chassis.turnToAngle(314);
    toggleDropDown(); // down
    chassis.driveDistanceWithOdomTime(9, 600);
    chassis.driveDistanceWithOdomTime(-2, 200);
    //chassis.movetopos(14.5, -15.5, 315); //15.5
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 7\n";
    toggleIntakeFlap();
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 36, percent); //65 // 55 // 45
    wait(1.4, sec); //1.2
    topStage.spin(reverse);
    bottomStage.spin(reverse);
    wait(.2, sec);
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 30, percent); // 55 // 45 // 36
    wait(2.3, sec); // 1.8 before change
    chassis.driveDistanceWithOdomTime(2, 200);
    chassis.driveDistanceWithOdomTime(-2, 200);

    //GRAB 2 RED ON WALL
    matchLoad.set(false);
    mainIntake.spin(reverse, 100, pct);
    colorSort.spin(reverse, 100, pct);
    topStage.spin(reverse, 100, pct);
    chassis.driveDistanceWithOdom(-10);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 7.5\n";
    chassis.driveDistanceWithOdom(-35.5);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 8\n";
    colorSort.stop();
    topStage.stop();
    chassis.turnToAngleTime(180, 1000, 10);
    toggleDropDown(); //up
    toggleIntakeFlap();
    chassis.movetopos(46, -63, 180);
    chassis.driveDistanceWithOdomTime(2, 200);
    chassis.chassisOdometry.setPosition(46, -63, 180);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 9\n";
    mainIntake.spin(forward);
    colorSort.spin(forward);
    matchLoad.set(true);
    wait(.2, sec);
    chassis.driveDistanceWithOdom(-12);  //MERGE 519-526


    //MATCH LOAD

    colorSort.stop();
    topStage.stop();
    chassis.turnToAngle(91);
    toggleLift(); //up
    matchLoad.set(false);
    chassis.movetopos(54, -51.5, 91);
    chassis.chassisOdometry.setPosition(55.5, -51.5, 91);
    chassis.driveDistanceWithOdomTime(4, 400);
    //chassis.movetopos(58, -51, 91);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 10\n";
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec); //1.7


    //LOAD LONG GOAL
    chassis.driveDistanceWithOdomTime(-5, 400); // -14.6
    chassis.turnToAngle(270); // 90
    //toggleLift();
    matchLoad.set(false);
    chassis.movetopos(30.5, -49.5, 270);
    chassis.chassisOdometry.setPosition(30.5, -49.5, 270);
    chassis.driveDistanceWithOdomTime(4, 200);
    //chassis.movetopos(29, -49.5, 270); // (11.5 + 9.6)
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 11\n";
    toggleIntakeFlap(); //open
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward, 20, percent);
    wait(1, sec);
    topStage.spin(reverse);
    colorSort.spin(reverse);
    bottomStage.spin(reverse);
    wait(0.15, sec);
    topStage.spin(forward, 100, percent);
    colorSort.spin(forward); 
    mainIntake.spin(forward);
    wait(2.7,sec); // 2 // 2.5
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();

    //GRAB 2 RED UNDER GOAL
    chassis.driveDistanceWithOdom(-25); 
    chassis.turnToAngleTime(250, 200, 10);
    chassis.movetopos(25, -61, 250);
    chassis.chassisOdometry.setPosition(25, -61, 250);
    toggleIntakeFlap(); // close
    chassis.turnToAngleTime(270, 200, 10);
    chassis.movetopos(-5.75, -61, 270);
    chassis.chassisOdometry.setPosition(-5.75, -61, 270);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 12\n";
    chassis.turnToAngle(0);
    toggleLift(); //down 
    wait(.1, sec);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdomTime(6, 400);
    matchLoad.set(true);
    chassis.driveDistanceWithOdomTime(-6, 400);


    //MATCH LOAD
    chassis.turnToAngle(270);
    chassis.movetopos(-30,-60, 270);
    chassis.chassisOdometry.setPosition(-30, -60, 270);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 13\n";
    matchLoad.set(false);
    chassis.turnToAngle(305);
    chassis.movetopos(-53, -45, 305);
    chassis.chassisOdometry.setPosition(-53, -45, 305);
    toggleLift(); //up
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 14\n";
    chassis.turnToAngle(271);
    chassis.movetopos(-58, -45, 271);
    chassis.chassisOdometry.setPosition(-58, -45, 271);
    chassis.driveDistanceWithOdomTime(4, 400);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 15\n";
     matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec);


    //LONG GOAL
    chassis.driveDistanceWithOdomTime(-6, 400);
    matchLoad.set(false);
    chassis.turnToAngleTime(90, 1000, 10);
    //toggleLift();
    chassis.movetopos(-33.5,-47, 90);
    chassis.chassisOdometry.setPosition(-33.5, -47, 90);
    chassis.driveDistanceWithOdomTime(6, 400);
    std::cout << "HIIIIIIIIIIIIIIIIIIIIT 16\n";
    toggleIntakeFlap();
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1, sec);
    topStage.spin(reverse);
    colorSort.spin(reverse);
    bottomStage.spin(reverse);
    wait(0.15, sec);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1, sec);


    odomDebugEnabled = false;
    chassis.moveable();*/

}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4() //TOP MIDDLE DEFENSE
{
  //SET UP
  //thread odomThread(odomDebugThread);
  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);
  bottomStage.setVelocity(100, percent);
  chassis.setPosition(-46,-8,0);
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(10);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdomTime(-39, 1500);
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(-55, -48, 270);
  chassis.driveDistanceWithOdomTime(4, 400);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdomTime(-6, 1000);
  chassis.turnToAngle(200);
  wait(.1, sec);
  mainIntake.spin(reverse, 50, percent);
  wait(.8, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //GRAB 5
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.driveDistanceWithOdomTime(7, 1000);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);
  chassis.driveDistanceWithOdomTime(-6, 1000);

  //LOAD INTO LONG GOAL
  chassis.turnToAngle(94);
  wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-29, -48.5, 94);
  chassis.driveDistanceWithOdomTime(6, 600);
  toggleIntakeFlap(); //OPEN
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(1.5, sec);
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.2, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(2.7, sec); // 1.5
  toggleIntakeFlap(); //CLOSE

  //RAM MIDDLE LEFT
  toggleLift();
  chassis.driveDistanceWithOdom(-20);
  chassis.turnToAngle(65);
  chassis.movetopos(14,-20, 65);

}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5() //PARK
{

}

/// @brief Auton Slot 6 - Write code for route within this function.
void Auton_6() //DOUBLE LOAD TOP
{
   //SET UP
  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);
  bottomStage.setVelocity(100, percent);
  chassis.setPosition(-46,-8,0);
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(10);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdomTime(-39, 1500);
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(-55, -47, 270);
  chassis.driveDistanceWithOdomTime(4, 400);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdomTime(-6, 1000);
  chassis.turnToAngle(200);
  wait(.1, sec);
  mainIntake.spin(reverse, 50, percent);
  wait(.8, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //LOAD INTO LONG GOAL
  chassis.turnToAngleTime(94, 600, 10);
  wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-31, -48.5, 94);
  chassis.driveDistanceWithOdomTime(8, 600);
  toggleIntakeFlap(); //OPEN
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(1, sec);
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.2, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(1.2, sec); // 1.5
  toggleIntakeFlap(); //CLOSE

  //GRAB MATCH LOADS
  chassis.driveDistanceWithOdomTime(-6, 1000); // -7
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); //DOWN
  wait(.1, sec);
  chassis.driveDistanceWithOdomTime(-6, 1000);
  chassis.turnToAngleTime(270, 1000, 10);
  chassis.movetopos(-55, -47, 270);
  chassis.driveDistanceWithOdomTime(6, 400);
  matchLoad.set(true);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(2.5,sec);
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //LOAD TOP GOAL AGAIN
  chassis.driveDistanceWithOdomTime(-6, 1000);
  chassis.turnToAngleTime(180, 500, 10);  
  chassis.turnToAngleTime(90, 500, 10);
  wait(.1, sec);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-31, -48.5, 94);
  chassis.driveDistanceWithOdomTime(6, 600);
  toggleIntakeFlap(); //OPEN
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(1, sec);
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.2, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(1.7, sec); // 1.5

}

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7() //SCORES TOP MIDDLE REAL
{
    //SET UP
  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);
  bottomStage.setVelocity(100, percent);
  chassis.setPosition(-46,-8,0);
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(10);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdomTime(-39,1500);
  // chassis.setDriveMaxVoltage(10);
  // wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.turnToAngleTime(269,1000,10);
  chassis.movetopos(-55, -48, 269);
  chassis.driveDistanceWithOdomTime(4,400);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(1.5, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdomTime(-5,400);
  chassis.turnToAngleTime(145,500,10);
  toggleLift(); // up
  mainIntake.spin(reverse, 50, percent);
  wait(.6, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //LOAD INTO LONG GOAL
  chassis.turnToAngleTime(180,500,10);
  chassis.turnToAngle(95);
  // wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  // wait(.1, sec);
  // chassis.movetopos(-24.5, -45, 90);
  chassis.driveDistanceWithOdomTime(21,750); //22
  chassis.driveDistanceWithOdomTime(4, 400);
  toggleIntakeFlap(); //OPEN
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(1, sec);
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.15, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(1.2, sec); // 1.5
  toggleIntakeFlap(); //CLOSE

  //GRAB MATCH LOADS
  chassis.driveDistanceWithOdomTime(-5, 400); // -7
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); //DOWN
  // wait(.1, sec);
  chassis.driveDistanceWithOdomTime(-5,500);
  chassis.turnToAngleTime(180,400,10);
  chassis.turnToAngleTime(272,1100,10);
  chassis.movetopos(-53, -45, 270);
  chassis.driveDistanceWithOdomTime(10,400);
  matchLoad.set(true);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(3,sec);
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //LOAD MIDDLE
  chassis.driveDistanceWithOdomTime(-4,400);
  chassis.turnToAngleTime(30,1100,10);
  // wait(.1, sec);
  // chassis.setDriveMaxVoltage(10);
  chassis.movetopos(-16, 15, 36); // -16,17
  chassis.turnToAngleTime(138,1200,10);
  chassis.driveDistanceWithOdomTime(6,400); // 14
  chassis.driveDistanceWithOdomTime(-2, 200);
  toggleIntakeFlap();
  mainIntake.spin(forward, 100, percent);
  colorSort.spin(forward, 100, percent);
  topStage.spin(forward, 43, percent); //65 // 55 // 45
  wait(1.3, sec); //1.2
  topStage.spin(reverse);
  bottomStage.spin(reverse);
  wait(0.20, sec);
  mainIntake.spin(forward, 100, percent);
  colorSort.spin(forward, 100, percent);
  topStage.spin(forward, 36.5, percent); // 55 // 45 // 36
  wait(200, sec); // 1.8 before change
  
}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8() //SCORES LOW MIDDLE REAL
{
  //SET UP
  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);
  bottomStage.setVelocity(100, percent);
  chassis.setPosition(-46,-8,0);
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(10);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdomTime(-39,1500);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.turnToAngleTime(270,1000,10);
  chassis.movetopos(-55, -48.5, 270);
  chassis.driveDistanceWithOdomTime(4,400);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(1.5, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdomTime(-5,400);
  chassis.turnToAngleTime(145,500,10);
  toggleLift(); // up
  mainIntake.spin(reverse, 50, percent);
  wait(.6, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //LOAD INTO LONG GOAL
  chassis.turnToAngleTime(180,500,10);
  chassis.turnToAngle(95);
  // wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  // wait(.1, sec);
  // chassis.movetopos(-24.5, -45, 90);
  chassis.driveDistanceWithOdomTime(21,750); //22
  toggleIntakeFlap(); //OPEN
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(1, sec);
  topStage.spin(reverse);
  mainIntake.spin(reverse);
  wait(.15, sec);
  topStage.spin(forward);
  mainIntake.spin(forward);
  wait(1.2, sec); // 1.5
  toggleIntakeFlap(); //CLOSE

  //GRAB MATCH LOADS
  chassis.driveDistanceWithOdomTime(-5, 400); // -7
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); //DOWN
  // wait(.1, sec);
  chassis.driveDistanceWithOdomTime(-5,500);
  chassis.turnToAngleTime(180,400,10);
  chassis.turnToAngleTime(272,1100,10);
  chassis.movetopos(-53, -45, 270);
  chassis.driveDistanceWithOdomTime(10,400);
  matchLoad.set(true);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(3,sec);
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //DRIVE TO MIDDLE AND SCORE
  chassis.driveDistanceWithOdomTime(-4, 400);
  chassis.turnToAngleTime(44, 1000, 10);
  chassis.movetopos(-14,-12,44);
  mainIntake.spin(reverse, 17, percent); // 15 // 35
  colorSort.spin(reverse, 100, percent); // 90 // 100
  topStage.spin(reverse, 100, percent);
  wait(1.5, sec); // 4
  topStage.spin(forward, 100, percent);
  colorSort.spin(forward,100, percent);
  wait(.3, sec);
  colorSort.spin(reverse, 100, percent); // 90 // 100
  topStage.spin(reverse, 100, percent);
  chassis.driveDistanceWithOdom(-2);
  mainIntake.spin(reverse, 10, percent);
  topStage.spin(reverse, 100, percent);
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
