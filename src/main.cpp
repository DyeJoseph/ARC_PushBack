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

  bool isInAuton = false;
  int lastPressed = 0;
  int teamColor = 0; //red = 0, blue = 1
  int driver = 0; //Elliot = 0, Jacob = 1

  //Prototypes
  void toggleLift();
  void toggleIntakeFlap();


  // Define Values for the Chassis here:
  Drive chassis
  (
    motor_group(LFT, LFB, LBB, LBT), // Left drive train motors
    motor_group(RFT, RFB, RBB, RBT), // Right drive train motors
    PORT20,               // Inertial Sensor Port
    2.75,              // The diameter size of the wheel in inches
    1,                   // 
    12,                   // The maximum amount of the voltage used in the drivebase (1 - 12)
    odomType,
    1.93,                  //Odometry wheel diameter (set to zero if no odom)
    -3.687,               //Odom pod1 offset 
    -3.867                //Odom pod2 offset
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

//////////////////////////////////////////////////////////////////////


/// @brief Runs before the competition starts
void preAuton() 
{
  setDriveTrainConstants();

  chassis.brake(coast);       // make sure they aren’t holding weirdly
  chassis.driveMotors(0, 0);  

  enum preAutonStates{START_SCREEN = 0, SELECTION_SCREEN = 1};
  int currentScreen = START_SCREEN;
  int lastPressed = 0;

  // Calibrates/Resets the Brains sensors before the competition
  inertial1.calibrate();
  rotation1.resetPosition();
  rotation2.resetPosition();

  vex::color colors[8] = {vex::color::red, vex::color::red, vex::color::red, vex::color::red, 
                          vex::color::blue, vex::color::blue, vex::color::blue, vex::color::blue};
  std::string names[8] = {"Auton 1", "Auton 2", "Auton 3", "Auton 4", 
                          "Auton 5", "Auton 6", "Auton 7", "Auton 8"};
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
  //drawSponsors();

  isInAuton = true;
  rotation1.resetPosition();
  rotation2.resetPosition();
  inertial1.resetHeading();
  wait(100, msec);

  setDriveTrainConstants();


 
  // Tiny forward preload to take up slack
  chassis.driveMotors(2, 2); // 2 volts forward for a moment
  wait(100, msec);
  chassis.brake(hold);


  chassis.setPosition(0,0,0);
  //chassis.driveDistance(24);
  //chassis.driveDistanceWithOdom(24);

  //chassis.bezierTurn(0,0,5,5,2,12,7);
  //chassis.driveDistanceWithOdom(24);
  // chassis.setTurnMaxVoltage(8);
  // chassis.turnToAngle(90);
  //chassis.moveToPosition(12,24);
  //chassis.turnToPosition(24,24);

 

  // switch (lastPressed) 
  // {
  //   case 1:
  //     Auton_1();
  //     break;
  //   case 2:
  //     Auton_2();
  //     break;
  //   case 3:
  //     Auton_3;
  //     break;
  //   case 4:
  //     Auton_4();
  //     break;
  //   case 5:
  //     Auton_5();
  //     break;
  //   case 6:
  //     Auton_6();
  //     break;
  //   case 7:
  //     Auton_7();
  //     break;
  //   case 8:
  //     Auton_8();
  //     break;
  //   default:
  //     DefaultAuton();
  //     break;
  // }
}

/// @brief Runs during the UserControl section of the competition
void usercontrol() 
{
  //drawSponsors();
 
  

  // User control code here, inside the loop
  bool flapState = false;
  bool isColorSorting = true;
  int lastSeen = teamColor;

  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);

  Controller1.ButtonL1.pressed(toggleLift);
  Controller1.ButtonUp.pressed(toggleIntakeFlap);

  bottomColorSort.setLight(ledState::on);
  while (1) {
    chassis.arcade();
    

    if(bottomColorSort.color() == vex::color::red){
      lastSeen = 0;
    }else if(bottomColorSort.color() == vex::color::blue){
      lastSeen = 1;
    }

    if(Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()){
      mainIntake.spin(forward);
      if(flapState){
        topStage.spin(forward);
      }else{
        topStage.stop();
      }
      if(lastSeen == teamColor){
        colorSort.spin(forward);
        std::cout << "COLORSORT FWD" << std::endl;
      }else{
        colorSort.spin(reverse);
        std::cout << "COLORSORT REV" << std::endl;

      }
    }else if(Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
      mainIntake.spin(reverse);
      topStage.spin(reverse);
      colorSort.spin(fwd,10,pct);
    }else if(Controller1.ButtonL2.pressing()){
      matchLoad.set(true);
      mainIntake.spin(forward);
      if(lastSeen == teamColor){
        colorSort.spin(forward);
        std::cout << "COLORSORT FWD" << std::endl;
      }else{
        colorSort.spin(reverse);
        std::cout << "COLORSORT REV" << std::endl;
      }
    }else if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
      mainIntake.spin(forward);
      topStage.spin(forward);
      flapState = true;
      if(lastSeen == teamColor){
        colorSort.spin(forward);
        std::cout << "COLORSORT FWD" << std::endl;
      }else{
        colorSort.spin(reverse);
        std::cout << "COLORSORT REV" << std::endl;
      }
    }else{
      matchLoad.set(false);
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
    }
    if(!Controller1.ButtonR1.pressing()){
      flapState = false;
    }
    intakeFlap.set(flapState);




    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Axis3: %d", Controller1.Axis3.position());
    Brain.Screen.newLine();
    Brain.Screen.print("Axis1: %d", Controller1.Axis1.position());
    Brain.Screen.newLine();
    Brain.Screen.print("color: ", isColorSorting);

    wait(20, msec);
  }
}

void toggleLift(){
  static bool liftState = false;
  liftState = !liftState;
  intakeLift.set(liftState);
}

void toggleIntakeFlap(){
  static bool flapState = false;
  flapState = !flapState;
  intakeFlap.set(flapState);
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
        0.35,  // Kp - Proportion Constant
        0.0003, // Ki - Integral Constant
        0.17, // Kd - Derivative Constant
        .5, // Settle Error
        300, // Time to Settle
        3000 // End Time 5000
    );

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        0.30,    // Kp - Proportion Constant
        0,      // Ki - Integral Constant
        0,      // Kd - Derivative Constant 
        0.5,    // Settle Error
        300,    // Time to Settle
        3000    // End Time
    );
    
}





//Auton Route Functions
/// @brief Auton Slot 1 - Write code for route within this function.
void Auton_1()
{
    Brain.Screen.print("Skills 1 running.");
    chassis.setTurnMaxVoltage(8);
    chassis.setPosition(-47,15,90);

}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2()
{
    Brain.Screen.print("Skills 2 running.");
}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3()
{
    Brain.Screen.print("Auton 3 running.");
    //KEEGAN WRITE HERE

    //SETUP
    chassis.setPosition(-46,10.5,180);
    chassis.setTurnMaxVoltage(8);
    

    //MOVE FORWARD AND INTAKE 2 RED
    chassis.moveToPosition(-46, 0.5);
    wait(1, sec);
    
    //GRAB 8 FROM MATCH LOADER 2
    chassis.driveDistanceWithOdom(-10);
    chassis.driveDistanceWithOdom(-36.2);
    chassis.moveToPosition(-58, 46.7);
    wait(1, sec);

    //LOAD 7 INTO LONG GOAL 1 (5RED, THEN 2BLUE)
    chassis.driveDistanceWithOdom(-5);
    chassis.moveToPosition(-32, 46.7);
    wait(-1, sec);

    //GRAB 2 BLUE AT TOP
    chassis.driveDistanceWithOdom(-14);
    chassis.moveToPosition(-46, 62.5);
    wait(1, sec);

    //GRAB 4 FROM START
    chassis.driveDistanceWithOdom(-10);
    chassis.moveToPosition(-30,0);
    chassis.moveToPosition(-46,0);
    wait(1, sec);

    //LOAD INTO UPPER GOAL
    chassis.driveDistanceWithOdom(-10);
    chassis.moveToPosition(-17.5, 18.5);
    chassis.moveToPosition(-13, 13.5);
    wait(-1, sec);

    //BLOCK LOWER MIDDLE GOAL
    chassis.driveDistanceWithOdom(-10);
    chassis.moveToPosition(-19.6, -4.9);
    chassis.turnToAngle(140);

    //GRAB 8 FROM MATCH LOADER
    chassis.moveToPosition(-47, -47);
    chassis.moveToPosition(-58, -47);
    wait(1, sec);

    //GRAB 2 BLUE FROM BOTTOM
    chassis.driveDistanceWithOdom(-11);
    chassis.moveToPosition(-47, -62.5);
    wait(1, sec);

    //LOAD IN TO BOTTOM LONG GOAL 2
    chassis.driveDistanceWithOdom(-15.5);
    chassis.moveToPosition(-31.7, -47);
    wait(-1, sec);

    //RAM INTO PARK ZONE
    chassis.driveDistanceWithOdom(-20);
    chassis.moveToPosition(-63.8, -8.2);




  



}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4()
{
    Brain.Screen.print("Auton 4 running.");
}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5()
{
    Brain.Screen.print("Auton 5 running.");
}

/// @brief Auton Slot 6 - Write code for route within this function.
void Auton_6()
{
    Brain.Screen.print("Auton 6 running.");
}

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7()
{
    Brain.Screen.print("Auton 7 running.");
}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8()
{
    Brain.Screen.print("Auton 8 running.");
    
}