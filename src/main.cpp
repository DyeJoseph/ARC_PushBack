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

  bool isColorSorting = false; //SET TO TRUE NORMALLY

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
    2.66,              // The diameter size of the wheel in inches
    1,                   // 
    12,                   // The maximum amount of the voltage used in the drivebase (1 - 12)
    odomType,
    1.955,                  //Odometry wheel diameter (set to zero if no odom) (1.96 robot behind by .2)
    -3.867,               //Odom pod1 offset 
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
  std::string names[8] = {"NONE", "NONE", "1mSkill", "TopDef5", 
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
  /*drawSponsors();
  isInAuton = true;
  rotation1.resetPosition();
  rotation2.resetPosition();
  inertial1.resetHeading();
  wait(100, msec);

  setDriveTrainConstants();

  switch (lastPressed) 
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

  //CHANGE IF COLOR SORTING
  isColorSorting = false;

  chassis.brake(coast);
  mainIntake.setStopping(coast);

  mainIntake.setVelocity(85, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);

  Controller1.ButtonL1.pressed(toggleLift);
  Controller1.ButtonLeft.pressed(toggleDropDown);
  Controller1.ButtonRight.pressed(toggleColorSort);

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
      
      // if(flapState){
      //   topStage.spin(forward);
      // }else{
      //   topStage.stop();
      // }
      if(lastSeen == teamColor || !isColorSorting){
        colorSort.spin(forward);
      }else{
        colorSort.spin(reverse);
      }
    }else if(Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
      mainIntake.spin(reverse);
      topStage.spin(reverse);
      colorSort.spin(forward, 25, percent);
    }else if(Controller1.ButtonL2.pressing()){
      matchLoad.set(true);
      mainIntake.spin(forward);
      if(lastSeen == teamColor || !isColorSorting){
        colorSort.spin(forward);
      }else{
        colorSort.spin(reverse);
      }

    }else if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
      mainIntake.spin(forward);
      topStage.spin(forward);
      flapState = true;
      if(lastSeen == teamColor || !isColorSorting){
        colorSort.spin(forward);
      }else{
        colorSort.spin(reverse);
      }
        
    }else if(Controller1.ButtonUp.pressing()){
      flapState = true;
      mainIntake.spin(forward);
      topStage.spin(forward, 35, percent);
      if(lastSeen == teamColor || !isColorSorting){
        colorSort.spin(forward);
      }else{
        colorSort.spin(reverse);
      }
    }else if(Controller1.ButtonDown.pressing()){
      flapState = true;
      mainIntake.spin(reverse, 35, percent);
      topStage.spin(reverse);
      colorSort.spin(forward, 20, percent);
    }else{
      matchLoad.set(false);
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
    }

    if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonDown.pressing() && !Controller1.ButtonUp.pressing() && !Controller1.ButtonDown.pressing()){
      flapState = false;
    }

    intakeFlap.set(flapState);
  }
  wait(20, msec);
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
        0.7,  // Kp - Proportion Constant
        0.0001, // Ki - Integral Constant
        1.7, // Kd - Derivative Constant
        1.00, // Settle Error
        200, // Time to Settle
        2500 // End Time 5000
    );  

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        .25,    // Kp - Proportion Constant
        0.0,      // Ki - Integral Constant
        1.4,      // Kd - Derivative Constant 
        1.5, //1.25    // Settle Error
        200,    // Time to Settle
        1500    // End Time
    );
    
}

//Auton Route Functions
/// @brief Auton Slot 1 - Write code for route within this function.
void Auton_1() //EMPTY (UPDATE WHEN CHANGED)
{
    
}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2() //HIT MIDDLE TOP, THEN RUN
{
}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3() //1 MINUTE SKILL REAL
{
    //SETUP
    std::cout << "RUNNNN";
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(-46,0,270);
    chassis.setDriveMaxVoltage(8);
    chassis.setTurnMaxVoltage(8);

    //GRAB 4 BLUE BALLS
    toggleLift(); //UP
    wait(0.25, sec);
    toggleDropDown(); // down
    wait(.4, sec); // .5
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(-15);
    matchLoad.set(true);
    chassis.driveDistanceWithOdom(5);

    //GRAB 2 BLUE WALL BALLS
    chassis.turnToAngle(194);
    chassis.setDriveMaxVoltage(10);
    chassis.movetopos(-47, -47, 194);
    chassis.setDriveMaxVoltage(8);
    chassis.turnToAngle(180);
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleDropDown(); // up
    chassis.movetopos(-47, -56, 180);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    matchLoad.set(true);
    chassis.driveDistanceWithOdomTime(-5, 1000); 
    

    //GRAB 1 BLUE BALL
    chassis.turnToAngle(41);
    chassis.setDriveMaxVoltage(10);
    chassis.movetopos(-14, -25, 41);
    chassis.setDriveMaxVoltage(8);
    mainIntake.stop();
    colorSort.stop();
    chassis.turnToAngle(90);
    matchLoad.set(false);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    chassis.setDriveMaxVoltage(10);
    chassis.movetopos(27,-24, 90);
    chassis.setDriveMaxVoltage(8);
    toggleLift(); //DOWN
    wait(.1, sec);

    //LOAD 7 BALLS
    matchLoad.set(true);
    chassis.turnToAngle(318);
    chassis.movetopos(15, -22.3, 318);
    toggleIntakeFlap();
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 36, percent); //65 // 55 // 45
    wait(1.4, sec); //1.2
    chassis.driveDistanceWithOdom(1);
    topStage.spin(reverse);
    bottomStage.spin(reverse);
    wait(.2, sec);
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 30, percent); // 55 // 45 // 36
    wait(2.3, sec); // 1.8 before change
    chassis.driveDistanceWithOdom(-3);
    chassis.driveDistanceWithOdom(3);
    
    matchLoad.set(false);
    
    mainIntake.spin(reverse, 100, pct);
    colorSort.spin(reverse, 100, pct);
    topStage.spin(reverse, 100, pct);
    chassis.driveDistanceWithOdom(-45);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();

    chassis.turnToAngle(180);
    toggleIntakeFlap();


    chassis.movetopos(48, -61, 180);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    matchLoad.set(true);
    chassis.driveDistanceWithOdomTime(-5, 1000); 

    colorSort.stop();
    topStage.stop();
    chassis.driveDistanceWithOdom(-8); //11  // -10 //9.5
    chassis.turnToAngle(90);
    matchLoad.set(false);
    chassis.movetopos(66, -52.5, 90);
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(2,sec); //1.7
    std::cout << "X POS: " << chassis.chassisOdometry.getXPosition()
              << " Y POS: " << chassis.chassisOdometry.getYPosition() 
              << " HEADING: " << chassis.chassisOdometry.getHeading() << std::endl;

    chassis.driveDistanceWithOdom(-5); // -14.6
    
    chassis.turnToAngle(270); // 90
    toggleLift();
    matchLoad.set(false);

    chassis.movetopos(27, -52, 270); // (11.5 + 9.6)
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
    wait(1.7,sec); // 2 // 2.5
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();


    chassis.driveDistanceWithOdom(-25);
    toggleIntakeFlap(); // close
    chassis.movetopos(-10, -67, 270);
    chassis.turnToAngle(0);
    toggleLift();
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(7);
    matchLoad.set(true);
    chassis.driveDistanceWithOdom(-7);
    chassis.turnToAngle(270);
    chassis.movetopos(-40,-67, 270);
    matchLoad.set(false);
    

    chassis.turnToAngle(315);
    chassis.driveDistanceWithOdom(16);
    chassis.turnToAngle(270);
    chassis.movetopos(-64, -53, 270);
     matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec);


    chassis.moveable();
    wait(100, sec);


    
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec);
    
    colorSort.stop();
    topStage.stop();
    

    //LOAD 8 INTO CLOSE LONG GOAL SIDE
    chassis.driveDistanceWithOdom(-15); 
    chassis.turnToAngle(273); // 270 // 272
    wait(.2, sec);
    chassis.turnToAngle(273);

    mainIntake.stop();
    matchLoad.set(false);

    chassis.driveDistanceWithOdomTime(12.5, 1000); // 16 before change
    toggleIntakeFlap(); //open

    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward, 20, percent);

    chassis.driveDistanceWithOdomTime(3, 1000);
    topStage.spin(forward, 100, percent);
    wait(0.1,sec);
    
    topStage.spin(forward, 100, percent);
    colorSort.spin(forward);
    mainIntake.spin(forward);
    wait(1.5, sec);

    mainIntake.spin(reverse);
    colorSort.spin(reverse);
    topStage.spin(reverse);
    wait(0.25, sec);

    topStage.spin(forward, 100, percent);
    colorSort.spin(forward);
    mainIntake.spin(forward);
    wait(1.5, sec);

    mainIntake.stop();
    colorSort.stop();
    topStage.stop();

    //PARK
    // UNTESTED
    chassis.driveDistanceWithOdom(-7);

    // if not enough time
    // chassis.turnToAngle(145); // no idea if thats close yet
    // chassis.driveDistanceWithOdom(27.5);
    // chassis.turnToAngle(180);
    
   
    // chassis.driveDistanceWithOdom(13);
    // toggleLift() // should be up idk
    //chassis.driveDistanceWithOdom(10);


    //if enough time
    // chassis.turnToAngle(215); // 225
    // chassis.driveDistanceWithOdom(44);
    // wait(2, sec); // need to block
    // chassis.driveDistanceWithOdom(-60);
    // chassis.turnToAngle(172); // no idea if thats close yet
    // chassis.setDriveMaxVoltage(10); // speeding up
    // chassis.driveDistanceWithOdom(35);


    

}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4() //TOP MIDDLE DEFENSE
{
  //SET UP
  mainIntake.setVelocity(100, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);
  bottomStage.setVelocity(100, percent);
  chassis.setPosition(-46,-8,0);
  chassis.setDriveMaxVoltage(10);
  chassis.setTurnMaxVoltage(8);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdom(-38);
  chassis.setDriveMaxVoltage(8);
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(-61.5, -43.6, 270);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdom(-6);
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
  chassis.driveDistanceWithOdom(6);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);
  chassis.driveDistanceWithOdom(-6);

  //LOAD INTO LONG GOAL
  chassis.turnToAngle(90);
  wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-23, -45, 90);
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
  chassis.setDriveMaxVoltage(12);
  toggleLift();
  chassis.driveDistanceWithOdom(-20);
  chassis.turnToAngle(65);
  chassis.movetopos(17,-17, 65);

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
  chassis.setDriveMaxVoltage(10);
  chassis.setTurnMaxVoltage(8);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdom(-38);
  chassis.setDriveMaxVoltage(8);
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(-61.5, -43.6, 270);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdom(-6);
  chassis.turnToAngle(200);
  wait(.1, sec);
  mainIntake.spin(reverse, 50, percent);
  wait(.8, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //LOAD INTO LONG GOAL
  chassis.turnToAngle(90);
  wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-24.5, -45, 90);
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
  toggleIntakeFlap(); //CLOSE

  //GRAB MATCH LOADS
  chassis.driveDistanceWithOdomTime(-6, 1000); // -7
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); //DOWN
  wait(.1, sec);
  chassis.driveDistanceWithOdom(-6);
  chassis.turnToAngle(271);
  chassis.movetopos(-65.5, -43, 271);
  matchLoad.set(true);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(3.5,sec);
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //LOAD TOP GOAL AGAIN
  chassis.driveDistanceWithOdomTime(-6, 1000);
  chassis.turnToAngle(90);
  wait(.1, sec);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-24.5, -45, 90);
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
  chassis.setDriveMaxVoltage(10);
  chassis.setTurnMaxVoltage(8);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdom(-38);
  chassis.setDriveMaxVoltage(8);
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(-61.5, -43.6, 270);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdom(-6);
  chassis.turnToAngle(200);
  wait(.1, sec);
  mainIntake.spin(reverse, 50, percent);
  wait(.8, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //LOAD INTO LONG GOAL
  chassis.turnToAngle(90);
  wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-24.5, -45, 90);
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
  toggleIntakeFlap(); //CLOSE

  //GRAB MATCH LOADS
  chassis.driveDistanceWithOdomTime(-6, 1000); // -7
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); //DOWN
  wait(.1, sec);
  chassis.driveDistanceWithOdom(-6);
  chassis.turnToAngle(270);
  chassis.movetopos(-65.5, -43, 270);
  matchLoad.set(true);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(3.5,sec);
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //LOAD MIDDLE
  chassis.driveDistanceWithOdom(-4);
  chassis.turnToAngle(36);
  wait(.1, sec);
  chassis.setDriveMaxVoltage(10);
  chassis.movetopos(-16, 17, 36);
  chassis.turnToAngle(138);
  chassis.driveDistanceWithOdom(5);
  toggleIntakeFlap();
  mainIntake.spin(forward, 100, percent);
  colorSort.spin(forward, 100, percent);
  topStage.spin(forward, 43, percent); //65 // 55 // 45
  wait(1.4, sec); //1.2
  topStage.spin(reverse);
  bottomStage.spin(reverse);
  wait(0.20, sec);
  mainIntake.spin(forward, 100, percent);
  colorSort.spin(forward, 100, percent);
  topStage.spin(forward, 36.5, percent); // 55 // 45 // 36
  wait(2.3, sec); // 1.8 before change

  
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
  chassis.setDriveMaxVoltage(10);
  chassis.setTurnMaxVoltage(8);
  toggleLift();
  wait(.2, sec);
  toggleLift();

  //GRAB MATCH LOAD
  matchLoad.set(true);
  mainIntake.spin(forward);

  //DRIVE TO MATCH LOAD
  chassis.driveDistanceWithOdom(-38);
  chassis.setDriveMaxVoltage(8);
  chassis.turnToAngle(270);
  wait(.1, sec);
  mainIntake.stop();
  matchLoad.set(false);
  chassis.movetopos(-61.5, -43.6, 270);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  matchLoad.set(true);
  wait(2, sec);

  //OUTTAKE 3 BALLS
  chassis.driveDistanceWithOdom(-6);
  chassis.turnToAngle(200);
  wait(.1, sec);
  mainIntake.spin(reverse, 50, percent);
  wait(.8, sec);
  mainIntake.stop();
  mainIntake.spin(forward);

  //LOAD INTO LONG GOAL
  chassis.turnToAngle(90);
  wait(.1, sec);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  matchLoad.set(false);
  toggleLift();
  wait(.1, sec);
  chassis.movetopos(-24.5, -45, 90);
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
  toggleIntakeFlap(); //CLOSE

  //GRAB MATCH LOADS
  chassis.driveDistanceWithOdomTime(-6, 1000); // -7
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  toggleLift(); //DOWN
  wait(.1, sec);
  chassis.driveDistanceWithOdom(-6);
  chassis.turnToAngle(270);
  chassis.setDriveMaxVoltage(8);
  chassis.movetopos(-65.5, -43, 270);
  matchLoad.set(true);
  mainIntake.spin(forward);
  colorSort.spin(forward);
  topStage.spin(forward);
  wait(3.5,sec);
  matchLoad.set(false);
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();

  //DRIVE TO MIDDLE AND SCORE
  chassis.driveDistanceWithOdom(-4);
  chassis.turnToAngle(45);
  wait(.1, sec);
  chassis.movetopos(-10,-10,46);
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