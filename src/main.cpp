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

void toggleLift();
void toggleIntakeFlap();
void slowIntake();

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


  Auton_1();
  //Auton_2();
  //Auton_3();
  //Auton_4();

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

  mainIntake.setVelocity(85, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);

  Controller1.ButtonL1.pressed(toggleLift);
  Controller1.ButtonUp.pressed(toggleIntakeFlap);
  Controller1.ButtonDown.pressed(slowIntake);

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
      if(flapState){
        topStage.spin(forward);
      }else{
        topStage.stop();
      }
      if(lastSeen == teamColor){
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
      if(lastSeen == teamColor){
        colorSort.spin(forward);
      }else{
        colorSort.spin(reverse);
      }
    }else if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
      mainIntake.spin(forward);
      topStage.spin(forward);
      flapState = true;
      if(lastSeen == teamColor){
        colorSort.spin(forward);
      }else{
        colorSort.spin(reverse);
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
    Brain.Screen.print("color: ", lastSeen);

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
        0.5,  // Kp - Proportion Constant
        0.0001, // Ki - Integral Constant
        0.9, // Kd - Derivative Constant
        .25, // Settle Error
        200, // Time to Settle
        1000 // End Time 5000
    );  

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        0.25,    // Kp - Proportion Constant
        0.000,      // Ki - Integral Constant
        1.4,      // Kd - Derivative Constant 
        0.2,    // Settle Error
        200,    // Time to Settle
        1000    // End Time
    );
    
}





//Auton Route Functions
/// @brief Auton Slot 1 - Write code for route within this function.
void Auton_1()
{
    Brain.Screen.print("Skills 1 running.");
    chassis.setTurnMaxVoltage(8);
    chassis.setPosition(-46,15,0);
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);



    //Pencers        matchLoad.set(true);
    //Intakes        
      //Main/Pencers mainIntake.spin(fwd);
      //Top          topStage.spin(fwd);
      //Color        colorSort.spin(fwd);
      //Bottom       bottomStage.spin(fwd);
    //Outake         toggleLift();
    //Dropdown       toggleDropDown();
    //flap           toggleIntakeFlap();


    


    //First T
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
    chassis.driveDistanceWithOdom(56); //50 to short, 56 hits wall every 3 times or so
    wait(0.5, sec);
    matchLoad.set(true);
      //wait(0.2, sec); 
    //Goes for matchload
    chassis.driveDistanceWithOdom(-14); //13
    toggleLift();
    // matchLoad.set(false);
    chassis.turnToAngle(270);
    matchLoad.set(false);
    chassis.driveDistanceWithOdom(12); //1.5 worked before
    matchLoad.set(true);
      wait(1.7, sec);
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
      matchLoad.set(false);    
    chassis.driveDistanceWithOdom(-14); //was -13.5
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdom(16.5);
    //Load long with 8
      toggleIntakeFlap();
      wait(0.05, sec);
      topStage.spin(reverse);
      wait(0.1, sec);
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      wait(1.7, sec);
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();

    chassis.driveDistanceWithOdom(-8.5);
    toggleIntakeFlap(); // down
    chassis.turnToAngle(45);
    chassis.driveDistanceWithOdom(17);
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdom(32);
    //Grabs 2 from bellow
    chassis.turnToAngle(180);
    //Drop Down bar
    wait(1, sec);
    chassis.turnToAngle(90);
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
    chassis.driveDistanceWithOdom(28);
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();

    // getting 3 blue and 3 red from wall intake
    chassis.turnToAngle(130); //123
    chassis.driveDistanceWithOdom(22); //24 overshoots
    chassis.turnToAngle(90);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(14); //5 unders
    matchLoad.set(true);
    wait(1.7, sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false);

    // scoring 5 blue and 2 red
    chassis.driveDistanceWithOdom(-4);
    chassis.turnToAngle(270); //WRONG ANGLE
    toggleIntakeFlap();
    chassis.driveDistanceWithOdom(30);
    topStage.spin(reverse);
    wait(0.2, sec);
    topStage.spin(forward);
    colorSort.spin(forward);
    mainIntake.spin(forward);
    wait(1.7, sec);

  


    //goes for wall balls
    chassis.driveDistanceWithOdom(-15); //-17 perfectly aligns
    toggleIntakeFlap();
    chassis.turnToAngle(0);
    chassis.driveDistanceWithOdom(13); //may be to close
    chassis.driveDistanceWithOdom(-2);
    matchLoad.set(true);
    chassis.turnToAngle(5);
    matchLoad.set(false);
    chassis.driveDistanceWithOdom(-62);
    chassis.turnToAngle(90);



}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2()
{
    Brain.Screen.print("Skills 2 running.");

    //SETUP
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(-46,0,90);
    chassis.setDriveMaxVoltage(10);
    chassis.setTurnMaxVoltage(8);

    //GRAB 4 BLUE START BALLS
    wait(.01, sec); //DROP BAR
    chassis.driveDistanceWithOdom(-10);
    
    //GRAB 2 BLUE WALL BALLS
    chassis.turnToAngle(12); // 10
    chassis.driveDistanceWithOdom(59);
    chassis.turnToAngle(0);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    chassis.driveDistanceWithOdom(18); //driveDistance
    matchLoad.set(true);
    // wait(.25, sec);
    chassis.driveDistanceWithOdom(-5); //driveDistance
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    matchLoad.set(false);

    //GRAB 1 BLUE BALL
    chassis.turnToAngle(208); 
    chassis.driveDistanceWithOdom(38.1); //39.5
    chassis.turnToAngle(270);
    mainIntake.spin(forward);
    chassis.driveDistanceWithOdom(49); // 47

    //PUT 7 BALLS IN TOP MIDDLE
    chassis.turnToAngle(136); // 134
    chassis.driveDistanceWithOdom(10); //driveDistance // 15 11 is sometimes too far
    toggleIntakeFlap(); 
    mainIntake.spin(forward, 65, percent);
    colorSort.spin(forward, 65, percent);
    topStage.spin(forward, 65, percent);
    wait(1.7, sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleIntakeFlap(); // closing after it scores


    //UNTESTED FROM HERE ON SO FARRRRRRRR
    //GRAB 2 RED BALLS
    chassis.driveDistanceWithOdom(-43); // -42
    chassis.turnToAngle(0);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(23); //driveDistance 24
    //matchLoad.set(true);
    // wait(.25, sec);
    matchLoad.set(true);
    chassis.driveDistanceWithOdom(-5); //driveDistance
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();

    //GRAB 6 FROM BOTTOM FAR MATCH LOADER
    // using driveDistanceWithOdom
    chassis.driveDistanceWithOdom(-11.5); // -10 //driveDistance
    chassis.turnToAngle(270);
    toggleLift();
    chassis.driveDistanceWithOdom(16.5); // 15
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.8,sec); //1.7
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false);

    //LOAD 8 INTO FAR LONG GOAL SIDE
    chassis.driveDistanceWithOdom(-15); 
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdom(20);
    toggleIntakeFlap(); //open
    bottomStage.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(2,sec);
    bottomStage.stop();
    colorSort.stop();
    topStage.stop();

    //GRAB 2 RED FROM CENTER UNDER GOAL
    chassis.driveDistanceWithOdom(-10);
    toggleIntakeFlap();
    chassis.turnToAngle(0);
    chassis.driveDistanceWithOdom(14); // 18
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdom(44); //49
    chassis.turnToAngle(180);
    chassis.driveDistanceWithOdom(5);
    chassis.driveDistanceWithOdom(-5);
    chassis.turnToAngle(90);


    //GRAB 6 FROM CLOSE MATCH LOADER
    chassis.driveDistanceWithOdom(23);
    chassis.turnToAngle(124);
    chassis.driveDistanceWithOdom(29);
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdom(7); // 5
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1.7,sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false);

    //LOAD 8 INTO CLOSE LONG GOAL SIDE
    chassis.driveDistanceWithOdom(-15); 
    chassis.turnToAngle(270);
    chassis.driveDistanceWithOdom(20);
    toggleIntakeFlap(); //open
    bottomStage.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(2,sec);
    bottomStage.stop();
    colorSort.stop();
    topStage.stop();

    //PARK
    chassis.driveDistanceWithOdom(-7);

    //if enough time
    chassis.turnToAngle(215); // 225
    chassis.driveDistanceWithOdom(44);
    wait(1, sec); // need to block
    chassis.driveDistanceWithOdom(-60);
    chassis.turnToAngle(172); // no idea if thats close yet
    chassis.setDriveMaxVoltage(10); // speeding up
    chassis.driveDistanceWithOdom(35);


    // if not enough time
    // chassis.turnToAngle(145); // no idea if thats close yet
    // chassis.driveDistanceWithOdom(27.5);
    // chassis.turnToAngle(180);
    // speeding up, idk if i did this right
    // chassis.driveDistanceWithOdom(13);


    //CLEAR FOR KEVIN
    wait(3, sec);
    mainIntake.spin(reverse);
    colorSort.spin(reverse);
    topStage.spin(reverse);
    wait(1.5, sec);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3()
{
    Brain.Screen.print("Auton 3 running.");
    //KEEGAN WRITE HERE

    //SETUP
    mainIntake.setVelocity(85, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    chassis.setPosition(-46,10.5,180);
    chassis.setTurnMaxVoltage(8);
    

    //MOVE FORWARD AND INTAKE 2 RED
    chassis.driveDistanceWithOdom(10);
    wait(1, sec); //intake 2 red preload
    
    //GRAB 8 FROM MATCH LOADER 1
    chassis.driveDistanceWithOdom(-47);
    chassis.turnToAngle(270);
    toggleLift();
    wait(.5, sec);
    chassis.driveDistanceWithOdom(14.5);
    matchLoad.set(true);
    mainIntake.spin(fwd);
    colorSort.spin(fwd);
    topStage.spin(fwd);
    wait(3, sec);

    matchLoad.set(false);
    mainIntake.stop();
    topStage.stop();
    colorSort.stop();

    //chassis.driveDistanceWithOdom(-10);
    //chassis.driveDistanceWithOdom(-36.2);
    //chassis.moveToPosition(-58, 46.7);

    //LOAD 7 INTO LONG GOAL 1 (5RED, THEN 2BLUE)
    chassis.driveDistanceWithOdom(-13);
    chassis.turnToAngle(90);
    //toggleLift();
    toggleIntakeFlap();
    wait(.5, sec);
    chassis.driveDistanceWithOdom(18);
    mainIntake.spin(fwd);
    colorSort.spin(fwd);
    topStage.spin(fwd);
    wait(10,sec);
    toggleIntakeFlap();
    mainIntake.stop();
    topStage.stop();
    colorSort.stop();
    // chassis.driveDistanceWithOdom(-5);
    // chassis.moveToPosition(-32, 46.7);
    // wait(-1, sec);

    // //GRAB 2 BLUE AT TOP
    chassis.driveDistanceWithOdom(-18);
    toggleLift();
    chassis.turnToAngle(0);
    chassis.driveDistanceWithOdom(15);
    // chassis.driveDistanceWithOdom(-14);
    // chassis.moveToPosition(-46, 62.5);
    // wait(1, sec);

    // //GRAB 4 FROM START
    // chassis.driveDistanceWithOdom(-10);
    // chassis.moveToPosition(-30,0);
    // chassis.moveToPosition(-46,0);
    // wait(1, sec);

    // //LOAD INTO UPPER GOAL
    // chassis.driveDistanceWithOdom(-10);
    // chassis.moveToPosition(-17.5, 18.5);
    // chassis.moveToPosition(-13, 13.5);
    // wait(-1, sec);

    // //BLOCK LOWER MIDDLE GOAL
    // chassis.driveDistanceWithOdom(-10);
    // chassis.moveToPosition(-19.6, -4.9);
    // chassis.turnToAngle(140);

    // //GRAB 8 FROM MATCH LOADER 2
    // chassis.moveToPosition(-47, -47);
    // chassis.moveToPosition(-58, -47);
    // wait(1, sec);

    // //GRAB 2 BLUE FROM BOTTOM
    // chassis.driveDistanceWithOdom(-11);
    // chassis.moveToPosition(-47, -62.5);
    // wait(1, sec);

    // //LOAD IN TO BOTTOM LONG GOAL 2
    // chassis.driveDistanceWithOdom(-15.5);
    // chassis.moveToPosition(-31.7, -47);
    // wait(-1, sec);

    // //RAM INTO PARK ZONE
    // chassis.driveDistanceWithOdom(-20);
    // chassis.moveToPosition(-63.8, -8.2);




  



}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4()
{
    Brain.Screen.print("Auton 4 running.");
    chassis.setTurnMaxVoltage(8);

    ///////// SETTING UP FOR UNDER LONG GOAL PART /////////
    chassis.setPosition(-55.5, -17, 180); // starting position
    chassis.driveDistanceWithOdom(2); // to get away from park zone
    chassis.moveToPosition(-29.5, -60);

    ///////// GETTING 2 RED UNDER LONG GOAL /////////
    chassis.moveToPosition(-7.5, -60);
    chassis.moveToPosition(-7.5, -56);
    wait(1, sec); // grabs 2 red  

    ///////// GETTING 2 BLUE UNDER LONG GOAL /////////
    chassis.driveDistanceWithOdom(-4);
    chassis.moveToPosition(7.5, -60);
    chassis.moveToPosition(7.5, -56);
    wait(1, sec); // grabs 2 blue

    ///////// SETTING UP FOR MATCH LOADER PART /////////
    chassis.driveDistanceWithOdom(-4);
    chassis.moveToPosition(33, -60);

    ///////// GETTING 3 BLUE FROM MATCH LOADER /////////
    chassis.moveToPosition(51, -47);
    chassis.moveToPosition(57.5, -47);
    wait(1, sec); // intakes 3 blue

    ///////// SCORING INTO LONG GOAL /////////
    chassis.moveToPosition(32, -47);
    wait(1, sec); // scores 2 red, and 5 blue
    
    ///////// GETTING 3 RED FROM MATCH LOADER /////////
    chassis.moveToPosition(57.5, -47);
    wait(1, sec); // intakes 3 red
    chassis.driveDistanceWithOdom(-6.5);

    ///////// GETTING 4 RED FROM BLUE PARKING ZONE /////////
    chassis.moveToPosition(41, 0.5);
    chassis.moveToPosition(46, 0.5);
    wait(1, sec); // intakes 4 red
    chassis.driveDistanceWithOdom(-5);

    ///////// SCORING 7 RED  IN BOTTOM X /////////
    chassis.moveToPosition(20,21);
    chassis.moveToPosition(17.5, 18);
    wait(1, sec); // scores 7 red in bottom X

    ///////// GETTING 2 RED FROM WALL /////////
    chassis.driveDistanceWithOdom(-4);
    chassis.moveToPosition(46.5, 47);
    chassis.moveToPosition(46.5, 59.5);
    wait(1, sec); // intakes two red

    ///////// GETTING ALL BALLS FROM MATCH LOADER /////////
    chassis.driveDistanceWithOdom(-12.5);
    chassis.moveToPosition(57.5, 47);
    wait(1, sec); // intakes 3 blue and 3 red

    ///////// SCORING INTO LONG GOAL /////////
    chassis.driveDistanceWithOdom(-4.5);
    chassis.moveToPosition(32, 47);
    wait(1, sec); // scores 2 red, 3 blue, and 3 red into long goal

    ///////// PARK TIME /////////
    chassis.driveDistanceWithOdom(-4);
    chassis.moveToPosition(32, 34.5);
    chassis.moveToPosition(-60, 34.5);
    // chassis.turnToAngle(180);
    chassis.moveToPosition(-62, 7.5);

    








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
  isInAuton = true;
  rotation1.resetPosition();
  rotation2.resetPosition();
  inertial1.resetHeading();
  wait(100, msec);

  chassis.setDriveConstants(
        0.7,  // Kp - Proportion Constant
        0.0003, // Ki - Integral Constant
        0.01, // Kd - Derivative Constant was 0.17
        .2, // Settle Error
        300, // Time to Settle
        3000 // End Time 5000
  );

  chassis.driveDistanceWithOdom(72);
  chassis.turn(90);
  chassis.driveDistanceWithOdom(72);
  chassis.turn(90);
  chassis.driveDistanceWithOdom(72);
  chassis.turn(90);
  chassis.driveDistanceWithOdom(72);
  chassis.turn(90);
    
}