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
  std::string names[8] = {"Auton 1", "1mSkill", "Auton 3", "Auton 4", 
                          "Split", "TopMid", "LowMid", "Auton 8"};
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

  switch (lastPressed) 
  {
    case 0:
      Auton_1();
      break;
    case 1:
      Auton_2();
      break;
    case 2:
      Auton_3;
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
  }

}

/// @brief Runs during the UserControl section of the competition
void usercontrol() 
{
  drawSponsors();
 
  // User control code here, inside the loop
  bool flapState = false;
  int lastSeen = teamColor;

  //CHANGE IF NOT COLOR SORTING
  isColorSorting = true;

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
      topStage.spin(forward);
      if(flapState){
        topStage.spin(forward);
      }else{
        topStage.stop();
      }
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
void Auton_1()
{
    Brain.Screen.print("Auton 1.");

}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2()
{
    Brain.Screen.print("Skills running.");
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";
    //SETUP
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(0,0,90);
    chassis.setDriveMaxVoltage(10);
    chassis.setTurnMaxVoltage(8);

    //GRAB 4 BLUE START BALLS
    toggleLift(); //UP
    wait(0.25, sec);
    toggleDropDown(); // down
    wait(.4, sec); // .5
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    chassis.driveDistanceWithOdom(-15);
    chassis.driveDistanceWithOdom(5);
    matchLoad.set(true);

    std::cout << "POINT 1: " << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() << std::endl;
    
    //GRAB 2 BLUE WALL BALLS
    chassis.turnToAngle(15);
    chassis.driveDistanceWithOdom(46);
    std::cout << "POINT 2: " << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() << std::endl;
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleDropDown(); // up

    chassis.turnToAngle(0);
    //std::cout << "HEADING: " << chassis.chassisOdometry.getHeading() << std::endl;
    mainIntake.spin(forward);
    colorSort.spin(forward);
    chassis.driveDistanceWithOdomTime(16, 1000);//14 to short
    matchLoad.set(true);
    chassis.driveDistanceWithOdomTime(-5, 1000); 
    // matchLoad.set(false);
    // mainIntake.stop();
    // colorSort.stop();


    //GRAB 1 BLUE BALL
    chassis.turnToAngle(208);
    std::cout << "HEADING: " << chassis.chassisOdometry.getHeading() << std::endl;
    // wait(50, msec);
    chassis.driveDistanceWithOdom(39.2); //38.1 39.5 // 39.3
    std::cout << "HEADING: " << chassis.chassisOdometry.getHeading() << std::endl;
    // wait(50, msec);
    mainIntake.stop();
    colorSort.stop();
    // matchLoad.set(false);
    chassis.turnToAngle(270);
    matchLoad.set(false);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    chassis.driveDistanceWithOdomSettle(48.76, 400, .4); // 47.5 // 48.76

    // wait(0.2, sec);


    
    //PUT 7 BALLS IN TOP MIDDLE
    
    chassis.turnToAngle(134); // 134
    topStage.setBrake(hold);
    toggleDropDown(); // down
    wait(.5, sec);
    toggleLift(); // down
    wait(0.5, sec);
    toggleIntakeFlap(); 

    
    chassis.driveDistanceWithOdomTime(5, 1000); //7 // 6.75

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

    // chassis.driveDistanceWithOdom(1);
    // wait(0.1, sec);


    // mainIntake.stop();
    // colorSort.stop();
    // topStage.stop();
    


    //GRAB 2 RED BALLS FROM WALL
    chassis.driveDistanceWithOdom(-42); // -42 // -43 // -41
    toggleIntakeFlap(); // down

    // Moving this above (-41)
    // mainIntake.stop();
    // colorSort.stop();
    // topStage.stop();
    // toggleIntakeFlap(); // down

    toggleLift();
    toggleDropDown(); // up
    chassis.turnToAngle(0);
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward,100, percent);
    topStage.spin(forward, 100, percent);
    chassis.driveDistanceWithOdomTime(21, 1000); // 23 //22 //21
    //matchLoad.set(true);
    wait(0.5, sec);
    // toggleDropDown(); // down
    matchLoad.set(true);
  
    chassis.driveDistanceWithOdom(-5); //driveDistance
    //toggleDropDown(); // up

    // mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    // matchLoad.set(false);



    //GRAB 6 FROM BOTTOM FAR MATCH LOADER
    chassis.driveDistanceWithOdom(-9); //11  // -10 //9.5
    chassis.turnToAngle(270);
    matchLoad.set(false);
    
    // toggleLift();

    chassis.driveDistanceWithOdomTime(15.7, 1000); // 15 // 14 //13.9
    wait(0.4, sec);

    topStage.stop();

    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    
    wait(2,sec); //1.7
    
    colorSort.stop();
    // topStage.stop();
    


    //LOAD 8 INTO FAR LONG GOAL SIDE
    chassis.driveDistanceWithOdomTime(-5, 1000); // -14.6
    
    chassis.turnToAngle(92); // 90
    
    // mainIntake.stop();
    matchLoad.set(false);

    chassis.driveDistanceWithOdomTime(21.1, 1000); // (11.5 + 9.6)
    toggleIntakeFlap(); //open
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward, 20, percent);

    wait(0.3, sec);
    chassis.driveDistanceWithOdomTime(3, 1000);

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

    //GRAB 2 RED FROM CENTER UNDER GOAL
    chassis.driveDistanceWithOdom(-10);
    toggleIntakeFlap(); // close
    chassis.turnToAngle(0);
    chassis.driveDistanceWithOdom(12.5); // 14 // 13.5 
    chassis.turnToAngle(88); // 90 // 87
    chassis.driveDistanceWithOdom(47.7); //49 // 46.5 // 47
    chassis.turnToAngle(180);
    chassis.driveDistanceWithOdomTime(5, 1000);
    toggleDropDown(); // down
    wait(0.25,sec);
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);

    
    chassis.driveDistanceWithOdomTime(-5, 1000);
    chassis.turnToAngle(90); 


    //GRAB 6 FROM CLOSE MATCH LOADER
    chassis.driveDistanceWithOdom(22); // 23
    
    toggleDropDown(); //up
    topStage.stop();
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    bottomStage.stop();

    chassis.turnToAngle(124); // 124
    chassis.driveDistanceWithOdom(29); // 27 // 29.5
    chassis.turnToAngle(90);
    chassis.driveDistanceWithOdomTime(9.5, 1000); //5 //8.5

    
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

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3()
{
    Brain.Screen.print("Auton 3 running.");
}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4()
{
    Brain.Screen.print("Auton 4 running.");
}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5() // SLPITS BLOCKS BETWEEN HIGH AND LOW
{
    chassis.moveable();

    Brain.Screen.print("Match Auton Right");
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(-46,-8,0);
    chassis.setDriveMaxVoltage(10);
    chassis.setTurnMaxVoltage(8);

    //Grab Other Bots Match Load
    matchLoad.set(true);
    mainIntake.spin(forward);
    chassis.driveDistanceWithOdom(-39);
    chassis.turnToAngle(272);
    mainIntake.stop();
    matchLoad.set(false);

    //Intake 6 From Match Load
    chassis.driveDistanceWithOdomTime(14, 1000);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    matchLoad.set(true);
    wait(2, sec);

    //Outtake 3 Blue
    chassis.driveDistanceWithOdom(-5);
    chassis.turnToAngle(200);
    mainIntake.spin(reverse, 50, percent);
    wait(.8, sec);
    mainIntake.stop();
    mainIntake.spin(forward);
    
    //Load Into 5 Red Into Long Goal
    chassis.turnToAngle(93);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false);
    toggleLift(); //UP
    chassis.driveDistanceWithOdomTime(26, 1000);// 25
    toggleIntakeFlap(); //OPEN
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1, sec);
    mainIntake.spin(reverse);
    wait(.2, sec);
    mainIntake.spin(forward);
    wait(1.7, sec); // 1.5

    //Grab 7 Match Loads
    toggleIntakeFlap(); //CLOSE
    chassis.driveDistanceWithOdom(-5); // -7
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleLift(); //DOWN
    chassis.turnToAngle(272); // 270
    wait(0.2, sec);
    chassis.turnToAngle(272);
    chassis.driveDistanceWithOdomTime(26, 1000); // 24
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(4.0,sec);
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();


   
    //LOAD 7 MATCH LAODS INTO BOTTOM MIDDLE GOAL
    chassis.driveDistanceWithOdom(-5);
    chassis.turnToAngle(49);
    chassis.moveable();
    chassis.movetopos(-8.78, -7.99, 47.8);
    chassis.driveDistanceWithOdom(53); //55
    mainIntake.spin(reverse, 17, percent); // 15 // 35
    colorSort.spin(reverse, 95, percent); // 90 // 100
    topStage.spin(reverse, 100, percent);
    wait(1, sec); // 2 before change
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 100, percent);
    wait(.5, sec); 
    mainIntake.spin(reverse, 17, percent); // 15 // 35
    colorSort.spin(reverse, 95, percent); // 90 // 100
    topStage.spin(reverse, 100, percent);
    wait(1.5, sec); // 3 before change
    mainIntake.spin(reverse, 15, percent);
    topStage.spin(reverse, 100, percent);
    wait(2, sec);

    // going to top middle
    chassis.driveDistanceWithOdom(-6);
    chassis.turnToAngle(0);
    chassis.driveDistanceWithOdom(10);
    chassis.turnToAngle(134);
    topStage.setBrake(hold);
    // toggleDropDown(); // down
    wait(.5, sec);
    // toggleLift(); // down
    wait(0.5, sec);
    toggleIntakeFlap(); //up
    chassis.driveDistanceWithOdomTime(14, 1000); // 15
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 43, percent); //65 // 55 // 45
    wait(1, sec); //1.4 before change
    topStage.spin(reverse);
    bottomStage.spin(reverse);
    wait(0.20, sec);
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 36.5, percent); // 55 // 45 // 36
    wait(1.5, sec); // 2.3 before change


    
}

/// @brief Auton Slot 6 - Write code for route within this function.

void Auton_6() // SCORES TOP MIDDLE
{
    Brain.Screen.print("Match Auton Right");
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(-46,-8,0);
    chassis.setDriveMaxVoltage(10);
    chassis.setTurnMaxVoltage(8);

    //Grab Other Bots Match Load
    matchLoad.set(true);
    mainIntake.spin(forward);
    chassis.driveDistanceWithOdom(-39);
    chassis.turnToAngle(272);
    mainIntake.stop();
    matchLoad.set(false);

    //Intake 6 From Match Load
    chassis.driveDistanceWithOdomTime(14, 1000);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    matchLoad.set(true);
    wait(2, sec);

    //Outtake 3 Blue
    chassis.driveDistanceWithOdomTime(-5, 1000);
    chassis.turnToAngle(200);
    mainIntake.spin(reverse, 50, percent);
    wait(.9, sec);
    mainIntake.stop();
    mainIntake.spin(forward);
    
    //Load Into 5 Red Into Long Goal]
    chassis.turnToAngle(95);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false);
    toggleLift(); //UP
    chassis.driveDistanceWithOdomTime(26, 1000);// 25
    toggleIntakeFlap(); //OPEN
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1, sec);
    mainIntake.spin(reverse);
    wait(.2, sec);
    mainIntake.spin(forward);
    wait(1.7, sec); // 1.5

    //Grab 7 Match Loads
    toggleIntakeFlap(); //CLOSE
    chassis.driveDistanceWithOdomTime(-6, 1000); // -7
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleLift(); //DOWN
    chassis.turnToAngle(272); // 270
    chassis.driveDistanceWithOdomTime(25, 1000); // 24
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(2.5,sec);
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();

 
    // LOADS TOP MIDDLE INSTEAD
    chassis.driveDistanceWithOdomTime(-5, 1000);
    chassis.turnToAngleTime(22, 1000, 8);
    chassis.driveDistanceWithOdom(78); //80
    chassis.turnToAngle(134);
    topStage.setBrake(hold);
    // toggleDropDown(); // down
    wait(.5, sec);
    // toggleLift(); // down
    wait(0.5, sec);
    toggleIntakeFlap(); //up
    chassis.driveDistanceWithOdomTime(18, 1000); // 15
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

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7() // SCORES LOW MIDDLE
{
    //Setup
    // Brain.Screen.print("Match Auton Right");
    mainIntake.setVelocity(100, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
    bottomStage.setVelocity(100, percent);
    chassis.setPosition(-46,-8,0);
    chassis.setDriveMaxVoltage(10);
    chassis.setTurnMaxVoltage(8);

    //Grab Other Bots Match Load
    matchLoad.set(true);
    mainIntake.spin(forward);
    chassis.driveDistanceWithOdom(-39);
    chassis.turnToAngle(272);
    mainIntake.stop();
    matchLoad.set(false);

    //Intake 6 From Match Load
    chassis.driveDistanceWithOdomTime(14, 1000);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    matchLoad.set(true);
    wait(2, sec);

    //Outtake 3 Blue
    chassis.driveDistanceWithOdomTime(-5, 1000);
    chassis.turnToAngle(200);
    mainIntake.spin(reverse, 50, percent);
    wait(.9, sec);
    mainIntake.stop();
    mainIntake.spin(forward);
    
    //Load Into 5 Red Into Long Goal]
    chassis.turnToAngle(95);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    matchLoad.set(false);
    toggleLift(); //UP
    chassis.driveDistanceWithOdomTime(26, 1000);// 25
    toggleIntakeFlap(); //OPEN
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(1, sec);
    mainIntake.spin(reverse);
    wait(.2, sec);
    mainIntake.spin(forward);
    wait(1.7, sec); // 1.5

    //Grab 7 Match Loads
    toggleIntakeFlap(); //CLOSE
    chassis.driveDistanceWithOdomTime(-6, 1000); // -7
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();
    toggleLift(); //DOWN
    chassis.turnToAngle(274); // 270
    wait(0.2, sec);
    chassis.turnToAngle(274);
    chassis.driveDistanceWithOdomTime(27, 1000); // 24
    matchLoad.set(true);
    mainIntake.spin(forward);
    colorSort.spin(forward);
    topStage.spin(forward);
    wait(4.0,sec);
    matchLoad.set(false);
    mainIntake.stop();
    colorSort.stop();
    topStage.stop();


    //Load 7 Match Loads Into Bottom Middle Goal
    chassis.driveDistanceWithOdomTime(-5, 1000);
    chassis.turnToAngle(52); // 51
    wait(.3, sec);
    chassis.turnToAngle(52);

    std::cout << inertial1.heading(deg) << std::endl;

    //chassis.moveable();
    //chassis.movetopos(-8.78, -7.99, 47.8);
    chassis.driveDistanceWithOdom(55); //55
    /*colorSort.spin(forward, 100, percent);
    mainIntake.spin(reverse, 17, percent);
    wait(4, sec);
    topStage.spin(reverse, 100, percent);
    wait(8, sec);*/

    
    mainIntake.spin(reverse, 17, percent); // 15 // 35
    colorSort.spin(forward, 100, percent); // 90 // 100
    topStage.spin(reverse, 100, percent);
    wait(2, sec); // 4
    mainIntake.spin(forward, 100, percent);
    colorSort.spin(forward, 100, percent);
    topStage.spin(forward, 100, percent);
    wait(.2, sec);
    mainIntake.spin(reverse, 17, percent); // 15 // 35
    colorSort.spin(forward, 100, percent); // 90 // 100
    topStage.spin(reverse, 100, percent);
    wait(3, sec);
    chassis.driveDistanceWithOdom(-3);
    mainIntake.spin(reverse, 15, percent);
    topStage.spin(reverse, 100, percent);
    wait(5, sec);
    // chassis.driveDistanceWithOdom(-2);
    //colorSort.spin(reverse, 100, percent);
    //mainIntake.spin(reverse, 100, percent);
    wait(10, sec);

    
}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8()
{
  Brain.Screen.print("Auton 8 running.");
}