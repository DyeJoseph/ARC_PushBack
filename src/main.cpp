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
#include "semiPIDTuner.h"
#include "images.h"
#include "sensorConversion.h"

using namespace vex;

////////////////////////// GLOBAL VARIABLES //////////////////////////

  // Competition Instance
  competition Competition;

  int odomType = TWO_AT_45;

  bool isColorSorting = true;
  bool odomDebugEnabled = true;

  bool isInAuton = false;
  int lastPressed = 0;
  int teamColor = 0; //red = 0, blue = 1
  int driver = 0; //Elliot = 0, Jacob = 1

  bool liftState = 0;
  bool isFiring = 0;
  bool isSPRunning = false;


  // Define Values for the Chassis here:
  Drive chassis
  (
    motor_group(LT1, LT2, LT3, LT4, LT5), // Left drive train motors
    motor_group(RT1, RT2, RT3, RT4, RT5), // Right drive train motors
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
void odomDebugThread();
void semiPIDTest();

void toggleLift();
void toggleIntakeFlap();
void toggleFrontIntake();
void toggleColorSort();
void toggleWings();
int fireClock();
void SPClock();

//////////////////////////////////////////////////////////////////////


/// @brief Runs before the competition starts
void preAuton() 
{
  setDriveTrainConstants();
  bottomColorSort.integrationTime(10);
  bottomColorSort.setLight(ledState::on);
  bottomColorSort.brightness(true);

  chassis.brake(coast);       // make sure they aren’t holding weirdly
  chassis.driveMotors(0, 0);  

  enum preAutonStates{START_SCREEN = 0, SELECTION_SCREEN = 1};
  int currentScreen = START_SCREEN;

  // Calibrates/Resets the Brains sensors before the competition
  // inertial1.calibrate();
  // rotation1.resetPosition();
  // rotation2.resetPosition();

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
  // rotation1.resetPosition();
  // rotation2.resetPosition();
  // inertial1.resetHeading();

  int port = 9;
  vexGenericSerialEnable(port, 0);
  vexGenericSerialBaudrate(port, 115200);

  uint8_t buf[64];
  float enc1;
  float enc2;
  float heading;

  while (true) {
    int n = vexGenericSerialReceive(port, buf, sizeof(buf));

    getSensorReading(buf, n, enc1, enc2, heading);

    wait(10, msec);
  }

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
  //REMOVE "//" BELOW to run Semi-Automatic PID Test
  // semiPIDTest();

  drawSponsors();

  // User control code here, inside the loop
  bool flapState = false;
  int lastSeen = teamColor;

  static vex::thread fireThread = vex::thread(fireClock);

  chassis.brake(coast);
  intake.setStopping(coast);

  intake.setVelocity(100, percent);
  colorSortIntake.setVelocity(100, percent);
  clockRotationSensor.resetPosition();

  //For Skills Auton

  bottomColorSort.setLight(ledState::on);
  bottomColorSort.integrationTime(10);

  //Pressed functions
  Controller1.ButtonL1.pressed(toggleLift);
  // Controller1.ButtonL2.pressed(toggleFrontIntake);
  Controller1.ButtonL2.pressed(toggleWings);
  Controller1.ButtonL2.released(toggleWings);
  Controller1.ButtonLeft.pressed(SPClock);

  int blueMinHue = 200;
  while (1) {

      if(driver)
        chassis.tank();
      else
        chassis.arcade();

      // OLD COLOR SORT
      // if(bottomColorSort.color() == vex::color::red){
      //   lastSeen = 0;
      // }else if(bottomColorSort.color() == vex::color::blue){
      //   lastSeen = 1;
      // }

      //Updated colorsort controls
      int seenHue = bottomColorSort.hue();
      if(seenHue < 20)
        lastSeen = RED;
      if(seenHue > blueMinHue && seenHue < 250)
        lastSeen = BLUE;

      if(Controller1.ButtonR1.pressing()){
        if(isColorSorting)
          topIntake.spin(forward, 50, percent);
        else
          topIntake.spin(forward, 100, percent);

        bottomIntake.spin(forward, 100, percent);
        if(lastSeen == teamColor && isColorSorting){
          colorSortIntake.spin(reverse);
        }else{
          colorSortIntake.spin(forward);
        }
      }else if(Controller1.ButtonR2.pressing()){
        intake.spin(reverse);
        colorSortIntake.spin(forward, 10, percent);
      }else{
        if(isSPRunning){
          bottomIntake.stop();
        }else{
          intake.stop();
          colorSortIntake.stop();
        }
      }
      if(Controller1.ButtonA.pressing()){
        matchLoad.set(true);
        colorSortIntake.spin(forward);
        bottomIntake.spin(reverse);
        if(lastSeen == teamColor && isColorSorting){
          topIntake.spin(forward);
        }else{
          topIntake.spin(reverse);
        }
      }else{
        matchLoad.set(false);
      }

    wait(20, msec);
  }
    
}

void toggleLift(){
  liftState = !liftState;
  intakeLift.set(liftState);
}

void toggleFrontIntake(){
  static bool frontIntakeState = false;
  frontIntakeState = !frontIntakeState;
  frontIntake.set(frontIntakeState);
}

void toggleWings(){
  static bool wingState = false;
  wingState = !wingState;
  wings.set(wingState);
}

int fireClock(){
  while(1){
    if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
      intakeFlap.set(true);
      int timeout = 0;
      int spinSpeed = 100;
      if(liftState){
        //Hood is up
        while(clockRotationSensor.position(degrees) <= 530.0 && timeout <= 750){ //Avg time to complete is ~550ms
          catapult.spin(forward, 100, percent);
          timeout += 10;
          wait(10, msec);
        }
      }else{
        //Hood is down
        while(clockRotationSensor.position(degrees) <= 540.0 && timeout <= 1250){ //Avg time to complete is ~1000ms
          if(clockRotationSensor.position(degrees) >= 250.0){
            //Decrease speed after first stage is complete
            spinSpeed = 50.0;
          } 
          catapult.spin(forward, spinSpeed, percent);
          timeout += 10;
          wait(10, msec);
        }
      }
      intakeFlap.set(false);
      while(clockRotationSensor.position(degrees) >= 45.0 || fabs(catapult.velocity(vex::rpm)) >= 5){
        catapult.spin(reverse, 100, percent);
        timeout += 10;
        wait(10, msec);
      }
      clockRotationSensor.resetPosition();
      catapult.stop();
     
    }
    wait(20, msec);
  }
  return 0;
}

void SPClock(){
  static bool isPrimed = false;
  int timeout = 0;
  if(!isSPRunning){
    isSPRunning = true;
    if(!isPrimed){
      catapult.setStopping(brake);
      topIntake.spin(reverse);
      colorSortIntake.spin(forward, 25, percent);
      wait(800, msec);
      topIntake.stop();
      colorSortIntake.stop();
      while(clockRotationSensor.position(degrees) <= 180.0 && timeout <= 500){
        catapult.spin(forward, 100, percent);
        timeout += 10;
        wait(10, msec);
      }
      catapult.stop();
      catapult.setStopping(coast);
    }else{
      while(clockRotationSensor.position(degrees) >= 45.0 || fabs(catapult.velocity(vex::rpm)) >= 5){
        catapult.spin(reverse, 100, percent);
        timeout += 10;
        wait(10, msec);
      }
      catapult.stop();
      clockRotationSensor.resetPosition();
    }
    isPrimed = !isPrimed;
  }
  isSPRunning = false;
  intake.stop();
  colorSortIntake.stop();
  
}

void toggleIntakeFlap(){
  static bool staticFlap = false;
  staticFlap = !staticFlap;
  intakeFlap.set(staticFlap);
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

/// @brief Auton Slot 4 - Write code for route within this function.
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

/// @brief A thread to get information printed to console while the robot is running (either autonomous routes or drive)
void odomDebugThread() {
  while (odomDebugEnabled) {
    std::cout << "X POS: " << chassis.chassisOdometry.getXPosition()
              << " Y POS: " << chassis.chassisOdometry.getYPosition()
              << " HEADING: " << chassis.chassisOdometry.getHeading()
              << std::endl;

    vex::this_thread::sleep_for(100);
  }
}

/// @brief Runs the semi-automatic PID Test
void semiPIDTest(){
  /*
  --------Buttons--------

  R2 - Drive the Robot (Robot alternates between driving forward and backwards automatically)
  R1/L1 - Swap between drive PID and turn PID
  UP/Down Arrows - Change the drive or turn distance / Adjust the variable values 
  Left/Right Arrows - Change the variable to change (P, I, D, settleError, settleTime, and endTime)
  A - Enter into a variable to be able to change it (Will not be able to use R2 while in this)
  B - Exit and Save a variable (able to use R2 after this)

  --------To Use--------
  Go into userControl and uncomment (Remove //) semiPIDTest();
  Then run the normal user-control and the controller screen will show the test
  */
  PIDTuner tuner(chassis);
  tuner.run();
}