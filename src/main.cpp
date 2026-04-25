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
#include "pidTests.h"



using namespace vex;

////////////////////////// GLOBAL VARIABLES //////////////////////////

  // Competition Instance
  competition Competition;

  int odomType = NO_ODOM;

  bool isColorSorting = true;
  bool odomDebugEnabled = true;

  bool isInAuton = false;
  int lastPressed = 0;
  int teamColor = 0; //red = 0, blue = 1
  int driver = 0; //Elliot = 0, Jacob = 1

  bool liftState = 0;
  bool isFiring = 0;
  bool isSPRunning = false;
  bool isPrimed = false;
  bool wingState = false;


  //Auton globals
  bool autonColorSorting = false;
  bool autonLastSeen;
  bool unjamActive = false;
  bool unjamActiveFullIntake = false;


  // Define Values for the Chassis here:
  Drive chassis
  (
    motor_group(LT1, LT2, LT3, LT4, LT5), // Left drive train motors
    motor_group(RT1, RT2, RT3, RT4, RT5), // Right drive train motors
    PORT8,               // Inertial Sensor Port
    2.40,              // The diameter size of the wheel in inches 2.66
    1,                   // 
    12,                   // The maximum amount of the voltage used in the drivebase (1 - 12)
    odomType,
    0.0,                  //Odometry wheel diameter (set to zero if no odom) (1.96 robot behind by .2)
    -1.280,               //Odom pod1 offset -3.867
    -1.280                //Odom pod2 offset -3.867
  );

//////////////////////////////////////////////////////////////////////

///////////////////////// Prototypes /////////////////////////////////

void setDriveTrainConstants();
int prime();
int unprime();
void startAutonToLongGoal();
void autonSetupConstants();
void longGoalWingPush();
void matchLoaderToMiddleLowOuttake();
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
void toggleWingsDown();
int fireClock();
void splitPrimeClock();
void splitReleaseClock();
void autonFireClock(int fireSpeed);

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
  std::string names[8] = {"NONE", "WPWSC", "SPD4", "WPNOSC", 
                          "BALL10", "SPD6", "NONE", "NONE"};
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
  isInAuton = true;
  drawLogo();
  // rotation1.resetPosition();
  // rotation2.resetPosition();
  inertial1.resetHeading();

  wait(100, msec);
  Auton_4();

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
  }
}

/// @brief Runs during the UserControl section of the competition
void usercontrol() 
{
  //REMOVE "//" BELOW to run Semi-Automatic PID Test
  //chassis.setDriveSlew(.5f);
  //semiPIDTest();
  /////////////////////////////////////

  drawLogo();

  // User control code here, inside the loop
  bool flapState = false;
  int lastSeen = teamColor;
  int timeSinceSeenWrong = 0;

  static vex::thread fireThread = vex::thread(fireClock);

  chassis.brake(coast);
  intake.setStopping(coast);

  intake.setVelocity(100, percent);
  colorSortIntake.setVelocity(100, percent);
  clockRotationSensor.resetPosition();
  intakeFlap.set(false);

  //For Skills Auton

  bottomColorSort.setLight(ledState::on);
  bottomColorSort.integrationTime(10);

  //Pressed functions
  Controller1.ButtonL1.pressed(toggleLift);
  Controller1.ButtonL2.pressed(toggleWings);
  Controller1.ButtonL2.released(toggleWingsDown);
  Controller1.ButtonLeft.pressed(splitPrimeClock);
  Controller1.ButtonRight.pressed(splitReleaseClock);

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
      if(seenHue < 20){
        lastSeen = RED;
        if(lastSeen != teamColor)
          timeSinceSeenWrong = 0;
      }else if(seenHue > blueMinHue && seenHue < 250){
        lastSeen = BLUE;
        if(lastSeen != teamColor)
          timeSinceSeenWrong = 0;
      }

      if(Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()){
        if(isColorSorting)
          topIntake.spin(forward, 50, percent);
        else
          topIntake.spin(forward, 100, percent);

        bottomIntake.spin(forward, 100, percent);
        if((lastSeen == teamColor || timeSinceSeenWrong >= 1250) && isColorSorting){
          colorSortIntake.spin(forward);
        }else{

          colorSortIntake.spin(reverse);
        }
      }else if(Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
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
        if((lastSeen == teamColor || timeSinceSeenWrong >= 1250) && isColorSorting){
          topIntake.spin(forward);
        }else{
          topIntake.spin(reverse);
        }
      }else{
        matchLoad.set(false);
      }
      if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
        intakeFlap.set(true);
      }else{
        intakeFlap.set(false);
      }

      if(Controller1.ButtonL2.pressing() && Controller1.ButtonR2.pressing()){
        frontIntake.set(true);
      }else{
        frontIntake.set(false);
      }

    timeSinceSeenWrong += 20;
    wait(20, msec);
  }
}

/// @brief Driver control function to toggle the intake hood
void toggleLift(){
  liftState = !liftState;
  intakeLift.set(liftState);
}

/// @brief Driver control function to toggle the front intake
void toggleFrontIntake(){
  static bool frontIntakeState = false;
  frontIntakeState = !frontIntakeState;
  frontIntake.set(frontIntakeState);
}

/// @brief Driver control function to toggle the descore mechanism
void toggleWings(){
  if(!Controller1.ButtonR2.pressing() || wingState == true){
    wingState = !wingState;
    wings.set(wingState);
  }
}

void toggleWingsDown(){
  wings.set(false);
  liftState = false;
}

/// @brief Driver control function to fire the clock (threaded) 
/// @return Must return an integer for threads
int fireClock(){
  while(1){
    if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
      int timeout = 0;
      int spinSpeed = 100;
      isPrimed = false;
      std::cout << liftState << std::endl;
      if(!liftState){
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
      timeout = 0;
      if(clockRotationSensor.position(degrees) < 100.0)
        clockRotationSensor.setPosition(100.0, degrees);
      while(clockRotationSensor.position(degrees) >= 45.0 || fabs(catapult.velocity(vex::rpm)) >= 5){
        catapult.spin(reverse, 100, percent);
        timeout += 10;
        if(timeout >= 2000)
          break;
        wait(10, msec);
      }
      clockRotationSensor.resetPosition();
      catapult.stop();
     
    }
    wait(20, msec);
  }
  return 0;
}

/// @brief Non-threaded function to fire the clock in autonomous
void autonFireClock(int fireSpeed = 100){
  int timeout = 0;
  int spinSpeed = 100;
  isPrimed = false;
  while(clockRotationSensor.position(degrees) <= 540.0 && timeout <= 750){ //Avg time to complete is ~1000ms
    if(clockRotationSensor.position(degrees) >= 250.0){
      spinSpeed = fireSpeed;
    } 
    catapult.spin(forward, spinSpeed, percent);
    timeout += 10;
    wait(10, msec);
  }
  timeout = 0;
  while(clockRotationSensor.position(degrees) >= 45.0 || fabs(catapult.velocity(vex::rpm)) >= 5){
    catapult.spin(reverse, 100, percent);
    timeout += 10;
    if(timeout >= 2000)
      break;
    wait(10, msec);
  }
  clockRotationSensor.resetPosition();
  catapult.stop();
}

/// @brief Non-threaded function to fire the clock in autonomous
void autonFireClockNoUnprime(int fireSpeed = 100){
  int timeout = 0;
  int spinSpeed = 100;
  isPrimed = false;
  while(clockRotationSensor.position(degrees) <= 540.0 && timeout <= 1500){ //Avg time to complete is ~1000ms
    if(clockRotationSensor.position(degrees) >= 250.0){
      spinSpeed = fireSpeed;
    } 
    catapult.spin(forward, spinSpeed, percent);
    timeout += 10;
    wait(10, msec);
  }
  catapult.stop();
}

/// @brief Splits the loaded balls in half and primes the catapult
void splitPrimeClock(){
  int timeout = 0;
  if(!isSPRunning && !isPrimed){
    //Set running variable to true and create guard (in case of early exit)
    isSPRunning = true;
    SPGuard guard(isSPRunning);

    //Reverse intake slightly
    catapult.setStopping(brake);
    topIntake.spin(reverse);
    colorSortIntake.spin(forward, 25, percent);
    wait(800, msec);
    topIntake.stop();
    colorSortIntake.stop();

    //Prime catapult
    while(clockRotationSensor.position(degrees) <= 180.0 && timeout <= 500){
      catapult.spin(forward, 100, percent);
      timeout += 10;
      wait(10, msec);
    }

    //Stop motor function
    catapult.stop();
    catapult.setStopping(coast);
    intake.stop();
    colorSortIntake.stop();

    //Set variables
    isPrimed = true;
    isSPRunning = false;
  }
}

/// @brief If primed, releases the prime
void splitReleaseClock(){
  if(!isSPRunning && isPrimed){
    //Set running variable to true and create guard (in case of early exit)
    isSPRunning = true;
    SPGuard guard(isSPRunning);
    int timeout = 0;
    //Reset catapult
    while(clockRotationSensor.position(degrees) >= 45.0 || fabs(catapult.velocity(vex::rpm)) >= 5){
      catapult.spin(reverse, 100, percent);
      timeout += 10;
      if(timeout >= 2000)
        break;
      wait(10, msec);
    }

    //Stop motor function
    catapult.stop();
    clockRotationSensor.resetPosition();

    //Set variables
    isPrimed = false;
    isSPRunning = false;
  }
}

// void toggleIntakeFlap(){
//   static bool staticFlap = false;
//   staticFlap = !staticFlap;
//   intakeFlap.set(staticFlap);
// }

/// @brief Function for toggling whether color sort is active
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
    chassis.setDriveConstants(
        1.0,  // Kp
        0.0,  // Ki
        10.0,  // Kd
        0.5,  // Settle Error
        200,  // Time to Settle
        5000  // End Time
    );

    chassis.setTurnConstants(
        // 0.4,  // Kp
        // 0.0,   // Ki
        // 1.5,   // Kd
        // 0.75,  // Settle Error
        // 200,   // Time to Settle
        // 2500   // End Time

        1.0,    // Kp - Proportion Constant
        0.0,      // Ki - Integral Constant
        3.5,      // Kd - Derivative Constant 
        .75, //1.25    // Settle Error
        200, 
        1500 
    );
}

//Easy use functions for auton
//void unloadMatchLoader
int prime(){
  int timeout = 0;
  clockRotationSensor.setPosition(0, degrees);
  while(clockRotationSensor.position(degrees) <= 180.0 && timeout <= 500){
      catapult.spin(forward, 100, percent);
      timeout += 10;
      wait(10, msec);
    }
    catapult.stop();
    catapult.setStopping(coast);
    return 0;
}

int unprime(){
  int timeout = 0;
    //Reset catapult
    if(clockRotationSensor.position(degrees) < 100){
      clockRotationSensor.setPosition(100, degrees);
    }
    while(clockRotationSensor.position(degrees) >= 45.0 || fabs(catapult.velocity(vex::rpm)) >= 5){
      catapult.spin(reverse, 100, percent);
      timeout += 10;
      if(timeout >= 2000)
        break;
      wait(10, msec);
    }

    //Stop motor function
    catapult.stop();
    clockRotationSensor.resetPosition();
    return 0;
}

void longGoalWingPush(){
  chassis.setTurnConstants(0.95, 0.0, 5.0, 1.0, 200, 1500);
  chassis.turnToAngle(315, 12);
  chassis.driveDistance(-4.5);
  chassis.turnToAngle(275, 12);
  chassis.driveDistance(25);
  setDriveTrainConstants();

}

int autonColorSort(){
  int blueMinHue = 200;
  while(1){
    if(autonColorSorting){
      int seenHue = bottomColorSort.hue();
      if(seenHue < 20){
        autonLastSeen = RED;
      }else if(seenHue > blueMinHue && seenHue < 250){
        autonLastSeen = BLUE;
      }
    }
  }
  return 0;
}

int colorUnjam(){
  while(1){
    if(fabs(colorSortIntake.velocity(rpm)) <= 5 && unjamActive){
      colorSortIntake.spin(reverse, 100, percent);
      std::cout << "UNJAMMED SO HARD" << std::endl;
      wait(250, msec);
      colorSortIntake.spin(forward, 100, percent);
    } else {
      wait(20, msec);
    }
  }
  return 0;
}

int fullIntakeUnjam(){
  while(1){
    if(fabs(bottomIntake.velocity(rpm)) <= 5 && unjamActiveFullIntake || fabs(topIntake.velocity(rpm) <= 5 && unjamActiveFullIntake)){
      intake.spin(reverse, 100, percent);
      std::cout << "UNJAMMED INTAKE INTAKE INTAKEEEEEE SO HARD" << std::endl;
      wait(250, msec);
      intake.spin(forward, 100, percent);
    } else {
      wait(20, msec);
    }
  }
  return 0;
}

void autonSetupConstants(){
  Brain.resetTimer();
  
  static vex::thread autonColor = vex::thread(autonColorSort);
  static vex::thread colorIsSpinningUnjam = vex::thread(colorUnjam);
  static vex::thread intakeIsSpinningUnjam = vex::thread(fullIntakeUnjam);
  autonColorSorting = true;
  chassis.setDriveMaxVoltage(12);
  chassis.setTurnMaxVoltage(12);

  //chassis.setSCurveConstants(60.0f, 120.0f, 400.0f);
  chassis.setSCurveConstants(120.0f, 240.0f, 1200.0f);
  chassis.setDriveKff(12.0f / 78.9891f *.2f);
  chassis.setDriveKs(1.0f);
  chassis.setStallDetection(0.05f, 200.0f);
  chassis.setPosition(0,0,0);

  static bool frontIntakeState = false;
}

void startAutonToLongGoal(){
  chassis.driveDistance(41); //41
  chassis.setTurnConstants(0.95, 0.0, 5.0, 1.0, 200, 1500);
  chassis.turnToAngle(273);
  setDriveTrainConstants();
  matchLoad.set(true);
  intake.spin(forward, 100, pct);
  colorSortIntake.spin(forward, 100, percent); 
  unjamActive = true;
  chassis.driveDistance(-11.5);
}

void matchLoaderToMiddleLowOuttake(){
    matchLoad.set(false);
    chassis.driveDistance(12);
    chassis.turnToAngle(225);
    colorSortIntake.spin(reverse, 50, pct);
    chassis.driveDistance(49);
    toggleFrontIntake();
    unjamActiveFullIntake = true;
    vex::thread unprimeThread(unprime);
    wait(100, msec);
    topIntake.spin(reverse, 100, percent);
    bottomIntake.spin(reverse, 25, percent);
    colorSortIntake.spin(reverse, 10, percent);
    // wait(1000, msec);
    // topIntake.spin(forward, 100, percent);
    // bottomIntake.spin(forward, 100, percent);
    // wait(150, msec);
    // topIntake.spin(reverse, 100, percent);
    // bottomIntake.spin(reverse, 25, percent);
    // colorSortIntake.spin(reverse, 10, percent);
    wait(2500, msec);
    unjamActiveFullIntake = false;
    toggleFrontIntake();
}
//Auton Route Functions
/// @brief Auton Slot 1 - Write code for route within this function.
void Auton_1() //TESTING RN DO NOT USE
{   
  std::cout << "AUTON 1" << std::endl;
  // chassis.setSCurveConstants(60.0f, 120.0f, 600.0f);
  // chassis.setDriveKff(12.0f / 78.9891f *.2f);
  // chassis.setDriveKs(1.0f);
  // chassis.setStallDetection(0.05f, 500.0f);
  // chassis.setPosition(0,0,0);

  // //BELOW 25 DEG
  // chassis.setTurnConstants(1.6, 0.0, 4.5, .75, 200, 1500);

  // //ABOVE 25 DEG
  // chassis.setTurnConstants(0.95, 0.0, 5.0, 1.0, 200, 1500);

  // toggleFrontIntake();
  // // chassis.driveDistance(24);
  // //longGoalWingPush();
  

  std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() << std::endl;
    

}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2() // Win Point with Scrape
{   
  std::cout << "AUTON 2" << std::endl;
  autonSetupConstants();
  int loopTime = 0;

  startAutonToLongGoal();
  while(loopTime <= 1000){
    if(autonLastSeen == !teamColor){
      vex::thread primeThread(prime);
      //bottomIntake.spin(reverse, 15, percent); 

      break;
    }
    loopTime += 5;
    wait(5, msec);
  }

  matchLoaderToMiddleLowOuttake();



  intake.stop();
  chassis.driveDistance(-49);
  chassis.setTurnConstants(0.95, 0.0, 5.0, 1.0, 200, 1500);
  chassis.turnToAngle(273);
  setDriveTrainConstants();
  matchLoad.set(true);
  intake.spin(forward, 100, pct);
  colorSortIntake.spin(forward, 100, percent); 
  unjamActive = true;
  chassis.driveDistance(-11.5);

  toggleWings(); // up
  chassis.turnToAngle(255); //283
  chassis.driveDistance(29.5); // 29
  chassis.turnToAngle(290);
  chassis.driveDistance(14); // 19

  toggleWings(); // down
  chassis.turnToAngle(270);
  chassis.driveDistance(-33);
  chassis.turnToAngle(330); // 225
  chassis.driveDistance(12);
  chassis.turnToAngle(270);
  chassis.driveDistance(10);

  intakeFlap.set(true);
  autonFireClock(30);
  intake.spin(forward, 100, percent);
  colorSortIntake.spin(forward, 100, percent);
  wait(1000, msec);
  autonFireClockNoUnprime(30);
  vex::thread unprimeThread(unprime);

  intake.stop();
  colorSortIntake.stop();
  unjamActive = false;
  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3() //FAST 4
{   
  std::cout << "AUTON 3" << std::endl;
  autonSetupConstants();
  int loopTime = 0;

  startAutonToLongGoal();

  while(loopTime <= 1000){
    if(autonLastSeen == !teamColor){
      vex::thread primeThread(prime);
      break;
    }

    loopTime += 5;
    wait(5, msec);
  }
  loopTime = 0;
  matchLoad.set(false);

  intake.stop();
  colorSortIntake.stop();
  
  chassis.driveDistance(28);
  intakeFlap.set(true);
  autonFireClockNoUnprime(30);
  vex::thread unprimeThread(unprime);
  
  unjamActive = false;
  colorSortIntake.stop();

  longGoalWingPush();
  intakeFlap.set(false);

  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4() // Win Point No Scrape
{  
  std::cout << "AUTON 4" << std::endl;
  autonSetupConstants();
  int loopTime = 0;

  startAutonToLongGoal();
  while(loopTime <= 1000){
    if(autonLastSeen == !teamColor){
      vex::thread primeThread(prime);
      //bottomIntake.spin(reverse, 15, percent); 

      break;
    }
    loopTime += 5;
    wait(5, msec);
  }

  matchLoaderToMiddleLowOuttake();

  intake.stop();
  chassis.driveDistance(-49);
  chassis.setTurnConstants(0.95, 0.0, 5.0, 1.0, 200, 1500);
  chassis.turnToAngle(273);
  setDriveTrainConstants();
  matchLoad.set(true);
  intake.spin(forward, 100, pct);
  colorSortIntake.spin(forward, 100, percent); 
  unjamActive = true;
  chassis.driveDistance(-11.5);

  wait(1000, msec);
  chassis.driveDistance(28);  
  intakeFlap.set(true);
  bottomIntake.stop();
  autonFireClock(30);
  //chassis.driveDistance(-10);
  //toggleFrontIntake();
  //intake.stop();
  

  intake.stop();
  colorSortIntake.stop();
  unjamActive = false;
  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5() //10 BALL
{
  std::cout << "AUTON 5" << std::endl;
  autonSetupConstants();
  int loopTime = 0;

  startAutonToLongGoal();

  while(loopTime <= 1000){
    if(autonLastSeen == !teamColor){
      vex::thread primeThread(prime);
      bottomIntake.spin(reverse, 15, percent); 
      wait(150, msec);

      break;
    }
    loopTime += 5;
    wait(5, msec);
  }

  matchLoad.set(false);
  topIntake.spin(forward, 25, percent);

  intake.spin(forward, 100, pct);

  chassis.driveDistance(28);
  intakeFlap.set(true);
  autonFireClockNoUnprime(30);
  vex::thread unprimeThread1(unprime);
  intakeFlap.set(false);

  matchLoad.set(true);
  topIntake.spin(forward, 100, percent);
  bottomIntake.spin(forward, 100, percent);
  chassis.driveDistance(-20);

  intakeFlap.set(true);
  autonFireClockNoUnprime(100);
  vex::thread unprimeThread2(unprime);

  chassis.driveDistance(-8);
  intakeFlap.set(false);

  wait(500, msec);
  bottomIntake.spin(reverse, 15, percent);
  wait(1000, msec);
  bottomIntake.spin(forward, 100, percent);

  intake.spin(forward, 100, pct);

  chassis.driveDistance(28);  
  intakeFlap.set(true);
  autonFireClock(30);
  autonFireClockNoUnprime(20);
  vex::thread unprimeThread3(unprime);

  chassis.driveDistance(-5);
  intakeFlap.set(false);
  chassis.driveDistance(5);
  intake.stop();
  colorSortIntake.stop();
  unjamActive = false;

  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
}

/// @brief Auton Slot 6 - Write code for route within this function.
void Auton_6() //speed 6
{
  std::cout << "AUTON 6" << std::endl;
  autonSetupConstants();
  int loopTime = 0;

  //Go to match loader
  startAutonToLongGoal();

  //Fire twice (clear matchloader)
  while(loopTime <= 1000){
    if(autonLastSeen == !teamColor){
      vex::thread primeThread(prime);
      bottomIntake.spin(reverse, 15, percent); 
      wait(300, msec);

      break;
    }
    loopTime += 5;
    wait(5, msec);
  }
  intakeFlap.set(true);
  autonFireClockNoUnprime();
  vex::thread unprimeThread1(unprime);
  intake.spin(forward, 100, percent);
  wait(800, msec);
  topIntake.stop();
  autonFireClockNoUnprime(100);
  intakeFlap.set(false);
  vex::thread unprimeThread2(unprime);
  autonLastSeen = !teamColor;
  loopTime = 0;
  while(loopTime <= 10000){
    if(autonLastSeen == teamColor){
      intake.spin(forward, 100, percent);
      wait(1000, msec);
      break;
    }
    loopTime += 5;
    wait(5, msec);
  }


  //Score
  chassis.driveDistance(28);  
  intakeFlap.set(true);
  bottomIntake.stop();
  autonFireClock(30);
  wait(100, msec);

  autonFireClockNoUnprime(30);
  vex::thread unprimeThread4(unprime);

  //Wing scrape
  longGoalWingPush();
  intakeFlap.set(false);

  intake.stop();
  colorSortIntake.stop();
  unjamActive = false;


  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
}

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7() //SCORES TOP MIDDLE REAL
{
  std::cout << "AUTON 7" << std::endl;
  /*
  autonSetupConstants();
  int loopTime = 0;

  startAutonToLongGoal();
  wait(1000, msec);

  chassis.driveDistance(28);
  intakeFlap.set(true);
  autonFireClock(30);
  autonFireClockNoUnprime(20);
  vex::thread unprimeThread3(unprime);
  chassis.setTurnConstants(0.95, 0.0, 5.0, 1.0, 200, 1500);
  chassis.turnToAngle(225, 12);
  chassis.driveDistance(5);
  intakeFlap.set(false);
  chassis.turnToAngle(90);
  chassis.driveDistance(28);
  chassis.turnToAngle(245);
  chassis.driveDistance(20.5);
  chassis.turnToAngle(180);
  chassis.driveDistance(19);
  chassis.turnToAngle(270);
  chassis.driveDistance(30);
  chassis.turnToAngle(0);
  chassis.driveDistance(17);
  chassis.turnToAngle(90);
  chassis.driveDistance(12);
  intakeFlap.set(true);
  autonFireClock(30);
  autonFireClockNoUnprime(20);
  vex::thread unprimeThread1(unprime);
  chassis.driveDistance(-28);
  wait(1000, msec);
  setDriveTrainConstants();
  toggleLift(); //DOWN
  chassis.driveDistance(10);
  intake.spin(forward, 100, percent);
  colorSortIntake.spin(forward, 100, percent)*/




  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8() //SCORES LOW MIDDLE REAL
{
  autonSetupConstants();
  int loopTime = 0;


  double time = Brain.timer(seconds);
  std::cout << "TIME: " << time << " seconds\n";
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