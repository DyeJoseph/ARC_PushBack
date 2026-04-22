#include "robot-config.h"

////////////// DO NOT REMOVE //////////////

// Defines the Brain and Controller
brain Brain;
controller Controller1;

//Color Sort Opticals
optical bottomColorSort = optical(PORT6);

// Drivetrain motors
motor LT1 = motor(PORT16, ratio6_1, false);
motor LT2 = motor(PORT17, ratio6_1, true);
motor LT3 = motor(PORT18, ratio6_1, false);
motor LT4 = motor(PORT19, ratio6_1, true);
motor LT5 = motor(PORT20, ratio6_1, false);

motor RT1 = motor(PORT11, ratio6_1, true);
motor RT2 = motor(PORT12, ratio6_1, false);
motor RT3 = motor(PORT13, ratio6_1, true);
motor RT4 = motor(PORT14, ratio6_1, false);
motor RT5 = motor(PORT15, ratio6_1, true);

motor_group leftDrive = motor_group(LT1, LT2, LT3, LT4, LT5);
motor_group rightDrive = motor_group(RT1, RT2, RT3, RT4, RT5);

/////////////////////////////////////////

/////////////////////////////////////////
motor bottomIntake = motor(PORT1,ratio6_1, false);
motor topIntake = motor(PORT2,ratio6_1, true);
motor_group intake = motor_group(bottomIntake, topIntake);
motor colorSortIntake = motor(PORT3, ratio6_1, true);

// Rotation Sensors
rotation clockRotationSensor = rotation(PORT7);

// Clock motors
motor rightCata = motor(PORT4, ratio18_1, false);
motor leftCata = motor(PORT5, ratio18_1, true);
motor_group catapult = motor_group(leftCata, rightCata);

//Pneumatics
digital_out intakeFlap = digital_out(Brain.ThreeWirePort.G);
digital_out wings = digital_out(Brain.ThreeWirePort.F);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.C);
digital_out frontIntake = digital_out(Brain.ThreeWirePort.E);
digital_out matchLoad = digital_out(Brain.ThreeWirePort.B);
digital_out midGoalBlocking = digital_out(Brain.ThreeWirePort.H);
digital_out odomRetraction = digital_out(Brain.ThreeWirePort.D);

//Inertial
inertial inertial1 = inertial(PORT8);



