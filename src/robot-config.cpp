#include "robot-config.h"

////////////// DO NOT REMOVE //////////////

// Defines the Brain and Controller
brain Brain;
controller Controller1;

///////////////////////////////////////////

//////////// Drive Train Motors ////////////

///////////////////////////////////////////


//////////// Odometry Sensors ////////////

/////////////////////////////////////////

//////////// Inertial Sensors ////////////

/////////////////////////////////////////
motor intake1 = motor(PORT1,ratio6_1, false);
motor intake2 = motor(PORT2,ratio6_1, true);
motor_group intake = motor_group(intake1, intake2);
motor colorSortIntake = motor(PORT3, ratio6_1, true);

//Pistons

//Color Sort Opticals
optical bottomColorSort = optical(PORT6);



motor LFT = motor(PORT9, ratio6_1, false);
motor LFB = motor(PORT7, ratio6_1, true);
motor LBB = motor(PORT8, ratio6_1, true);
motor LBT = motor(PORT10, ratio6_1, false);

motor RFT = motor(PORT4, ratio6_1, true);
motor RFB = motor(PORT21, ratio6_1, false);
motor RBB = motor(PORT21, ratio6_1, false);
motor RBT = motor(PORT21, ratio6_1, true);
///////////////////////////////////////////

motor_group leftDrive = motor_group(LFT, LFB, LBB, LBT); // Left drive train motors
motor_group rightDrive = motor_group(RFT, RFB, RBB, RBT);

//////////// Odometry Sensors ////////////
rotation rotation1 = rotation(PORT13);
rotation rotation2 = rotation(PORT12);

/////////////////////////////////////////

//////////// Inertial Sensors ////////////

inertial inertial1 = inertial(PORT14);

/////////////////////////////////////////

motor intakeL = motor(PORT5, ratio6_1, true);
motor intakeR = motor(PORT6, ratio6_1, false);
motor colorSort = motor(PORT19, ratio18_1, true);
motor bottomStage = motor(PORT20, ratio6_1, false);
motor topStage = motor(PORT18, ratio6_1, false); //not always spinning

motor_group mainIntake = motor_group(intakeL, intakeR, bottomStage);

//Pistons

/// @brief Pincers
digital_out matchLoad = digital_out(Brain.ThreeWirePort.H);

digital_out intakeFlap = digital_out(Brain.ThreeWirePort.A);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.D);


digital_out dropDown = digital_out(Brain.ThreeWirePort.F);

