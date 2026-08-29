#pragma once

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

////////////// DO NOT REMOVE //////////////

// Defines the Brain and Controller
extern brain Brain;
extern controller Controller1;

//Color Sort Opticals
extern optical bottomColorSort;

// Drivetrain motors
extern motor LT1;
extern motor LT2;
extern motor LT3;
extern motor LT4;
extern motor LT5;

extern motor RT1;
extern motor RT2;
extern motor RT3;
extern motor RT4;
extern motor RT5;

extern motor_group leftDrive;
extern motor_group rightDrive;


/////////////////////////////////////////
extern motor bottomIntake;
extern motor topIntake;
extern motor_group intake;
extern motor colorSortIntake;

// Rotation Sensors
extern rotation clockRotationSensor;

//Clock motors
extern motor rightCata;
extern motor leftCata;
extern motor_group catapult;

//Pneumatics
extern digital_out intakeFlap;
extern digital_out wings;
extern digital_out intakeLift;
extern digital_out frontIntake;
extern digital_out matchLoad;
extern digital_out midGoalBlocking;
extern digital_out odomRetraction;

extern inertial inertial1;

extern rotation rotation1;
extern rotation rotation2;
extern rotation rotation3;

