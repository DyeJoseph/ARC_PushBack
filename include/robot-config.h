#pragma once

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

extern brain Brain;
extern controller Controller1;

extern motor LFT;
extern motor LFB;
extern motor LBB;
extern motor LBT;

extern motor RFT;
extern motor RFB;
extern motor RBB;
extern motor RBT;

extern motor_group rightDrive;
extern motor_group leftDrive;

extern rotation rotation1;
extern rotation rotation2;

extern inertial inertial1;

extern motor intakeL;
extern motor intakeR;
extern motor colorSort;
extern motor bottomStage;
extern motor topStage;

extern motor_group mainIntake;


extern digital_out matchLoad;
extern digital_out intakeFlap;
extern digital_out intakeLift;

extern optical bottomColorSort;
