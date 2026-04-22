#pragma once
#include <stdint.h>
#include <vector>

float getLeftRotation(uint8_t* b);
float getRightRotation(uint8_t* b);
float getIMUHeading(uint8_t* b);
void getSensorReading(uint8_t *buf, int n, float &enc1, float &enc2, float &heading);
float getMotorEncoderPosition(vex::motor m1, vex::motor m2, vex::motor m3, vex::motor m4, vex::motor m5);