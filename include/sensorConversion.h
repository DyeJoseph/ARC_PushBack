#pragma once
#include <stdint.h>

float getLeftRotation(uint8_t* b);
float getRightRotation(uint8_t* b);
float getIMUHeading(uint8_t* b);
void getSensorReading(uint8_t *buf, int n, float &enc1, float &enc2, float &heading);