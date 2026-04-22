#include <stdint.h>
#include "vex.h"
#include "sensorConversion.h"

extern "C" {
  #include "v5_api.h"
}
using namespace vex;

// ── packet layout ──────────────────────────────
// byte  0    : 0xAA  (sync 1)
// byte  1    : 0x55  (sync 2)
// bytes 2–5  : encoder 1  (int32, little-endian)
// bytes 6–9  : encoder 2  (int32, little-endian)
// bytes 10–11: heading    (uint16, little-endian, degrees × 10)
// byte  12   : CRC8       (XOR of bytes 2–11)
// ───────────────────────────────────────────────

/// @brief Gets the degrees of the left rotation sensor
/// @param b bytearray
/// @return degrees
float getLeftRotation(uint8_t* b){
    uint32_t l_rot; //DWORD
    l_rot = (uint32_t)(b[2] | b[3] << 8 | b[4] << 16 | b[5] << 24);
    return ((int)l_rot) / 2048.0 * 360.0;
}

/// @brief Gets the degrees of the right rotation sensor
/// @param b bytearray
/// @return degrees
float getRightRotation(uint8_t* b){
    uint32_t r_rot; //DWORD
    r_rot = (uint32_t)(b[6] | b[7] << 8 | b[8] << 16 | b[9] << 24);
    return ((int)r_rot) / 2048.0 * 360.0;
}

/// @brief Gets the degrees of the IMU 
/// @param b bytearray
/// @return degrees
float getIMUHeading(uint8_t* b){
    uint16_t heading; //WORD
    heading = (uint16_t)(b[10] | b[11] << 8);
    return ((int)heading) / 10.0;
}

/// @brief Gets the reading for rotation sensors and IMU from 3rd party sensors
/// @param buf Input bytearray
/// @param n Number of bytes read
/// @param enc1 Left encoder
/// @param enc2 Right encoder
/// @param heading Heading
void getSensorReading(uint8_t *buf, int n, float &enc1, float &enc2, float &heading){
    for (int i = 0; i < n - 12; i++) {
      // look for sync bytes
      if (buf[i] == 0xAA && buf[i+1] == 0x55) {

        // extract values
        enc1 = getLeftRotation(&buf[i]);
        enc2 = getRightRotation(&buf[i]);
        heading = getIMUHeading(&buf[i]);

        uint8_t crc = buf[i+12];

        // print values
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("enc1: %f      ", enc1);

        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("enc2: %f      ", enc2);

        Brain.Screen.setCursor(4,1);
        Brain.Screen.print("head: %f      ", heading);

        Brain.Screen.setCursor(5,1);
        Brain.Screen.print("crc: %02X      ", crc);
        break;
      }
    }
}

float getMotorEncoderPosition(vex::motor m1, vex::motor m2, vex::motor m3, vex::motor m4, vex::motor m5){
  vex::motor motorArr[5] = {m1, m2, m3, m4, m5};
  std::vector<double> motorList = {m1.position(degrees), m2.position(degrees), m3.position(degrees), m4.position(degrees), m5.position(degrees)};
  float position = 0.0;
  int minIndex = 0, maxIndex = 0;

  for(int i=1;i<motorList.size();i++){
    if(motorList.at(i) > motorList.at(maxIndex))
      maxIndex = i;
    else if(motorList.at(i) < motorList.at(minIndex))
      minIndex = i;
  }
  motorList.erase(motorList.begin() + maxIndex);
  motorList.erase(motorList.begin() + minIndex);

  position += motorList.at(1);
  position += motorList.at(2);
  position += motorList.at(3);
  position /= 3;

  motorArr[maxIndex].setPosition(position, degrees);
  motorArr[minIndex].setPosition(position, degrees);

  return position;
}