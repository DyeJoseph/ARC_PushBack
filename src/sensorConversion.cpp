#include <stdint.h>

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