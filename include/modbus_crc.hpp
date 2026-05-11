#pragma once
#include <cstdint>

// Bit-reversal helpers and CRC-16/MODBUS checksum used for motor controller framing.
void invert_uint8(uint8_t* dst, const uint8_t* src);
void invert_uint16(uint16_t* dst, const uint16_t* src);
uint16_t crc16_modbus(const uint8_t* data, uint32_t len);

// Encode a signed velocity (rad/s) into two's-complement high/low bytes
// suitable for the Modbus RTU frame sent to the motor controller.
void velocity_to_bytes(float velocity_rpm, uint8_t& data_low, uint8_t& data_high);
