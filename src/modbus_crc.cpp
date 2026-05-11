#include "modbus_crc.hpp"
#include <cstdlib>

void invert_uint8(uint8_t* dst, const uint8_t* src)
{
    uint8_t temp = 0;
    for (int i = 0; i < 8; i++) {
        if (src[0] & (1 << i))
            temp |= static_cast<uint8_t>(1 << (7 - i));
    }
    dst[0] = temp;
}

void invert_uint16(uint16_t* dst, const uint16_t* src)
{
    uint16_t temp = 0;
    for (int i = 0; i < 16; i++) {
        if (src[0] & (1 << i))
            temp |= static_cast<uint16_t>(1 << (15 - i));
    }
    dst[0] = temp;
}

uint16_t crc16_modbus(const uint8_t* data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    const uint16_t poly = 0x8005;

    while (len--) {
        uint8_t byte = *data++;
        invert_uint8(&byte, &byte);
        crc ^= static_cast<uint16_t>(byte << 8);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    invert_uint16(&crc, &crc);
    return crc;
}

// Encode a signed RPM value into two's-complement high/low bytes.
// The motor controller expects a 16-bit signed integer split into two bytes.
void velocity_to_bytes(float velocity_rpm, uint8_t& data_low, uint8_t& data_high)
{
    if (velocity_rpm >= 0) {
        uint16_t val = static_cast<uint16_t>(velocity_rpm);
        data_high = static_cast<uint8_t>(val / 256);
        data_low  = static_cast<uint8_t>(val % 256);
    } else {
        uint16_t val = static_cast<uint16_t>(-velocity_rpm);
        uint8_t hi = static_cast<uint8_t>(val / 256);
        uint8_t lo = static_cast<uint8_t>(val % 256);
        if (lo == 0) {
            data_high = 0xFF - hi + 0x01;
            data_low  = 0x00;
        } else {
            data_high = 0xFF - hi;
            data_low  = 0xFF - lo + 0x01;
        }
    }
}
