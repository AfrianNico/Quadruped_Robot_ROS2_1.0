#pragma once

#include <stddef.h>
#include <cstdint>

/**
 * @brief Fixed-size binary packet sent from ESP32 to Orange Pi via SPI.
 *
 * Layout (14 bytes total):
 *   Offset  Size  Field
 *   0       2     header         0xAA 0x55
 *   2       4     voltage        float32, V
 *   6       4     current        float32, A
 *   10      1     socPercent     uint8, 0-100
 *   11      1     robotBehavior  uint8 (Command enum value)
 *   12      2     crc16          uint16, raw CRC register value (LE: low byte first = MODBUS wire order)
 *
 * @note Packed to prevent compiler padding between fields.
 *       Without this, float alignment inserts 2 pad bytes after header.
 */
#pragma pack(push, 1)
struct TelemetryPacket
{
    uint8_t header[2];     /**< Start-of-frame marker: {0xAA, 0x55} */
    float voltage;         /**< Battery voltage in V */
    float current;         /**< Battery current in A */
    uint8_t socPercent;    /**< State of charge 0-100 % */
    uint8_t robotBehavior; /**< Cast of Command or RobotState enum */
    uint16_t crc;          /**< CRC-16/MODBUS over bytes[0..11], raw register value (LE storage = MODBUS wire order) */
};
#pragma pack(pop)

constexpr size_t TELEMETRY_PACKET_SIZE = 14;

static_assert(sizeof(TelemetryPacket) == TELEMETRY_PACKET_SIZE,
              "TelemetryPacket must be exactly 14 bytes (packed)");