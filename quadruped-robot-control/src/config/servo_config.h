#pragma once

#include <Arduino.h>

/**
 * @brief Servo IDs for all 8 joints (4 legs × 2 joints).
 *
 * FR = Front-Right    FL = Front-Left
 * BR = Back-Right     BL = Back-Left
 */
constexpr uint8_t ID_HIP_FR = 1;
constexpr uint8_t ID_KNEE_FR = 2;
constexpr uint8_t ID_HIP_FL = 3;
constexpr uint8_t ID_KNEE_FL = 4;
constexpr uint8_t ID_HIP_BL = 5;
constexpr uint8_t ID_KNEE_BL = 6;
constexpr uint8_t ID_HIP_BR = 7;
constexpr uint8_t ID_KNEE_BR = 8;

/**
 * @brief Knee center offset adjustment per knee servo ID.
 *
 * Dynamic offset applied to knee goal positions so each leg's neutral
 * standing angle is mechanically correct despite assembly variations.
 */
inline int32_t KNEE_CENTER_OFFSET(uint8_t id)
{
    switch (id)
    {
    case ID_KNEE_FR:
        return 2000;
    case ID_KNEE_FL:
        return 2200;
    case ID_KNEE_BL:
        return 2100;
    case ID_KNEE_BR:
        return 2300;
    default:
        return 2048;
    }
}

/**
 * @brief Startup profile acceleration and velocity values.
 */
constexpr uint32_t HIP_PROFILE_ACC = 30;
constexpr uint32_t HIP_PROFILE_VEL = 180;
constexpr uint32_t KNEE_PROFILE_ACC = 40;
constexpr uint32_t KNEE_PROFILE_VEL = 220;

/**
 * @brief Dynamixel control table addresses used by this firmware.
 */
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_PROFILE_ACCELERATION = 108;
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;
constexpr uint16_t ADDR_MOVING = 122;

/**
 * @brief Status packet minimum length: header (4) + reserved (1) + id (1) + length (2) + instr (1) + err (1) + crc (2) = 12.
 * For a 4-byte payload the total is 16.
 */
constexpr uint8_t STATUS_HEADER_LEN = 10;
constexpr uint8_t STATUS_CRC_LEN = 2;

/**
 * @brief Read instruction timeout (ms).
 * Wait for the full response + 1 ms per byte margin.
 */
constexpr uint32_t READ_TIMEOUT_MS = 10;
