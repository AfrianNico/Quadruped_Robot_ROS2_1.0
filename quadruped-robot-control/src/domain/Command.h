#pragma once

#include <cstdint>

/**
 * @brief High-level robot commands derived from user input (serial, BT, SPI, PS3).
 */
enum class Command : uint8_t
{
    NONE = 0,
    FORWARD = 1,    // Forward / Maju
    BACKWARD = 2,   // Right / Kanan
    TURN_LEFT = 3,  // Left / Kiri
    TURN_RIGHT = 4, // Backward / Mundur
    STAND_UP = 5,   // Stand up to neutral pose
    STAND_DOWN = 6, // Sit down crouch pose
    DUCK = 7,       // Merunduk (duck under obstacle) -- handled separately
};