#pragma once

#include <Arduino.h>

/**
 * @brief Operational mode of the robot.
 *
 * - Auto: SPI/vision input drives movement; manual inputs only toggle mode.
 * - Manual: Serial, Bluetooth, and PS3 inputs drive movement.
 */
enum class Mode : uint8_t
{
    AUTO = 0,
    MANUAL = 1,
};