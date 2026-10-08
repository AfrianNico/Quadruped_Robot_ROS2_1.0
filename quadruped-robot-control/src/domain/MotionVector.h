#pragma once

#include "Command.h"

/**
 * @brief Continuous 2-axis motion command for blended gait control.
 *
 * Both axes are active simultaneously — there is no dominant-axis constraint.
 * A non-zero forward with a non-zero turn produces a curved/differential stride.
 *
 * Ranges:
 *   forward: ±MAX_HIP_SWING (typically ±350)   negative = backward, positive = forward
 *   turn:    ±MAX_TURN_OFFSET (typically ±175)  negative = left,    positive = right
 *   buttonCommand: Command::NONE, Command::STAND_UP, or Command::STAND_DOWN
 */
struct MotionVector
{
    int16_t forward;
    int16_t turn;
    Command buttonCommand;

    String toString() const
    {
        return "forward=" + String(forward) + ", turn=" + String(turn) + ", button=" + int(buttonCommand);
    }
};
