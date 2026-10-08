#pragma once

#include "Command.h"
#include "Mode.h"
#include "MotionVector.h"

/**
 * @brief Interface for all input sources (Serial, Bluetooth, PS3, SPI).
 *
 * Each concrete driver implements begin() and update().
 * update() may modify the command reference based on the input source state.
 */
class ICommandInput
{
public:
    virtual ~ICommandInput() = default;

    /**
     * @brief Initializes the input hardware or protocol handler.
     */
    virtual void begin() = 0;

    /**
     * @brief Polls the input source and updates the active command.
     * @param command Reference to the current command (may be modified).
     * @param mode Reference to the current operational mode (may be modified).
     * @return true if a new valid command or mode change was detected.
     */
    virtual bool update(Command &command, Mode &mode) = 0;

    /**
     * @brief Polls the input source and fills a continuous MotionVector.
     *
     * Optional method — default returns false (not supported).
     * Override in drivers capable of producing continuous 2-axis motion.
     *
     * @param motion Reference to output MotionVector (forward + turn).
     * @return true if a non-zero motion vector is available.
     */
    virtual bool updateMotion(MotionVector &motion)
    {
        (void)motion;
        return false;
    }
};