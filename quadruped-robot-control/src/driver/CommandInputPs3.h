#pragma once

#include <Arduino.h>

#include "domain/Command.h"
#include "domain/Mode.h"
#include "domain/ICommandInput.h"
#include "domain/ILogger.h"

/**
 * @brief PS3 Bluetooth gamepad input driver.
 *
 * Only compiled when ENABLE_PS3_INPUT is true.
 * Maps left analogue stick to forward/backward (Y-axis) and
 * turn left/right (X-axis) using dominant-axis constraint.
 * Triangle button triggers STAND_UP; Cross button triggers STAND_DOWN.
 */
class CommandInputPs3 : public ICommandInput
{
public:
    explicit CommandInputPs3(ILogger *logger = nullptr);
    void begin() override;
    bool update(Command &command, Mode &mode) override;
    bool updateMotion(MotionVector &motion) override;

};
