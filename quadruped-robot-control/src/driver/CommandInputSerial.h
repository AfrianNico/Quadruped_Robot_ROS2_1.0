#pragma once

#include "domain/ICommandInput.h"

/**
 * @brief Serial-based command input driver.
 *
 * Template parameter SerialT must provide:
 *   - int available()
 *   - int read()
 *   - void print(const char *)
 *   - void println(const char *)
 *
 * Production instantiation: CommandInputSerial<HardwareSerial>
 * Test instantiation:      CommandInputSerial<MockSerial>
 */
template <typename SerialT>
class CommandInputSerial : public ICommandInput
{
public:
    explicit CommandInputSerial(SerialT &serial)
        : serial_(serial), rxBuffer_()
    {
    }

    void begin() override {}

    bool update(Command &command, Mode &mode) override
    {
        bool changed = false;

        while (serial_.available() > 0)
        {
            char ch = static_cast<char>(serial_.read());

            if (ch == '\n' || ch == '\r')
            {
                rxBuffer_.trim();
                if (rxBuffer_.equalsIgnoreCase("AUTO"))
                {
                    mode = Mode::AUTO;
                    serial_.println("Switch mode to: AUTO");
                    changed = true;
                }
                else if (rxBuffer_.equalsIgnoreCase("MANUAL"))
                {
                    mode = Mode::MANUAL;
                    serial_.println("Switch mode to: MANUAL");
                    changed = true;
                }
                rxBuffer_ = "";
            }
            else if (ch == ' ' || ch == '\t')
            {
                continue;
            }
            else
            {
                rxBuffer_ += ch;
            }

            if (rxBuffer_.length() == 1)
            {
                char cmd = rxBuffer_[0];
                if (cmd >= 'a' && cmd <= 'z')
                {
                    cmd = static_cast<char>(cmd - ('a' - 'A'));
                }

                if (cmd >= 'A' && cmd <= 'D')
                {
                    if (mode == Mode::MANUAL)
                    {
                        command = static_cast<Command>(static_cast<uint8_t>(cmd - 'A') + 1);
                        serial_.print("Execute command: ");
                        serial_.println(cmd);
                        changed = true;
                    }
                    rxBuffer_ = "";
                }
            }
        }

        return changed;
    }

    bool updateMotion(MotionVector &motion) override
    {
        (void)motion;
        return false;
    }

private:
    SerialT &serial_;
    String rxBuffer_;
};