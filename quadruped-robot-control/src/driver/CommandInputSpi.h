#pragma once

#include <Arduino.h>

#include "config/pin_config.h"
#include "domain/Command.h"
#include "domain/Mode.h"
#include "domain/TelemetryPacket.h"
#include "domain/ICommandInput.h"
#include "domain/ILogger.h"
#include "platform/BatterySharedState.h"
#include "platform/ModeSharedState.h"

/**
 * @brief SPI slave command input from an external processor (e.g. Orange Pi).
 *
 * Bidirectional protocol:
 *   - Master (Orange Pi) initiates a 32-byte transaction every ~50 ms.
 *   - RX: Master sends a command keyword string.
 *   - TX: Slave responds with a TelemetryPacket binary struct (14 bytes)
 *         followed by 18 bytes of zero padding.
 *
 * Keywords (case-insensitive, per ESP32 Update Guide §5):
 *   "forward"   → Command::FORWARD
 *   "backward"  → Command::BACKWARD  (not currently sent by server)
 *   "left"      → Command::TURN_LEFT
 *   "right"     → Command::TURN_RIGHT
 *   "stop"      → Command::NONE
 */
class CommandInputSpi : public ICommandInput
{
public:
    /**
     * @param logger         Logging interface.
     * @param batteryState   Shared battery state for telemetry TX.
     * @param robotBehavior  Current robot behavior int code (set externally).
     */
    CommandInputSpi(ILogger &logger, BatterySharedState &batteryState, ModeSharedState &modeState);

    void begin() override;

    /**
     * @brief Polls the SPI slave for newly received data.
     * @param command Reference to the current command (will be modified).
     * @param mode Reference to the current operational mode (will be modified).
     * @return true if a new valid command was parsed from an SPI packet.
     *
     * @note SPI command data is only accepted when mode == Mode::AUTO.
     */
    bool update(Command &command, Mode &mode) override;

    /**
     * @brief Sets the robot behavior code for the next telemetry packet.
     * @param code  Integer code (e.g. RobotState enum value cast to uint8).
     */
    void setBehaviorCode(uint8_t code) { behaviorCode_ = code; }

#ifdef PIO_UNIT_TESTING
    /**
     * @brief Parses a received null-terminated string into a Command.
     * Exposed as public static for unit testing.
     * @param msg The received string (lowercased in-place).
     * @param command Output command.
     * @return true if a command was recognised.
     */
    static bool parseMessage(const char *msg, Command &command);
#endif

private:
    static constexpr size_t BUF_SIZE = SPI_BUFFER_SIZE;
    ILogger &logger_;
    BatterySharedState &batteryState_;
    ModeSharedState &modeState_;
    uint8_t behaviorCode_ = 0;
    char recvBuf_[BUF_SIZE];
    uint8_t txBuf_[BUF_SIZE];
    bool transactionQueued_ = false;
    Command lastAutoCommand_ = Command::NONE;
    unsigned long lastTransactionMs_ = 0;
    static constexpr unsigned long WATCHDOG_TIMEOUT_MS = 5000;

    /**
     * @brief Queues a new SPI slave receive transaction with telemetry TX.
     */
    void queueTransaction();

    /**
     * @brief Fills txBuf_ with the latest TelemetryPacket + zero padding.
     */
    void fillTelemetryTxBuffer();

#ifndef PIO_UNIT_TESTING
    /**
     * @brief Parses a received null-terminated string into a Command.
     * @param msg The received string (lowercased in-place).
     * @param command Output command.
     * @return true if a command was recognised.
     */
    bool parseMessage(const char *msg, Command &command);
#endif
};