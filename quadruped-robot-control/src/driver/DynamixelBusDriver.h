#pragma once

#include <Arduino.h>

#include "domain/IDynamixelBus.h"

/**
 * @brief Concrete Dynamixel Protocol 2.0 bus driver.
 *
 * Implements IDynamixelBus over a HardwareSerial UART with an RS485
 * half-duplex direction-control pin (DXL_DIR_PIN).
 */
class DynamixelBusDriver : public IDynamixelBus
{
public:
    /**
     * @param port Reference to the HardwareSerial UART (e.g. Serial1, Serial2).
     * @param dirPin GPIO pin used for RS485 direction control.
     */
    DynamixelBusDriver(HardwareSerial &port, uint8_t dirPin);

    void begin(int txPin, int rxPin) override;

    void write1Byte(uint8_t id, uint16_t addr, uint8_t value) override;
    void write4Byte(uint8_t id, uint16_t addr, uint32_t value) override;
    void torqueEnable(uint8_t id, bool enable) override;
    void writeLegGoal(int32_t posHip, int32_t posKnee, uint8_t idHip, uint8_t idKnee) override;
    void writeLegSpeedRaw(int32_t speedHip, int32_t speedKnee, uint8_t idHip, uint8_t idKnee) override;
    int32_t readPresentPosition(uint8_t id) override;
    bool isMoving(uint8_t id) override;
    void setProfileAccel(uint8_t id, uint32_t accel) override;
    void setProfileVelocity(uint8_t id, uint32_t vel) override;

    /**
     * @brief Computes a Dynamixel Protocol 2.0 CRC-16.
     * Exposed publicly so shared helpers (e.g. anonymous-namespace readResponse)
     * can compute CRC without duplication.
     */
    static uint16_t updateCRC(uint16_t crcAccum, const uint8_t *data, uint16_t size);

private:
    HardwareSerial &port_;
    uint8_t dirPin_;

    void sendPacket(uint8_t *packet, uint8_t len);
    int readStatusPacket(uint8_t id, uint8_t *buf, uint8_t expectedLen);
};