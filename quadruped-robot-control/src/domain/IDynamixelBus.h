#pragma once

#include <cstdint>

/**
 * @brief Interface for the Dynamixel servo bus communication layer.
 *
 * All concrete Dynamixel driver implementations (e.g. DynamixelBusDriver on Serial1)
 * must implement this interface to decouple service logic from hardware.
 */
class IDynamixelBus
{
public:
    virtual ~IDynamixelBus() = default;

    /**
     * @brief Initialises the UART port and direction pin.
     * @param txPin UART TX pin number.
     * @param rxPin UART RX pin number.
     */
    virtual void begin(int txPin, int rxPin) = 0;

    /**
     * @brief Writes one byte to a control table address.
     * @param id Servo ID.
     * @param addr Control table address.
     * @param value Byte value to write.
     */
    virtual void write1Byte(uint8_t id, uint16_t addr, uint8_t value) = 0;

    /**
     * @brief Writes a 4-byte (int32) value to a control table address.
     * @param id Servo ID.
     * @param addr Control table address.
     * @param value 32-bit value to write.
     */
    virtual void write4Byte(uint8_t id, uint16_t addr, uint32_t value) = 0;

    /**
     * @brief Enables or disables torque on a servo.
     * @param id Servo ID.
     * @param enable true to enable torque.
     */
    virtual void torqueEnable(uint8_t id, bool enable) = 0;

    /**
     * @brief Reads the present position (4-byte) from a single servo.
     * @param id Servo ID.
     * @return The present position value, or INT32_MIN on failure/timeout.
     */
    virtual int32_t readPresentPosition(uint8_t id) = 0;

    /**
     * @brief Reads the "Moving" status bit (addr 122, 1 byte).
     * @param id Servo ID.
     * @return true if the servo reports it is still moving.
     */
    virtual bool isMoving(uint8_t id) = 0;

    /**
     * @brief Writes goal position to one leg (hip + knee servo) using a sync-write packet.
     * @param posHip Raw position offset for the hip servo.
     * @param posKnee Raw position offset for the knee servo.
     * @param idHip Servo ID for the hip.
     * @param idKnee Servo ID for the knee.
     */
    virtual void writeLegGoal(int32_t posHip, int32_t posKnee, uint8_t idHip, uint8_t idKnee) = 0;

    /**
     * @brief Writes profile velocity to one leg (hip + knee) using a sync-write packet.
     * @param speedHip Profile velocity for the hip.
     * @param speedKnee Profile velocity for the knee.
     * @param idHip Servo ID for the hip.
     * @param idKnee Servo ID for the knee.
     */
    virtual void writeLegSpeedRaw(int32_t speedHip, int32_t speedKnee, uint8_t idHip, uint8_t idKnee) = 0;

    /**
     * @brief Sets profile acceleration for a servo.
     * @param id Servo ID.
     * @param accel Acceleration value.
     */
    virtual void setProfileAccel(uint8_t id, uint32_t accel) = 0;

    /**
     * @brief Sets profile velocity for a servo.
     * @param id Servo ID.
     * @param vel Velocity value.
     */
    virtual void setProfileVelocity(uint8_t id, uint32_t vel) = 0;
};