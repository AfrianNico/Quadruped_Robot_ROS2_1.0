#pragma once

#include <Arduino.h>
#include <ModbusMaster.h>

#include "domain/IBatterySensor.h"
#include "config/pin_config.h"

/**
 * @brief PZEM-017 DC Energy Meter driver over Modbus RTU.
 *
 * Implements IBatterySensor. Returns values in V, A, W, Wh.
 * The PZEM-017 natively reports in V, A, W, Wh — no conversion needed.
 *
 * Register map (PZEM-017 v1.0, from PZEM017v1 reference library):
 *   0x0000  Voltage      (uint16, 0.01 V resolution)
 *   0x0001  Current      (uint16, 0.01 A resolution)
 *   0x0002  Power        (uint32,  0.1 W resolution, registers 0x0002-0x0003)
 *   0x0004  Energy       (uint32,    1 Wh resolution, registers 0x0004-0x0005)
 *   0x0006  HV Alarms    (uint16)
 *   0x0007  LV Alarms    (uint16)
 *   0x0100  Alarm / Thresholds
 *   0x0103  Address
 *
 * Measurement registers use Input Registers (function code 0x04).
 * Parameter/alarm registers use Holding Registers (function code 0x03).
 */
class Pzem017Driver : public IBatterySensor
{
public:
    /**
     * @param serial      HardwareSerial port (e.g. Serial2).
     * @param slaveAddr   Modbus slave address (default 0x01).
     */
    Pzem017Driver(HardwareSerial &serial, uint8_t slaveAddr = 0x01);

    bool begin() override;

    /**
     * @brief Reads voltage, current, power, and cumulative energy.
     *
     * Blocks for ~50 ms per read (Modbus transaction).
     * Returns raw float values; the PZEM-017 library already returns
     * V, A, W, Wh — NOT mV, mA, mW.
     *
     * @return true if the Modbus transaction succeeded and CRC was valid.
     */
    bool read(float &voltage, float &current, float &power, float &energyWh) override;

private:
    HardwareSerial &serial_;
    uint8_t slaveAddr_;
    ModbusMaster node_;
    bool initialized_ = false;

    /**
     * @brief Reads a single 16-bit register from the PZEM-017.
     * @param reg   Register address.
     * @param val   Output value.
     * @return true on success.
     */
    bool readRegister(uint16_t reg, uint16_t &val);

    /**
     * @brief Reads two consecutive 16-bit registers as a 32-bit value.
     * @param reg      Start register address (high word first).
     * @param val      Output 32-bit value.
     * @return true on success.
     */
    bool readRegister32(uint16_t reg, uint32_t &val);
};