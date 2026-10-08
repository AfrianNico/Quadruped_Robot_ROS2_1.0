#include "driver/Pzem017Driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Pzem017Driver::Pzem017Driver(HardwareSerial &serial, uint8_t slaveAddr)
    : serial_(serial), slaveAddr_(slaveAddr), node_()
{
}

/** @brief Idle callback: yields to other tasks during Modbus busy-wait. */
static void modbusIdle()
{
    vTaskDelay(0);
}

bool Pzem017Driver::begin()
{
    serial_.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    node_.begin(slaveAddr_, serial_);
    node_.idle(modbusIdle);
    initialized_ = true;
    return true;
}

bool Pzem017Driver::read(float &voltage, float &current, float &power, float &energyWh)
{
    if (!initialized_)
    {
        return false;
    }

    // Registr map: 0x0000=V, 0x0001=C, 0x0002-3=P, 0x0004-5=E
    uint8_t result = node_.readInputRegisters(0x0000, 6);
    if (result != node_.ku8MBSuccess)
    {
        return false;
    }

    // Voltage (register 0x0000, 0.01 V per LSB)
    voltage = static_cast<float>(node_.getResponseBuffer(0)) * 0.01f;

    // Current (register 0x0001, 0.01 A per LSB)
    current = static_cast<float>(node_.getResponseBuffer(1)) * 0.01f;

    // Power (registers 0x0002-0x0003, uint32, 0.1 W per LSB)
    uint32_t pRaw = (static_cast<uint32_t>(node_.getResponseBuffer(3)) << 16) | node_.getResponseBuffer(2);
    power = static_cast<float>(pRaw) * 0.1f;

    // Energy (registers 0x0004-0x0005, uint32, 1 Wh per LSB)
    uint32_t eRaw = (static_cast<uint32_t>(node_.getResponseBuffer(5)) << 16) | node_.getResponseBuffer(4);
    energyWh = static_cast<float>(eRaw) * 1.0f;

    return true;
}

bool Pzem017Driver::readRegister(uint16_t reg, uint16_t &val)
{
    uint8_t result = node_.readInputRegisters(reg, 1);
    if (result == node_.ku8MBSuccess)
    {
        val = node_.getResponseBuffer(0);
        return true;
    }
    return false;
}

bool Pzem017Driver::readRegister32(uint16_t reg, uint32_t &val)
{
    uint8_t result = node_.readInputRegisters(reg, 2);
    if (result == node_.ku8MBSuccess)
    {
        uint16_t hi = node_.getResponseBuffer(1);
        uint16_t lo = node_.getResponseBuffer(0);
        val = (static_cast<uint32_t>(hi) << 16) | lo;
        return true;
    }
    return false;
}