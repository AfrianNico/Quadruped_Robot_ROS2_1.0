#pragma once

/**
 * @brief Interface for battery voltage/current/power sensor (PZEM-017).
 */
class IBatterySensor
{
public:
    virtual ~IBatterySensor() = default;

    /**
     * @brief Initializes the sensor hardware (UART, Modbus).
     * @return true if initialization succeeded.
     */
    virtual bool begin() = 0;

    /**
     * @brief Reads voltage, current, power, and cumulative energy.
     *
     * All values are in SI units: V, A, W, Wh.
     *
     * @param[out] voltage   Bus voltage in Volts.
     * @param[out] current   Bus current in Amperes.
     * @param[out] power     Instantaneous power in Watts.
     * @param[out] energyWh  Cumulative energy consumption in Watt-hours.
     * @return true if the read was successful, false otherwise (timeout, CRC error).
     */
    virtual bool read(float &voltage, float &current, float &power, float &energyWh) = 0;
};