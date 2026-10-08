#pragma once

#include <cstdint>

/**
 * @brief Raw reading from the PZEM-017 sensor.
 *
 * All values in SI units (V, A, W, Wh).
 */
struct PzemReading
{
    float voltage = 0.0f;  /**< Bus voltage (V) */
    float current = 0.0f;  /**< Bus current (A) */
    float power = 0.0f;    /**< Instantaneous power (W) */
    float energyWh = 0.0f; /**< Cumulative energy (Wh) */

    /** @return true if all values are finite (not NaN, not Inf). */
    bool isValid() const
    {
        return voltage == voltage && current == current &&
               power == power && energyWh == energyWh;
    }
};

/**
 * @brief Battery state-of-charge and diagnostic summary.
 *
 * Produced by BatteryService, consumed by main loop for telemetry.
 */
struct BatteryState
{
    float voltage = 0.0f;    /**< Last measured voltage (V) */
    float current = 0.0f;    /**< Last measured current (A) */
    float power = 0.0f;      /**< Last measured power (W) */
    float socPercent = 0.0f; /**< State of charge 0-100 % */
};