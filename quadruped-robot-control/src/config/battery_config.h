#pragma once

/**
 * @brief Battery pack parameters and SOC thresholds.
 *
 * Designed for a 3S LiPo battery pack:
 *   Nominal: 11.1 V
 *   Full:    12.6 V
 *   Empty:    9.0 V (under load cutoff)
 */

constexpr float BATTERY_FULL_VOLTAGE = 12.6f;   /**< 100 % SOC voltage */
constexpr float BATTERY_EMPTY_VOLTAGE =  9.0f;  /**< 0 % SOC voltage   */
constexpr float BATTERY_CAPACITY_AH = 2.2f;     /**< Pack capacity (Ah)   */
constexpr float SOC_PERSIST_INTERVAL_S = 30.0f; /**< NVS write interval   */

static constexpr float VOLTAGE_SOC_TABLE[][2] = {
    {100.0f, 12.60f},
    {95.0f, 12.45f},
    {90.0f, 12.30f},
    {80.0f, 12.00f},
    {70.0f, 11.70f},
    {60.0f, 11.40f},
    {50.0f, 11.10f},
    {40.0f, 10.80f},
    {30.0f, 10.50f},
    {20.0f, 10.20f},
    {10.0f,  9.90f},
    {5.0f,   9.60f},
    {0.0f,   9.00f},
};

constexpr int VOLTAGE_SOC_TABLE_SIZE =
    sizeof(VOLTAGE_SOC_TABLE) / sizeof(VOLTAGE_SOC_TABLE[0]);

constexpr unsigned long BATTERY_READ_INTERVAL_MS = 500; /**< 2 Hz */