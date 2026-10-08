#pragma once

#include "config/battery_config.h"
#include "domain/BatteryData.h"
#include "domain/IBatterySensor.h"
#include "domain/IStorageRepository.h"
#include "domain/ILogger.h"

/**
 * @brief Battery state-of-charge estimation using a hybrid approach:
 *
 *   1. Voltage-based lookup table (on cold-start, before first sensor read).
 *   2. Coulomb-counting (Ah integration) after the first valid sensor read.
 *   3. Periodic NVS persistence every SOC_PERSIST_INTERVAL_S.
 *
 * SOC is clamped to [0, 100].
 */
class BatteryService
{
public:
    /**
     * @param sensor    PZEM-017 Modbus sensor driver.
     * @param storage   NVS repository for SOC persistence.
     * @param logger    Logging interface.
     */
    BatteryService(IBatterySensor &sensor, IStorageRepository &storage, ILogger &logger);

    /**
     * @brief Initialises sensor and NVS, loads persisted SOC if available.
     * @return true if minimum required dependencies initialised.
     */
    bool begin();

    /**
     * @brief Reads sensor, updates SOC via coulomb-counting.
     *
     * Called periodically from the FreeRTOS battery task (~2 Hz).
     *
     * @param[out] state  Current battery state snapshot.
     * @return true if a valid reading was obtained.
     */
    bool update(BatteryState &state);

    /**
     * @return The most recently computed SOC (0-100 %).
     */
    float getSocPercent() const { return socPercent_; }

private:
    IBatterySensor &sensor_;
    IStorageRepository &storage_;
    ILogger &logger_;

    float socPercent_ = 0.0f;
    float energyBaselineWh_ = 0.0f;
    float socAtBaseline_ = 0.0f;
    bool firstRead_ = true;
    bool validEnergy_ = false;

    unsigned long lastPersistMs_ = 0;

    /**
     * @brief Voltage-to-SOC lookup using the LiFePO4 table.
     * @return SOC in percent (0-100).
     */
    float voltageToSoc(float voltage) const;

    /**
     * @brief Integrates power over time to update SOC.
     * @param deltaSeconds  Time elapsed since last reading.
     * @param power         Instantaneous power (W).
     */
    void coulombCount(float deltaSeconds, float power);

    /**
     * @brief Persists current SOC state to NVS if persist interval has elapsed.
     */
    void persistIfNeeded();
};