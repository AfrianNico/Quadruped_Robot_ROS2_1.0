#include <Arduino.h>
#include "service/BatteryService.h"

BatteryService::BatteryService(IBatterySensor &sensor, IStorageRepository &storage, ILogger &logger)
    : sensor_(sensor), storage_(storage), logger_(logger)
{
}

bool BatteryService::begin()
{
    bool sensorOk = sensor_.begin();
    bool storageOk = storage_.begin();

    if (!sensorOk)
    {
        logger_.error("BatteryService", "Sensor init failed");
    }
    if (!storageOk)
    {
        logger_.error("BatteryService", "Storage init failed");
    }

    float persistedSoc = 0.0f, baselineWh = 0.0f, socBl = 0.0f;
    if (storage_.isValid() && storage_.readSoc(persistedSoc, baselineWh, socBl))
    {
        socPercent_ = persistedSoc;
        energyBaselineWh_ = baselineWh;
        socAtBaseline_ = socBl;
        validEnergy_ = true;
        logger_.info("BatteryService", "Restored SOC: " + String(persistedSoc, 1) + "%");
    }
    else
    {
        logger_.info("BatteryService", "No valid persisted SOC");
        validEnergy_ = false;
    }

    lastPersistMs_ = millis();
    return sensorOk;
}

bool BatteryService::update(BatteryState &state)
{
    float voltage = 0.0f, current = 0.0f, power = 0.0f, energyWh = 0.0f;

    if (!sensor_.read(voltage, current, power, energyWh))
    {
        logger_.warn("BatteryService", "Sensor read failed");
        state.voltage = 0.0f;
        state.current = 0.0f;
        state.power = 0.0f;
        state.socPercent = socPercent_;
        return false;
    }

    unsigned long now = millis();

    if (firstRead_)
    {
        firstRead_ = false;

        if (!validEnergy_)
        {
            socPercent_ = voltageToSoc(voltage);
            logger_.debug("BatteryService", "Cold-start V=" + String(voltage, 2) + "V; SOC=" + String(socPercent_, 1) + "%");
        }

        energyBaselineWh_ = energyWh;
        socAtBaseline_ = socPercent_;
        validEnergy_ = true;
    }
    else
    {
        static unsigned long lastReadMs = now;
        float deltaSec = (now - lastReadMs) / 1000.0f;
        if (deltaSec > 0.0f && deltaSec < 60.0f)
        {
            coulombCount(deltaSec, power);
        }
        lastReadMs = now;
    }

    // logger_.info("TEST", String(voltage, 2) + "," + String(current, 2) + "," + String(power, 2) + "," + String(socPercent_));

    logger_.debug("PZEM", "V=" + String(voltage, 2) + "V; Curr=" + String(current, 2) + "A; Power=" + String(power, 2) + "; Level=" + String(socPercent_) + "%");

    state.voltage = voltage;
    state.current = current;
    state.power = power;
    state.socPercent = socPercent_;

    persistIfNeeded();
    return true;
}

float BatteryService::voltageToSoc(float voltage) const
{
    if (voltage >= BATTERY_FULL_VOLTAGE)
    {
        return 100.0f;
    }
    if (voltage <= BATTERY_EMPTY_VOLTAGE)
    {
        return 0.0f;
    }

    for (int i = 0; i < VOLTAGE_SOC_TABLE_SIZE - 1; i++)
    {
        float vHigh = VOLTAGE_SOC_TABLE[i][1];
        float vLow = VOLTAGE_SOC_TABLE[i + 1][1];

        if (voltage <= vHigh && voltage > vLow)
        {
            float socHigh = VOLTAGE_SOC_TABLE[i][0];
            float socLow = VOLTAGE_SOC_TABLE[i + 1][0];
            float t = (voltage - vLow) / (vHigh - vLow);
            return socLow + t * (socHigh - socLow);
        }
    }

    return 0.0f;
}

void BatteryService::coulombCount(float deltaSeconds, float power)
{
    float deltaAh = (power * deltaSeconds) / 3600.0f;
    float deltaSoc = (deltaAh / BATTERY_CAPACITY_AH) * 100.0f;

    if (power > 0.0f)
    {
        socPercent_ -= deltaSoc;
    }
    else if (power < 0.0f)
    {
        socPercent_ += (-deltaSoc);
    }

    if (socPercent_ > 100.0f)
        socPercent_ = 100.0f;
    if (socPercent_ < 0.0f)
        socPercent_ = 0.0f;
}

void BatteryService::persistIfNeeded()
{
    unsigned long now = millis();
    unsigned long intervalMs = static_cast<unsigned long>(SOC_PERSIST_INTERVAL_S * 1000UL);

    if (now - lastPersistMs_ >= intervalMs)
    {
        storage_.writeSoc(socPercent_, energyBaselineWh_, socAtBaseline_);
        lastPersistMs_ = now;
    }
}