#pragma once

#include <Preferences.h>

#include "domain/IStorageRepository.h"

/**
 * @brief ESP32 NVS (Preferences) implementation of IStorageRepository.
 *
 * Stores SOC state across power cycles in the "battery" namespace.
 * Keys:
 *   "soc"             float   — last saved SOC (0-100 %)
 *   "energyWh"        float   — accumulated energy baseline (Wh)
 *   "socBaseline"     float   — SOC at the time energy baseline was recorded
 *   "valid"           uint8_t — validity flag (1 = valid)
 */
class NvsStorageDriver : public IStorageRepository
{
public:
    NvsStorageDriver();
    ~NvsStorageDriver();

    bool begin() override;

    bool readSoc(float &soc, float &energyBaselineWh, float &socAtBaseline) override;
    bool writeSoc(float soc, float energyBaselineWh, float socAtBaseline) override;

    void invalidate() override;
    bool isValid() const override;

private:
    Preferences prefs_;
    static constexpr const char *NAMESPACE = "battery";
    static constexpr const char *KEY_SOC = "soc";
    static constexpr const char *KEY_ENERGY = "energyWh";
    static constexpr const char *KEY_SOC_BASELINE = "socBl";
    static constexpr const char *KEY_VALID = "valid";
    bool valid_ = false;
};