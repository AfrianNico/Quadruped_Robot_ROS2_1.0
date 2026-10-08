#include "driver/NvsStorageDriver.h"

NvsStorageDriver::NvsStorageDriver()
{
}

NvsStorageDriver::~NvsStorageDriver()
{
    prefs_.end();
}

bool NvsStorageDriver::begin()
{
    prefs_.begin(NAMESPACE, false); // RW mode
    uint8_t v = prefs_.getUChar(KEY_VALID, 0);
    valid_ = (v == 1);
    return true;
}

bool NvsStorageDriver::readSoc(float &soc, float &energyBaselineWh, float &socAtBaseline)
{
    if (!valid_)
    {
        return false;
    }

    soc = prefs_.getFloat(KEY_SOC, 0.0f);
    energyBaselineWh = prefs_.getFloat(KEY_ENERGY, 0.0f);
    socAtBaseline = prefs_.getFloat(KEY_SOC_BASELINE, 0.0f);

    if (soc <= 0.0f || soc > 100.0f)
    {
        invalidate();
        return false;
    }
    return true;
}

bool NvsStorageDriver::writeSoc(float soc, float energyBaselineWh, float socAtBaseline)
{
    prefs_.putFloat(KEY_SOC, soc);
    prefs_.putFloat(KEY_ENERGY, energyBaselineWh);
    prefs_.putFloat(KEY_SOC_BASELINE, socAtBaseline);
    prefs_.putUChar(KEY_VALID, 1);
    valid_ = true;
    return true;
}

void NvsStorageDriver::invalidate()
{
    prefs_.putUChar(KEY_VALID, 0);
    valid_ = false;
}

bool NvsStorageDriver::isValid() const
{
    return valid_;
}