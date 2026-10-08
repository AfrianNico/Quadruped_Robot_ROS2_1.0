#pragma once

/**
 * @brief Interface for non-volatile storage read/write operations.
 *
 * Used by BatteryService to persist SOC state across power cycles.
 */
class IStorageRepository
{
public:
    virtual ~IStorageRepository() = default;

    /**
     * @brief Opens the storage namespace for read/write.
     * @return true if the storage was opened (or created) successfully.
     */
    virtual bool begin() = 0;

    /**
     * @brief Reads persisted SOC state.
     *
     * @param[out] soc                Last saved SOC (0-100 %).
     * @param[out] energyBaselineWh   Last saved accumulated energy baseline (Wh).
     * @param[out] socAtBaseline      SOC value at the time energy baseline was recorded.
     * @return true if all keys existed and contained valid (in-range) values.
     */
    virtual bool readSoc(float &soc, float &energyBaselineWh, float &socAtBaseline) = 0;

    /**
     * @brief Writes current SOC state to persistence.
     * @param soc               Current SOC (0-100 %).
     * @param energyBaselineWh  Current accumulated energy baseline (Wh).
     * @param socAtBaseline     SOC at the time energy baseline was recorded.
     * @return true if the write succeeded.
     */
    virtual bool writeSoc(float soc, float energyBaselineWh, float socAtBaseline) = 0;

    /**
     * @brief Marks stored state as invalid (e.g. after cold-start fallback).
     */
    virtual void invalidate() = 0;

    /**
     * @brief Indicates whether stored state is considered valid.
     * @return true if valid data was previously persisted.
     */
    virtual bool isValid() const = 0;
};