#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "domain/BatteryData.h"

/**
 * @brief Thread-safe wrapper around BatteryState for inter-task sharing.
 *
 * The battery sensor task writes via set(), the main loop / SPI task reads
 * via get(). Uses a mutex for atomic access so the reader never sees a
 * partially-updated state.
 */
class BatterySharedState
{
public:
    BatterySharedState();
    ~BatterySharedState();

    /** @brief Initialises the internal mutex. */
    bool begin();

    /**
     * @brief Atomically stores a new battery snapshot.
     * Called from the battery sensor task.
     */
    void set(const BatteryState &state);

    /**
     * @brief Atomically reads the latest battery snapshot.
     * @param[out] state  Receives a copy of the current state.
     * @return true if at least one valid write has been performed.
     */
    bool get(BatteryState &state) const;

    /** @brief Quick peek at SOC without a full struct copy. */
    bool getSocPercent(float &soc) const;

private:
    mutable SemaphoreHandle_t mutex_;
    BatteryState state_;
    bool written_ = false;
};