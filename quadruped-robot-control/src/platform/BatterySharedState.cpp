#include "platform/BatterySharedState.h"

BatterySharedState::BatterySharedState()
    : mutex_(nullptr)
{
}

BatterySharedState::~BatterySharedState()
{
    if (mutex_ != nullptr)
    {
        vSemaphoreDelete(mutex_);
    }
}

bool BatterySharedState::begin()
{
    mutex_ = xSemaphoreCreateMutex();
    return (mutex_ != nullptr);
}

void BatterySharedState::set(const BatteryState &state)
{
    if (mutex_ != nullptr)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    state_ = state;
    written_ = true;
    if (mutex_ != nullptr)
    {
        xSemaphoreGive(mutex_);
    }
}

bool BatterySharedState::get(BatteryState &state) const
{
    if (mutex_ != nullptr)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    state = state_;
    bool w = written_;
    if (mutex_ != nullptr)
    {
        xSemaphoreGive(mutex_);
    }
    return w;
}

bool BatterySharedState::getSocPercent(float &soc) const
{
    if (mutex_ != nullptr)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    soc = state_.socPercent;
    bool w = written_;
    if (mutex_ != nullptr)
    {
        xSemaphoreGive(mutex_);
    }
    return w;
}