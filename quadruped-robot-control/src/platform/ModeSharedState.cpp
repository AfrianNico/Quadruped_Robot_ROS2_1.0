#include "platform/ModeSharedState.h"

ModeSharedState::ModeSharedState(Mode initialMode)
    : mode_(initialMode)
{
}

void ModeSharedState::set(Mode mode)
{
    mode_ = mode;
}

Mode ModeSharedState::get() const
{
    return mode_;
}
