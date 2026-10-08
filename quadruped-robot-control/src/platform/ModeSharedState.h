#pragma once

#include "domain/Mode.h"

class ModeSharedState
{
public:
    ModeSharedState(Mode initialMode);

    void set(Mode mode);
    Mode get() const;

private:
    Mode mode_;
};
