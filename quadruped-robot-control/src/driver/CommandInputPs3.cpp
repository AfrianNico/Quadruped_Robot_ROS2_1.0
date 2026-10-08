#include "CommandInputPs3.h"
#include "config/system_config.h"
#include "domain/MotionVector.h"

#if ENABLE_PS3_INPUT

#include <Ps3Controller.h>

namespace
{
    volatile int gLx = 0;
    volatile int gLy = 0;
    volatile bool gTriangle = false;
    volatile bool gCross = false;
    volatile unsigned long gLastNotifyMs = 0;
    volatile bool gDisconnectLogged = false;
    bool gConnected = false;
    ILogger *gLogger = nullptr;

    void onNotify()
    {
        gLastNotifyMs = millis();
        gLx = Ps3.data.analog.stick.lx;
        gLy = Ps3.data.analog.stick.ly;

        if (gLx >= 0 && gLx < PS3_DEADZONE)
            gLx = 0;
        if (gLx <= 0 && gLx > -PS3_DEADZONE)
            gLx = 0;
        if (gLy >= 0 && gLy < PS3_DEADZONE)
            gLy = 0;
        if (gLy <= 0 && gLy > -PS3_DEADZONE)
            gLy = 0;

        gTriangle = Ps3.data.button.triangle;
        gCross = Ps3.data.button.cross;
    }

    void onConnect()
    {
        gConnected = true;
        if (gLogger)
            gLogger->info("PS3", "Controller connected");
    }
}

CommandInputPs3::CommandInputPs3(ILogger *logger)
{
    gLogger = logger;
}

void CommandInputPs3::begin()
{
    Ps3.attach(onNotify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin(PS3_MAC_ADDRESS);
}

bool CommandInputPs3::update(Command &command, Mode &mode)
{
    MotionVector mv{};
    const bool motionChanged = updateMotion(mv);

    const bool connected = (millis() - gLastNotifyMs <= PS3_DISCONNECT_TIMEOUT_MS);
    Command newCmd = Command::NONE;

    if (connected)
    {
        if (mv.buttonCommand != Command::NONE)
        {
            newCmd = mv.buttonCommand;
        }
        else if (mv.forward > 0)
        {
            newCmd = Command::FORWARD;
        }
        else if (mv.forward < 0)
        {
            newCmd = Command::BACKWARD;
        }
    }

    if (newCmd != command)
    {
        command = newCmd;
        return true;
    }
    return motionChanged;
}

bool CommandInputPs3::updateMotion(MotionVector &mv)
{
    const bool connected = (millis() - gLastNotifyMs <= PS3_DISCONNECT_TIMEOUT_MS);

    if (!connected)
    {
        if (!gDisconnectLogged)
        {
            gDisconnectLogged = true;
            gConnected = false;
            if (gLogger)
                gLogger->info("PS3", "Controller disconnected (timeout)");
        }
        MotionVector zero{};
        const bool wasActive = (mv.forward != 0 || mv.turn != 0 || mv.buttonCommand != Command::NONE);
        mv = zero;
        return wasActive;
    }

    gDisconnectLogged = false;
    MotionVector newMv{};
    static bool prevTriangle = false;
    static bool prevCross = false;

    const bool triangleEdge = gTriangle && !prevTriangle;
    const bool crossEdge = gCross && !prevCross;
    prevTriangle = gTriangle;
    prevCross = gCross;

    if (gTriangle)
    {
        newMv.buttonCommand = Command::STAND_UP;
        if (triangleEdge)
            gLogger->debug("PS3", "Triangle pressed");
    }
    else if (gCross)
    {
        newMv.buttonCommand = Command::STAND_DOWN;
        if (crossEdge)
            gLogger->debug("PS3", "Cross pressed");
    }
    else
        newMv.buttonCommand = Command::NONE;

    newMv.forward = static_cast<int16_t>(-gLy);
    newMv.turn = static_cast<int16_t>(gLx);

    if (newMv.forward > FORWARD_SATURATION_MAX)
        newMv.forward = FORWARD_SATURATION_MAX;
    else if (newMv.forward < -FORWARD_SATURATION_MAX)
        newMv.forward = -FORWARD_SATURATION_MAX;

    if (newMv.turn > TURN_SATURATION_MAX)
        newMv.turn = TURN_SATURATION_MAX;
    else if (newMv.turn < -TURN_SATURATION_MAX)
        newMv.turn = -TURN_SATURATION_MAX;

    const bool changed = (abs(newMv.forward - mv.forward) > MOTION_HYSTERESIS) ||
                         (abs(newMv.turn - mv.turn) > MOTION_HYSTERESIS) ||
                         (newMv.buttonCommand != mv.buttonCommand);

    if (changed)
    {
        static unsigned long lastMotionLogMs = 0;
        unsigned long now = millis();
        if (now - lastMotionLogMs >= 100)
        {
            gLogger->debug("PS3", newMv.toString());
            lastMotionLogMs = now;
        }
        mv = newMv;
    }

    return changed;
}

#else

CommandInputPs3::CommandInputPs3(ILogger *logger) {}

void CommandInputPs3::begin() {}

bool CommandInputPs3::update(Command &command, Mode &mode)
{
    (void)command;
    (void)mode;
    return false;
}

bool CommandInputPs3::updateMotion(MotionVector &motion)
{
    (void)motion;
    return false;
}

#endif
