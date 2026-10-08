#pragma once

#include <Arduino.h>
#include "domain/Command.h"
#include "domain/IDynamixelBus.h"
#include "domain/ILogger.h"
#include "domain/MotionVector.h"

struct PoseKeyframe;

/**
 * @brief Leg pair index constants used for KNEE_DIRECTION lookup.
 */
enum LegIndex : uint8_t
{
    LEG_FR = 0,
    LEG_FL = 1,
    LEG_BR = 2,
    LEG_BL = 3
};

/**
 * @brief Phase step indices for the 8-phase blended gait cycle.
 *
 * Each leg pair (diagonal FR+BL, then FL+BR) cycles through four phases:
 *   LIFT  — knee bends, hip at neutral (foot leaves ground)
 *   SWING — knee bent, hip moves to computed position (foot swings forward)
 *   PLANT — knee straightens to idle, hip stays forward (foot contacts ground)
 *   PUSH  — hip retracts back to neutral, knee stays idle (body pulled forward)
 */
enum GaitPhaseIdx : uint8_t
{
    PHASE_LIFT_FR_BL = 0,
    PHASE_SWING_FR_BL = 1,
    PHASE_PLANT_FR_BL = 2,
    PHASE_PUSH_FR_BL = 3,
    PHASE_LIFT_FL_BR = 4,
    PHASE_SWING_FL_BR = 5,
    PHASE_PLANT_FL_BR = 6,
    PHASE_PUSH_FL_BR = 7,
    PHASE_COUNT = 8
};

/**
 * @brief Leg-to-phase mapping: which two legs are active in each phase.
 */
struct PhaseLegPair
{
    uint8_t hipA;
    uint8_t kneeA;
    uint8_t hipB;
    uint8_t kneeB;
    int8_t dirA;      // KNEE_DIRECTION for leg A
    int8_t dirB;      // KNEE_DIRECTION for leg B
    LegIndex legIdxA; // LegIndex for leg A
    LegIndex legIdxB; // LegIndex for leg B
};

/**
 * @brief Robot state machine: prevents walking while sitting/standing.
 */
enum class RobotState : uint8_t
{
    IDLE,
    WALKING,
    STANDING_UP,
    SITTING_DOWN,
    SITTING
};

/**
 * @brief Orchestrates leg movement sequences (gait patterns) based on a command
 *        or a continuous MotionVector.
 */
class GaitService
{
public:
    /**
     * @param bus Reference to a concrete Dynamixel bus driver.
     * @param logger Optional logger instance.
     */
    GaitService(IDynamixelBus &bus, ILogger &logger);

    /**
     * @brief Enables torque on all servos and moves to the start pose.
     */
    void initialize();

    /**
     * @brief Executes one gait cycle for the given discrete command.
     * @param command The command to execute.
     */
    void run(Command command);

    /**
     * @brief Processes one phase of the blended gait from a continuous MotionVector.
     *
     * Call this from loop() at a high rate (≥50 Hz).  It advances the gait phase
     * only when the phase timer expires (non-blocking).
     *
     * @param motion The motion vector (forward + turn blended).
     * @return true if a servo write occurred this call.
     */
    bool run(MotionVector motion);

    /**
     * @brief Returns the current robot state.
     */
    RobotState getState() const { return state_; }

private:
    IDynamixelBus &bus_;
    ILogger &logger_;

    uint8_t phase_;
    unsigned long phaseStartMs_;
    MotionVector lastMotion_;
    bool motionActive_;
    bool stopping_;
    RobotState state_;
    uint8_t readFailCount_;
    int32_t prevKnee_[4]; // [LEG_FR, LEG_FL, LEG_BR, LEG_BL]
    Command lastStandSitCmd_ = Command::NONE;

    void writeLeg(int32_t posHip, int32_t posKnee, uint8_t idHip, uint8_t idKnee);
    void initLegPose();

    void runStandUp();
    void runSitDown();
    void runPoseSequence(const PoseKeyframe* keyframes, uint8_t count);

    void advancePhase(const MotionVector &motion);
    void writeBlendedPhase(const MotionVector &motion);
    bool isPhaseComplete();
    void resetPhaseMachine();
    bool isWalking() const { return state_ == RobotState::WALKING; }

    /**
     * @brief Maps a raw stick value (±SATURATION_MAX) to a hip swing amplitude.
     */
    int32_t forwardToHip(int16_t forward) const;

    /**
     * @brief Maps a raw stick value (±SATURATION_MAX) to a turn offset.
     */
    int32_t turnToOffset(int16_t turn) const;

    /**
     * @brief Computes per-leg hip offset = forwardBias + turnBias, then clamps.
     */
    int32_t computeHipPosition(int16_t forward, int16_t turn, LegIndex leg) const;
};