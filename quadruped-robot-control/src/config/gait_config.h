#pragma once

#include <Arduino.h>
#include "servo_config.h"

/**
 * @brief Gait timing: delay (ms) between each leg movement phase step.
 */
// DEPRECATED: replaced by PHASE_DELAY_TABLE in blended state machine
// constexpr uint32_t GAIT_STEP_DELAY_MS = 200; // 150

/**
 * @brief Idle / init leg pose offsets (hip, knee) for each of the 4 legs.
 *
 * These produce the asymmetric neutral standing position:
 *   FR: (0, 300)   FL: (0, -400)
 *   BR: (0, -400)  BL: (0, -300)
 */
struct LegPose
{
    int32_t hip;
    int32_t knee;
};

constexpr LegPose INIT_POSE_FR = {0, 200};  // 300
constexpr LegPose INIT_POSE_FL = {0, -300}; // -400
constexpr LegPose INIT_POSE_BR = {0, -300}; // -400
constexpr LegPose INIT_POSE_BL = {0, -200}; // -300

/**
 * @brief Maximum absolute hip offset (saturation clamp) per leg.
 *
 * Prevents overflow when forward + turn amplitudes exceed servo travel limits.
 * Applied via constrain() in GaitService before every writeLegGoal() call.
 */
constexpr int32_t HIP_SATURATION_LIMIT = 850; // 450 300 270 370

/**
 * @brief Maximum absolute knee offset (saturation clamp) per leg.
 */
constexpr int32_t KNEE_SATURATION_LIMIT = 1200;

/**
 * @brief Maximum forward/backward hip swing amplitude.
 *
 * Maps to the full stick deflection when motion.forward == ±127.
 */
constexpr int16_t MAX_HIP_SWING = 250; // 350 200 150

/**
 * @brief Maximum turn offset (differential stride asymmetry).
 *
 * Smaller than MAX_HIP_SWING so that forward + turn stays within
 * HIP_SATURATION_LIMIT. (350 + 175 = 525 → clamped to 450)
 */
constexpr int16_t MAX_TURN_OFFSET = 500; // 175 100 70 120 200

/**
 * @brief Knee lift magnitude during trot phase (positive = lift direction,
 * actual sign per leg determined by KNEE_DIRECTION[] in GaitService).
 */
constexpr int32_t KNEE_LIFT_AMOUNT = 200; // 300 150 300 500

/**
 * @brief Per-leg knee direction multipliers.
 *
 * Index order: FR=0, FL=1, BR=2, BL=3
 * Determines the direction to move the knee servo when bending (lifting the foot):
 *   -1 = servo moves toward lower encoder values to bend
 *   +1 = servo moves toward higher encoder values to bend
 *
 */
constexpr int8_t KNEE_DIRECTION[4] = {-1, +1, +1, +1};

/**
 * @brief Static per-phase delay table (milliseconds) — fallback when
 * ENABLE_SERVO_READBACK is false or read attempts fail consecutively.
 *
 */
constexpr uint32_t PHASE_DELAY_TABLE[8] = {50, 50, 40, 40, 50, 50, 40, 40};

/**
 * @brief Consecutive read failures before auto-switching to static delay table.
 */
constexpr uint8_t READBACK_FAIL_THRESHOLD = 3;

/**
 * @brief Micro-step interval (ms) for stand-up/sit-down interpolation.
 * Each keyframe transition is subdivided into steps of this duration,
 * with eased intermediate positions sent to the servos.
 */
constexpr uint16_t STAND_SIT_STEP_MS = 20;

/**
 * @brief A single keyframe in a stand-up or sit-down pose sequence.
 *
 * Stores per-leg knee target positions and the duration over which
 * to approach them (eased from the previous keyframe).
 */
struct PoseKeyframe
{
    int32_t kneeFR;
    int32_t kneeFL;
    int32_t kneeBR;
    int32_t kneeBL;
    uint16_t durationMs;
};

/**
 * @brief Stand-up keyframe sequence (absolute knee positions).
 *
 * Extracted from the original 5-phase runStandUp():
 *   Phase 1 (20 %)  —  80 ms
 *   Phase 2 (45 %)  — 100 ms
 *   Phase 3 (70 %)  — 130 ms
 *   Phase 4 (90 %)  — 160 ms
 *   Phase 5 (100 %) — 200 ms  → idle/init pose
 */
static constexpr PoseKeyframe STAND_UP_KEYFRAMES[5] = {
    //     kneeFR  kneeFL  kneeBR  kneeBL  dur
    {     -120,      0,     20,     40,    80  },
    {      -20,    -94,    -80,    -35,   100  },
    {       80,   -188,   -180,   -110,   130  },
    {      160,   -263,   -260,   -170,   160  },
    {      200,   -300,   -300,   -200,   200  },
};

/**
 * @brief Sit-down keyframe sequence (absolute knee positions).
 *
 * Extracted from the original 5-phase runSitDown():
 *   Phase 1 (20 %)  —  80 ms
 *   Phase 2 (45 %)  — 100 ms
 *   Phase 3 (70 %)  — 130 ms
 *   Phase 4 (90 %)  — 160 ms
 *   Phase 5 (100 %) — 200 ms  → full sit pose
 */
static constexpr PoseKeyframe SIT_DOWN_KEYFRAMES[5] = {
    //     kneeFR  kneeFL  kneeBR  kneeBL  dur
    {      120,   -225,   -220,   -140,    80  },
    {       20,   -131,   -120,    -65,   100  },
    {      -80,    -38,    -20,     10,   130  },
    {     -160,     38,     60,     70,   160  },
    {     -200,     75,    100,    100,   200  },
};