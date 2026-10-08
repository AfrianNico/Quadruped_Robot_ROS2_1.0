#include "GaitService.h"
#include "config/servo_config.h"
#include "config/gait_config.h"
#include "config/system_config.h"
#include "utils/Easing.h"

static const PhaseLegPair PHASE_LEGS[PHASE_COUNT] = {
    // Phase 0: lift FR + BL (knee bends, hip at neutral)
    {ID_HIP_FR, ID_KNEE_FR, ID_HIP_BL, ID_KNEE_BL, KNEE_DIRECTION[LEG_FR], KNEE_DIRECTION[LEG_BL], LEG_FR, LEG_BL},
    // Phase 1: swing FR + BL (hip swings forward/back, knee stays bent)
    {ID_HIP_FR, ID_KNEE_FR, ID_HIP_BL, ID_KNEE_BL, KNEE_DIRECTION[LEG_FR], KNEE_DIRECTION[LEG_BL], LEG_FR, LEG_BL},
    // Phase 2: plant FR + BL (knee extends to idle, hip stays forward)
    {ID_HIP_FR, ID_KNEE_FR, ID_HIP_BL, ID_KNEE_BL, KNEE_DIRECTION[LEG_FR], KNEE_DIRECTION[LEG_BL], LEG_FR, LEG_BL},
    // Phase 3: push FR + BL (hip retracts to neutral, knee stays idle)
    {ID_HIP_FR, ID_KNEE_FR, ID_HIP_BL, ID_KNEE_BL, KNEE_DIRECTION[LEG_FR], KNEE_DIRECTION[LEG_BL], LEG_FR, LEG_BL},
    // Phase 4: lift FL + BR
    {ID_HIP_FL, ID_KNEE_FL, ID_HIP_BR, ID_KNEE_BR, KNEE_DIRECTION[LEG_FL], KNEE_DIRECTION[LEG_BR], LEG_FL, LEG_BR},
    // Phase 5: swing FL + BR
    {ID_HIP_FL, ID_KNEE_FL, ID_HIP_BR, ID_KNEE_BR, KNEE_DIRECTION[LEG_FL], KNEE_DIRECTION[LEG_BR], LEG_FL, LEG_BR},
    // Phase 6: plant FL + BR
    {ID_HIP_FL, ID_KNEE_FL, ID_HIP_BR, ID_KNEE_BR, KNEE_DIRECTION[LEG_FL], KNEE_DIRECTION[LEG_BR], LEG_FL, LEG_BR},
    // Phase 7: push FL + BR (hip retracts to neutral, knee stays idle)
    {ID_HIP_FL, ID_KNEE_FL, ID_HIP_BR, ID_KNEE_BR, KNEE_DIRECTION[LEG_FL], KNEE_DIRECTION[LEG_BR], LEG_FL, LEG_BR},
};

static const int32_t IDLE_HIP[4] = {INIT_POSE_FR.hip, INIT_POSE_FL.hip, INIT_POSE_BR.hip, INIT_POSE_BL.hip};
static const int32_t IDLE_KNEE[4] = {INIT_POSE_FR.knee, INIT_POSE_FL.knee, INIT_POSE_BR.knee, INIT_POSE_BL.knee};

GaitService::GaitService(IDynamixelBus &bus, ILogger &logger)
    : bus_(bus), logger_(logger),
      phase_(0), phaseStartMs_(0),
      lastMotion_{}, motionActive_(false), stopping_(false),
      state_(RobotState::IDLE), readFailCount_(0),
      prevKnee_{INIT_POSE_FR.knee, INIT_POSE_FL.knee, INIT_POSE_BR.knee, INIT_POSE_BL.knee}
{
}

void GaitService::initialize()
{
    logger_.info("GaitSvc", "Init start");

    bus_.torqueEnable(ID_HIP_FR, true);
    bus_.torqueEnable(ID_HIP_FL, true);
    bus_.torqueEnable(ID_HIP_BR, true);
    bus_.torqueEnable(ID_HIP_BL, true);
    delay(20);

    bus_.torqueEnable(ID_KNEE_FR, true);
    bus_.torqueEnable(ID_KNEE_FL, true);
    bus_.torqueEnable(ID_KNEE_BR, true);
    bus_.torqueEnable(ID_KNEE_BL, true);
    delay(20);

    bus_.writeLegSpeedRaw(HIP_PROFILE_VEL, KNEE_PROFILE_VEL, ID_HIP_FR, ID_KNEE_FR);
    bus_.writeLegSpeedRaw(HIP_PROFILE_VEL, KNEE_PROFILE_VEL, ID_HIP_FL, ID_KNEE_FL);
    bus_.writeLegSpeedRaw(HIP_PROFILE_VEL, KNEE_PROFILE_VEL, ID_HIP_BR, ID_KNEE_BR);
    bus_.writeLegSpeedRaw(HIP_PROFILE_VEL, KNEE_PROFILE_VEL, ID_HIP_BL, ID_KNEE_BL);
    delay(500);

    initLegPose();
    resetPhaseMachine();
    logger_.info("GaitSvc", "Init done");
}

void GaitService::run(Command command)
{
    switch (command)
    {
    case Command::FORWARD:
        run(MotionVector{FORWARD_SATURATION_MAX, 0, Command::NONE});
        break;
    case Command::BACKWARD:
        run(MotionVector{static_cast<int16_t>(-FORWARD_SATURATION_MAX), 0, Command::NONE});
        break;
    case Command::TURN_LEFT:
        run(MotionVector{0, static_cast<int16_t>(-FORWARD_SATURATION_MAX), Command::NONE});
        break;
    case Command::TURN_RIGHT:
        run(MotionVector{0, FORWARD_SATURATION_MAX, Command::NONE});
        break;
    case Command::STAND_UP:
        if (state_ == RobotState::IDLE || state_ == RobotState::STANDING_UP)
            break;
        runStandUp();
        break;
    case Command::STAND_DOWN:
        if (state_ == RobotState::SITTING)
            break;
        runSitDown();
        break;
    case Command::DUCK:
        // Merunduk: untuk saat ini memakai pose crouch yang sama dengan
        // sit-down, tapi menggunakan enum Command sendiri agar mudah
        // dikustomisasi (keyframe/state) di kemudian hari.
        if (state_ == RobotState::SITTING)
            break;
        runSitDown();
        break;
    case Command::NONE:
    default:
        run(MotionVector{0, 0, Command::NONE});
        break;
    }
}

bool GaitService::run(MotionVector motion)
{
    if (motion.buttonCommand == Command::STAND_UP)
    {
        if (state_ == RobotState::STANDING_UP ||
            state_ == RobotState::IDLE ||
            state_ == RobotState::WALKING)
            return true;

        if (lastStandSitCmd_ != Command::STAND_UP)
        {
            lastStandSitCmd_ = Command::STAND_UP;
            logger_.info("GaitSvc", "Stand up requested");
            state_ = RobotState::STANDING_UP;
            runStandUp();
            resetPhaseMachine();
        }
        return true;
    }

    if (motion.buttonCommand == Command::STAND_DOWN)
    {
        if (state_ == RobotState::SITTING)
            return true;

        if (lastStandSitCmd_ != Command::STAND_DOWN)
        {
            lastStandSitCmd_ = Command::STAND_DOWN;
            logger_.info("GaitSvc", "Sit down requested");
            state_ = RobotState::SITTING_DOWN;
            runSitDown();
            resetPhaseMachine();
        }
        return true;
    }

    lastStandSitCmd_ = Command::NONE;

    if (state_ == RobotState::STANDING_UP ||
        state_ == RobotState::SITTING_DOWN ||
        state_ == RobotState::SITTING)
    {
        return false;
    }

    if (stopping_)
    {
        stopping_ = false;
        motionActive_ = false;
        state_ = RobotState::IDLE;

        initLegPose();
        resetPhaseMachine();
        return true;
    }

    const bool isZero = (motion.forward == 0 && motion.turn == 0);

    if (isZero && motionActive_)
    {
        stopping_ = true;
        return false;
    }

    if (isZero)
    {
        return false;
    }

    if (!motionActive_)
    {
        motionActive_ = true;
        state_ = RobotState::WALKING;
        lastMotion_ = motion;
        resetPhaseMachine();
        writeBlendedPhase(motion);
        return true;
    }

    if (isPhaseComplete())
    {
        lastMotion_ = motion;
        advancePhase(motion);
        return true;
    }

    return false;
}

void GaitService::resetPhaseMachine()
{
    phase_ = 0;
    phaseStartMs_ = millis();
    readFailCount_ = 0;
}

void GaitService::advancePhase(const MotionVector &motion)
{
    const uint8_t prevPhase = phase_;
    phase_ = (phase_ + 1) % PHASE_COUNT;
    phaseStartMs_ = millis();
    logger_.debug("GaitPhs", "Advance " + String(prevPhase) + " -> " + String(phase_));
    writeBlendedPhase(motion);
}

bool GaitService::isPhaseComplete()
{
    const unsigned long elapsed = millis() - phaseStartMs_;
    const uint32_t expectedDelay = PHASE_DELAY_TABLE[phase_];

#if ENABLE_SERVO_READBACK
    const PhaseLegPair &legs = PHASE_LEGS[phase_];

    unsigned long t0 = micros();
    const bool movingHipA = bus_.isMoving(legs.hipA);
    unsigned long t1 = micros() - t0;
    if (t1 > 5000)
        logger_.debug("GaitPhs", "isMoving hipA id=" + String(legs.hipA) + " took " + String(t1) + "us");

    t0 = micros();
    const bool movingKneeA = bus_.isMoving(legs.kneeA);
    unsigned long t2 = micros() - t0;
    if (t2 > 5000)
        logger_.debug("GaitPhs", "isMoving kneeA id=" + String(legs.kneeA) + " took " + String(t2) + "us");

    t0 = micros();
    const bool movingHipB = bus_.isMoving(legs.hipB);
    unsigned long t3 = micros() - t0;
    if (t3 > 5000)
        logger_.debug("GaitPhs", "isMoving hipB id=" + String(legs.hipB) + " took " + String(t3) + "us");

    t0 = micros();
    const bool movingKneeB = bus_.isMoving(legs.kneeB);
    unsigned long t4 = micros() - t0;
    if (t4 > 5000)
        logger_.debug("GaitPhs", "isMoving kneeB id=" + String(legs.kneeB) + " took " + String(t4) + "us");

    const bool anyMoving = movingHipA || movingKneeA || movingHipB || movingKneeB;

    if (!anyMoving)
    {
        if (elapsed >= expectedDelay)
        {
            readFailCount_ = 0;
            return true;
        }
        return false;
    }

    if (elapsed >= expectedDelay)
    {
        readFailCount_++;
        if (readFailCount_ >= READBACK_FAIL_THRESHOLD)
        {
            return true;
        }
    }
    return false;
#endif
    return (elapsed >= expectedDelay);
}

void GaitService::writeBlendedPhase(const MotionVector &motion)
{
    const PhaseLegPair &legs = PHASE_LEGS[phase_];
    const bool isLiftPhase = (phase_ == PHASE_LIFT_FR_BL || phase_ == PHASE_LIFT_FL_BR);
    const bool isSwingPhase = (phase_ == PHASE_SWING_FR_BL || phase_ == PHASE_SWING_FL_BR);
    const bool isPlantPhase = (phase_ == PHASE_PLANT_FR_BL || phase_ == PHASE_PLANT_FL_BR);
    const bool isPushPhase = (phase_ == PHASE_PUSH_FR_BL || phase_ == PHASE_PUSH_FL_BR);

    const int32_t computedA = computeHipPosition(motion.forward, motion.turn, legs.legIdxA);
    const int32_t computedB = computeHipPosition(motion.forward, motion.turn, legs.legIdxB);

    const int32_t idleHipA = IDLE_HIP[legs.legIdxA];
    const int32_t idleHipB = IDLE_HIP[legs.legIdxB];
    const int32_t idleKneeA = IDLE_KNEE[legs.legIdxA];
    const int32_t idleKneeB = IDLE_KNEE[legs.legIdxB];

    int32_t hipA, hipB;
    if (isLiftPhase || isPushPhase)
    {
        hipA = idleHipA;
        hipB = idleHipB;
    }
    else
    {
        hipA = idleHipA + computedA;
        hipB = idleHipB + computedB;
    }

    int32_t kneeA, kneeB;
    if (isLiftPhase || isSwingPhase)
    {
        kneeA = idleKneeA + legs.dirA * KNEE_LIFT_AMOUNT;
        kneeB = idleKneeB + legs.dirB * KNEE_LIFT_AMOUNT;
    }
    else
    {
        kneeA = idleKneeA;
        kneeB = idleKneeB;
    }

    hipA = constrain(hipA, -HIP_SATURATION_LIMIT, HIP_SATURATION_LIMIT);
    hipB = constrain(hipB, -HIP_SATURATION_LIMIT, HIP_SATURATION_LIMIT);
    kneeA = constrain(kneeA, -KNEE_SATURATION_LIMIT, KNEE_SATURATION_LIMIT);
    kneeB = constrain(kneeB, -KNEE_SATURATION_LIMIT, KNEE_SATURATION_LIMIT);

    writeLeg(hipA, kneeA, legs.hipA, legs.kneeA);
    writeLeg(hipB, kneeB, legs.hipB, legs.kneeB);

    logger_.debug("GaitPhs", "ph=" + String(phase_) +
                                 " fwd=" + String(motion.forward) + " trn=" + String(motion.turn) +
                                 " hipA=" + String(hipA) + " kneeA=" + String(kneeA) +
                                 " hipB=" + String(hipB) + " kneeB=" + String(kneeB));
}

int32_t GaitService::forwardToHip(int16_t forward) const
{
    return static_cast<int32_t>(forward) * MAX_HIP_SWING / FORWARD_SATURATION_MAX;
}

int32_t GaitService::turnToOffset(int16_t turn) const
{
    return static_cast<int32_t>(turn) * MAX_TURN_OFFSET / TURN_SATURATION_MAX;
}

int32_t GaitService::computeHipPosition(int16_t forward, int16_t turn, LegIndex leg) const
{
    const int32_t fwd = forwardToHip(forward);
    const int32_t trn = turnToOffset(turn);

    int32_t hip;
    switch (leg)
    {
    case LEG_FR:
    case LEG_BR:
        hip = +fwd - trn; // right side: positive = forward
        break;
    case LEG_FL:
    case LEG_BL:
        hip = -fwd - trn; // left side:  negative = forward
        break;
    }

    return constrain(hip, -HIP_SATURATION_LIMIT, HIP_SATURATION_LIMIT);
}

void GaitService::writeLeg(int32_t posHip, int32_t posKnee, uint8_t idHip, uint8_t idKnee)
{
    bus_.writeLegGoal(posHip, posKnee, idHip, idKnee);
}

void GaitService::initLegPose()
{
    constexpr uint32_t INTER_WRITE_DELAY_MS = 5;

    writeLeg(INIT_POSE_FR.hip, INIT_POSE_FR.knee, ID_HIP_FR, ID_KNEE_FR);
    delay(INTER_WRITE_DELAY_MS);
    writeLeg(INIT_POSE_FL.hip, INIT_POSE_FL.knee, ID_HIP_FL, ID_KNEE_FL);
    delay(INTER_WRITE_DELAY_MS);
    writeLeg(INIT_POSE_BR.hip, INIT_POSE_BR.knee, ID_HIP_BR, ID_KNEE_BR);
    delay(INTER_WRITE_DELAY_MS);
    writeLeg(INIT_POSE_BL.hip, INIT_POSE_BL.knee, ID_HIP_BL, ID_KNEE_BL);
}

void GaitService::runPoseSequence(const PoseKeyframe* keyframes, uint8_t count)
{
    if (count == 0) return;

    int32_t startKnee[4] = {prevKnee_[0], prevKnee_[1], prevKnee_[2], prevKnee_[3]};

    for (uint8_t k = 0; k < count; k++)
    {
        const int32_t targetKnee[4] = {
            keyframes[k].kneeFR,
            keyframes[k].kneeFL,
            keyframes[k].kneeBR,
            keyframes[k].kneeBL
        };
        const uint16_t duration = keyframes[k].durationMs;

        if (duration == 0)
        {
            for (int i = 0; i < 4; i++)
                prevKnee_[i] = startKnee[i] = targetKnee[i];
            continue;
        }

        unsigned long elapsed = 0;
        while (elapsed < duration)
        {
            const float t = (float)elapsed / (float)duration;
            const float eased = easeInOutCubic(t);

            int32_t knee[4];
            for (int i = 0; i < 4; i++)
            {
                const float diff = (float)(targetKnee[i] - startKnee[i]);
                knee[i] = startKnee[i] + (int32_t)(diff * eased);
            }

            writeLeg(0, knee[LEG_FR], ID_HIP_FR, ID_KNEE_FR);
            writeLeg(0, knee[LEG_FL], ID_HIP_FL, ID_KNEE_FL);
            writeLeg(0, knee[LEG_BR], ID_HIP_BR, ID_KNEE_BR);
            writeLeg(0, knee[LEG_BL], ID_HIP_BL, ID_KNEE_BL);

            delay(STAND_SIT_STEP_MS);
            elapsed += STAND_SIT_STEP_MS;
        }

        writeLeg(0, targetKnee[LEG_FR], ID_HIP_FR, ID_KNEE_FR);
        writeLeg(0, targetKnee[LEG_FL], ID_HIP_FL, ID_KNEE_FL);
        writeLeg(0, targetKnee[LEG_BR], ID_HIP_BR, ID_KNEE_BR);
        writeLeg(0, targetKnee[LEG_BL], ID_HIP_BL, ID_KNEE_BL);

        for (int i = 0; i < 4; i++)
            prevKnee_[i] = startKnee[i] = targetKnee[i];
    }
}

void GaitService::runStandUp()
{
    if (state_ == RobotState::IDLE ||
        state_ == RobotState::WALKING)
        return;

    logger_.debug("GaitSvc", "Stand up sequence");
    runPoseSequence(STAND_UP_KEYFRAMES, 5);
    state_ = RobotState::IDLE;
}

void GaitService::runSitDown()
{
    if (state_ == RobotState::SITTING)
        return;

    logger_.debug("GaitSvc", "Sit down sequence");
    runPoseSequence(SIT_DOWN_KEYFRAMES, 5);
    state_ = RobotState::SITTING;
}