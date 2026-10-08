#pragma once

#include <Arduino.h>

#include "config/system_config.h"
#include "domain/Command.h"
#include "domain/Mode.h"
#include "domain/ICommandInput.h"
#include "domain/ILogger.h"
#include "platform/ModeSharedState.h"

#if ENABLE_MICROROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#endif

/**
 * @brief micro-ROS command input from the vision node.
 *
 * Subscribes to MICRO_ROS_VISION_TOPIC (std_msgs/msg/String) and maps
 * the Indonesian command strings published by vision_avoidance_ros2 to
 * the internal Command enum:
 *
 *   "maju"     -> FORWARD
 *   "mundur"   -> BACKWARD
 *   "kiri"     -> TURN_LEFT
 *   "kanan"    -> TURN_RIGHT
 *   "merunduk" -> STAND_DOWN (sit / duck)
 *   "berdiri"  -> STAND_UP
 *   "berhenti" / "stop" -> NONE
 *
 * Commands are only accepted while Mode::AUTO. A watchdog stops the
 * robot when no message arrives for MICRO_ROS_WATCHDOG_TIMEOUT_MS.
 */
class CommandInputMicroRos : public ICommandInput
{
public:
    CommandInputMicroRos(ILogger &logger, ModeSharedState &modeState);

    void begin() override;

    bool update(Command &command, Mode &mode) override;

    /** Called by the executor (via g_instance) on each incoming message. */
    void subscriptionCallback(const void *msgin);

private:
    ILogger &logger_;
    ModeSharedState &modeState_;

    Command lastCommand_ = Command::NONE;
    bool hasNewCommand_ = false;
    unsigned long lastMessageMs_ = 0;
    unsigned long lastRetryMs_ = 0;
    bool nodeReady_ = false;

#if ENABLE_MICROROS
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rcl_subscription_t subscription_;
    rclc_executor_t executor_;
    std_msgs__msg__String msg_;

    void tearDown();
    bool initAgent();
#endif
};
