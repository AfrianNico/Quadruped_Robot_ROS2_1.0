#include <cstring>

#include "driver/CommandInputMicroRos.h"

// Forward declaration; subscription callback needs access to the instance.
static CommandInputMicroRos *g_instance = nullptr;

CommandInputMicroRos::CommandInputMicroRos(ILogger &logger, ModeSharedState &modeState)
    : logger_(logger), modeState_(modeState)
{
    g_instance = this;
}

void CommandInputMicroRos::subscriptionCallback(const void *msgin)
{
    const auto *msg = static_cast<const std_msgs__msg__String *>(msgin);
    if (msg->data.data == nullptr)
        return;

    char buf[24];
    std::strncpy(buf, msg->data.data, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // lowercase in-place
    for (size_t i = 0; buf[i] != '\0'; ++i)
    {
        if (buf[i] >= 'A' && buf[i] <= 'Z')
            buf[i] = static_cast<char>(buf[i] - 'A' + 'a');
    }

    Command parsed = Command::NONE;
    bool recognized = true;

    if (std::strcmp(buf, "maju") == 0)
        parsed = Command::FORWARD;
    else if (std::strcmp(buf, "mundur") == 0)
        parsed = Command::BACKWARD;
    else if (std::strcmp(buf, "kiri") == 0)
        parsed = Command::TURN_LEFT;
    else if (std::strcmp(buf, "kanan") == 0)
        parsed = Command::TURN_RIGHT;
    else if (std::strcmp(buf, "merunduk") == 0)
        parsed = Command::DUCK;
    else if (std::strcmp(buf, "berdiri") == 0)
        parsed = Command::STAND_UP;
    else if (std::strcmp(buf, "berhenti") == 0 || std::strcmp(buf, "stop") == 0)
        parsed = Command::NONE;
    else
        recognized = false;

    if (recognized)
    {
        lastCommand_ = parsed;
        hasNewCommand_ = true;
        lastMessageMs_ = millis();
    }
}

#if ENABLE_MICROROS

static void stringSubCallback(const void *msgin)
{
    if (g_instance != nullptr)
        g_instance->subscriptionCallback(msgin);
}

void CommandInputMicroRos::tearDown()
{
    if (nodeReady_)
    {
        rcl_subscription_fini(&subscription_, &node_);
        rcl_node_fini(&node_);
        rclc_executor_fini(&executor_);
        rclc_support_fini(&support_);
        nodeReady_ = false;
    }
}

bool CommandInputMicroRos::initAgent()
{
    if (MICRO_ROS_USE_SERIAL)
    {
        // ESP32 default transport = Serial (UART0), sudah Serial.begin() di setup()
        set_microros_transports();
    }
    else
    {
        char ssidBuf[strlen(MICRO_ROS_WIFI_SSID) + 1];
        char passBuf[strlen(MICRO_ROS_WIFI_PASSWORD) + 1];
        char ipBuf[strlen(MICRO_ROS_AGENT_IP) + 1];
        std::strcpy(ssidBuf, MICRO_ROS_WIFI_SSID);
        std::strcpy(passBuf, MICRO_ROS_WIFI_PASSWORD);
        std::strcpy(ipBuf, MICRO_ROS_AGENT_IP);
        set_microros_wifi_transports(ssidBuf, passBuf, ipBuf, MICRO_ROS_AGENT_PORT);
    }

    allocator_ = rcl_get_default_allocator();

    if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&node_, MICRO_ROS_NODE_NAME, "", &support_) != RCL_RET_OK)
        return false;

    if (rclc_subscription_init_default(&subscription_, &node_,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                       MICRO_ROS_VISION_TOPIC) != RCL_RET_OK)
        return false;

    msg_.data.capacity = 64;
    msg_.data.data = static_cast<char *>(allocator_.allocate(msg_.data.capacity, allocator_.state));
    if (msg_.data.data == nullptr)
        return false;
    msg_.data.size = 0;
    msg_.data.data[0] = '\0';

    if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) != RCL_RET_OK)
        return false;

    if (rclc_executor_add_subscription(&executor_, &subscription_, &msg_,
                                       &stringSubCallback, ON_NEW_DATA) != RCL_RET_OK)
        return false;

    return true;
}

#endif

void CommandInputMicroRos::begin()
{
#if ENABLE_MICROROS
    logger_.info("MicroROS", "Connecting to agent " + String(MICRO_ROS_AGENT_IP) +
                                 ":" + String(MICRO_ROS_AGENT_PORT));
    if (initAgent())
    {
        nodeReady_ = true;
        lastMessageMs_ = millis();
        logger_.info("MicroROS", "Node created, subscribing to " + String(MICRO_ROS_VISION_TOPIC));
    }
    else
    {
        logger_.warn("MicroROS", "Init failed, will retry in loop");
    }
#endif
}

bool CommandInputMicroRos::update(Command &command, Mode &mode)
{
#if ENABLE_MICROROS
    if (!nodeReady_)
    {
        unsigned long now = millis();
        if (now - lastRetryMs_ >= MICRO_ROS_RETRY_INTERVAL_MS)
        {
            lastRetryMs_ = now;
            tearDown();
            if (initAgent())
            {
                nodeReady_ = true;
                lastMessageMs_ = millis();
                logger_.info("MicroROS", "Node created, subscribing to " + String(MICRO_ROS_VISION_TOPIC));
            }
        }
        return false;
    }

    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(0));

    if (hasNewCommand_)
    {
        hasNewCommand_ = false;
        if (mode == Mode::AUTO)
        {
            command = lastCommand_;
            return true;
        }
    }

    if (mode == Mode::AUTO && millis() - lastMessageMs_ > MICRO_ROS_WATCHDOG_TIMEOUT_MS)
    {
        if (lastCommand_ != Command::NONE)
        {
            logger_.warn("MicroROS", "Watchdog timeout: no vision command, stopping");
            lastCommand_ = Command::NONE;
            command = Command::NONE;
            return true;
        }
    }
#else
    (void)command;
    (void)mode;
#endif
    return false;
}
