#pragma once

#include <Arduino.h>

/**
 * @brief Compile-time log level filter.
 *
 * When APP_DEBUG is true:  all levels (Debug, Info, Warn, Error) are compiled & printed.
 * When APP_DEBUG is false: Debug-level calls are stripped at compile time,
 *                          only Info/Warn/Error remain.  Saves flash/cpu on production builds.
 */
#ifndef APP_DEBUG
#define APP_DEBUG 0 // 1 = debug on, 0 = debug off
#endif

/**
 * @brief Dynamixel bus baud rate.
 */
constexpr uint32_t DXL_BAUDRATE = 1000000;

/**
 * @brief USB serial (logging / command) baud rate.
 */
constexpr uint32_t CMD_BAUDRATE = 115200;

/**
 * @brief Bluetooth Serial device name for pairing.
 */
constexpr const char *BT_DEVICE_NAME = "ESP32_Robodog";

/**
 * @brief Optional PS3 controller MAC address placeholder.
 * Set to your controller's MAC address. Disabled by default.
 */
#define ENABLE_PS3_INPUT 1
constexpr const char *PS3_MAC_ADDRESS = "02:00:00:00:00:00";

/**
 * @brief PS3 disconnect detection timeout in milliseconds.
 * If no onNotify callback fires within this window, the controller
 * is considered disconnected and the robot is stopped.
 */
constexpr unsigned long PS3_DISCONNECT_TIMEOUT_MS = 2000;

/**
 * @brief PS3 joystick deadzone threshold.
 * Analog stick values below this threshold are treated as zero.
 */
constexpr int PS3_DEADZONE = 20;

/**
 * @brief Hysteresis margin for PS3 analog stick re-activation.
 *
 * Once the stick returns to zero (inside deadzone), it must exceed
 * PS3_DEADZONE + PS3_HYSTERESIS_MARGIN before a new motion command
 * is generated.  Prevents flicker/jitter at the deadzone boundary.
 */
constexpr int PS3_HYSTERESIS_MARGIN = 5;

/**
 * @brief Maximum absolute value for MotionVector::forward after saturation.
 *
 * PS3 analog stick range is ±127.  The gait service maps this linearly
 * to ±MAX_HIP_SWING hip positions.
 */
constexpr int16_t FORWARD_SATURATION_MAX = 127;

/**
 * @brief Maximum absolute value for MotionVector::turn after saturation.
 *
 * Same PS3 analog ±127 range.  The gait service maps this linearly
 * to ±MAX_TURN_OFFSET hip differential.
 */
constexpr int16_t TURN_SATURATION_MAX = 127;

/**
 * @brief Minimum change in forward or turn (raw units) to trigger
 * a MotionVector update.  Prevents micro-jitter on noisy analog sticks.
 */
constexpr int16_t MOTION_HYSTERESIS = 5;

/**
 * @brief Enables Dynamixel servo position read-back for adaptive phase timing.
 *
 * When true, the gait service reads the "Moving" status bit from the two
 * active servos between phases and advances early if both have settled.
 * When false, a static per-phase delay table is used instead.
 */
#define ENABLE_SERVO_READBACK 0

/**
 * @brief Default operational mode on startup.
 * AUTO = SPI/vision drives movement; MANUAL = serial/BT/PS3 drives movement.
 */
constexpr bool DEFAULT_MODE_AUTO = false;

/**
 * @brief Knee position offset applied to all knee goal writes.
 */
constexpr int32_t KNEE_POS_OFFSET = 2048;

/**
 * @brief Enable WiFi log forwarding inside SerialLogger.
 * When true, SerialLogger::log() also sends each message over UDP.
 */
#define ENABLE_WIFI_LOGGING 0 // 0 = off, 1 = on

#if ENABLE_WIFI_LOGGING
constexpr const char *WIFI_SSID = "QDRPD";
constexpr const char *WIFI_PASSWORD = "qwertyuiop";
constexpr const char *WIFI_LOG_TARGET_IP = "192.168.0.200";
constexpr uint16_t WIFI_LOG_TARGET_PORT = 9999;
#endif

/**
 * @brief Set to 0 to disable the PZEM-017 battery sensor task.
 * Required when Serial1 pins are reused (e.g. Dynamixel on GPIO 25/26).
 */
#ifndef ENABLE_PZEM
#define ENABLE_PZEM 0
#endif

// ------------------------------------------------------------------
// micro-ROS
// ------------------------------------------------------------------

/**
 * @brief Enable micro-ROS subscription to the vision node topic.
 * When 1, the ESP32 connects to a micro-ROS agent over WiFi UDP and
 * subscribes to MICRO_ROS_VISION_TOPIC, mapping string commands to
 * Command values (same semantics as AUTO mode vision input).
 */
#define ENABLE_MICROROS 1 // 1 = on, 0 = off

#if ENABLE_MICROROS
// 1 = transport serial UART (UART0 pins via jumper ke Orange Pi), 0 = WiFi UDP
#ifndef MICRO_ROS_USE_SERIAL
#define MICRO_ROS_USE_SERIAL 1
#endif

constexpr const char *MICRO_ROS_WIFI_SSID = "QDRPD";
constexpr const char *MICRO_ROS_WIFI_PASSWORD = "qwertyuiop";
constexpr const char *MICRO_ROS_AGENT_IP = "192.168.0.200";
constexpr uint16_t MICRO_ROS_AGENT_PORT = 8888;
constexpr const char *MICRO_ROS_VISION_TOPIC = "/quadruped/cmd_vision";
constexpr const char *MICRO_ROS_NODE_NAME = "quadruped_esp32";

/**
 * @brief If no vision command is received within this window, the robot
 * is stopped (Command::NONE). Mirrors the SPI watchdog behaviour.
 */
constexpr unsigned long MICRO_ROS_WATCHDOG_TIMEOUT_MS = 5000;

/**
 * @brief Interval between micro-ROS agent reconnect attempts.
 */
constexpr unsigned long MICRO_ROS_RETRY_INTERVAL_MS = 2000;
#endif