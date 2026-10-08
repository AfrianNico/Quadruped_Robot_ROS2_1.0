#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config/pin_config.h"
#include "config/system_config.h"
#include "config/battery_config.h"
#include "domain/Command.h"
#include "domain/Mode.h"
#include "domain/MotionVector.h"
#include "domain/BatteryData.h"
#include "platform/SerialLogger.h"
#include "platform/BatterySharedState.h"
#include "platform/ModeSharedState.h"
#include "service/GaitService.h"
#include "service/BatteryService.h"
#include "driver/DynamixelBusDriver.h"
#include "driver/CommandInputSerial.h"
#include "driver/CommandInputSpi.h"
#if ENABLE_MICROROS
#include "driver/CommandInputMicroRos.h"
#endif
#include "driver/Pzem017Driver.h"
#include "driver/NvsStorageDriver.h"
#if ENABLE_PS3_INPUT
#include "driver/CommandInputPs3.h"
#endif

static SerialLogger logger(Serial, LogLevel::Debug);
static DynamixelBusDriver dynamixel(Serial1, DXL_DIR_PIN);
static CommandInputSerial<HardwareSerial> serialInput(Serial);
#if ENABLE_PZEM
static Pzem017Driver pzemSensor(Serial1);
#endif
static NvsStorageDriver nvsStorage;
static BatterySharedState batterySharedState;
static Mode activeMode = DEFAULT_MODE_AUTO ? Mode::AUTO : Mode::MANUAL;
static Command activeCommand = Command::NONE;
static ModeSharedState modeSharedState(activeMode);
#if ENABLE_PZEM
static BatteryService batteryService(pzemSensor, nvsStorage, logger);
#endif
static CommandInputSpi spiInput(logger, batterySharedState, modeSharedState);
#if ENABLE_MICROROS
static CommandInputMicroRos microRosInput(logger, modeSharedState);
#endif
static GaitService gait(dynamixel, logger);
static MotionVector activeMotion{};
static constexpr unsigned long BATTERY_INTERVAL_MS = 500; // 2 Hz

#if ENABLE_PS3_INPUT
static CommandInputPs3 ps3Input(&logger);
#endif

/**
 * @brief FreeRTOS task that owns the PZEM Modbus communication.
 */
#if ENABLE_PZEM
void batteryTaskMain(void *pvParameters)
{
  auto *service = static_cast<BatteryService *>(pvParameters);
  BatteryState state;

  while (true)
  {
    if (service->update(state))
    {
      batterySharedState.set(state);
    }
    vTaskDelay(pdMS_TO_TICKS(BATTERY_INTERVAL_MS));
  }
}
#endif

/**
 * @brief Maps RobotState to a behaviour integer code for telemetry.
 */
static uint8_t robotStateToBehaviorCode(RobotState state)
{
  switch (state)
  {
  case RobotState::IDLE:
    return 0;
  case RobotState::STANDING_UP:
    return 0;
  case RobotState::SITTING_DOWN:
    return 0;
  case RobotState::SITTING:
    return 0;
  case RobotState::WALKING:
    return 3;
  default:
    return 0;
  }
}

/**
 * @brief Maps activeCommand to a behaviour integer code for telemetry.
 * Used when the robot is walking/running.
 */
static uint8_t commandToBehaviorCode(Command cmd)
{
  switch (cmd)
  {
  case Command::NONE:
    return 0;
  case Command::FORWARD:
    return 1;
  case Command::BACKWARD:
    return 2;
  case Command::TURN_LEFT:
    return 3;
  case Command::TURN_RIGHT:
    return 4;
  default:
    return 0;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  logger.info("Main", "Initializing...");

  dynamixel.begin(DXL_UART_TX_PIN, DXL_UART_RX_PIN);
  logger.info("Main", "Dynamixel bus initialised");

  nvsStorage.begin();
  batterySharedState.begin();
#if ENABLE_PZEM
  pzemSensor.begin();
  batteryService.begin();
  xTaskCreatePinnedToCore(
      batteryTaskMain,
      "battery",
      4096,
      &batteryService,
      1,
      nullptr,
      1);
  logger.info("Main", "Battery monitoring initialised");
#endif

#if !(ENABLE_MICROROS && MICRO_ROS_USE_SERIAL)
  serialInput.begin();
#endif
  spiInput.begin();
  logger.info("Main", "SPI input initialised");
#if ENABLE_MICROROS
#if MICRO_ROS_USE_SERIAL
  // XRCE-DDS akan menguasai UART0 -- log teks harus dibungkam agar
  // stream biner tidak tercampur.
  logger.setEnabled(false);
#endif
  microRosInput.begin();
  logger.info("Main", "micro-ROS input initialised");
#endif
#if ENABLE_PS3_INPUT
  logger.info("Main", "PS3 input initializing...");
  ps3Input.begin();
  logger.info("Main", "PS3 input enabled");
#endif

  gait.initialize();
  logger.info("Main", "Robot is ready");

#if ENABLE_WIFI_LOGGING
  if (logger.beginWiFi())
  {
    logger.info("Main", "WiFi log forwarding to " +
                            String(WIFI_LOG_TARGET_IP) + ":" + String(WIFI_LOG_TARGET_PORT));
  }
  else
  {
    logger.warn("Main", "WiFi log connect failed, continuing without");
  }
#endif
}

void loop()
{
  unsigned long loopStart = micros();
  static unsigned long lastLoopWarnMs = 0;

  Command newCmd = activeCommand;
  Mode newMode = activeMode;
  MotionVector newMv = activeMotion;

#if !(ENABLE_MICROROS && MICRO_ROS_USE_SERIAL)
  if (serialInput.update(newCmd, newMode))
  {
    activeCommand = newCmd;
    activeMode = newMode;
  }
#endif

  if (spiInput.update(newCmd, newMode))
  {
    activeCommand = newCmd;
    activeMode = newMode;
  }

#if ENABLE_MICROROS
  if (microRosInput.update(newCmd, newMode))
  {
    activeCommand = newCmd;
    activeMode = newMode;
  }
#endif

  activeMode = modeSharedState.get();
  {
    static Mode prevMode = activeMode;
    if (prevMode == Mode::AUTO && activeMode == Mode::MANUAL)
    {
      activeCommand = Command::NONE;
    }
    prevMode = activeMode;
  }

#if ENABLE_PS3_INPUT
  if (activeMode == Mode::MANUAL)
  {
    if (ps3Input.updateMotion(newMv))
    {
      activeMotion = newMv;
    }
    else if (ps3Input.update(newCmd, newMode))
    {
      activeCommand = newCmd;
      activeMode = newMode;
    }
  }
  else
  {
    activeMotion = MotionVector{};
  }
#endif

  const RobotState state = gait.getState();
  uint8_t behaviorCode = robotStateToBehaviorCode(state);
  if (state == RobotState::WALKING)
  {
    behaviorCode = commandToBehaviorCode(activeCommand);
  }
  spiInput.setBehaviorCode(behaviorCode);

  if (state == RobotState::IDLE || state == RobotState::WALKING)
  {
    if (activeMotion.forward != 0 || activeMotion.turn != 0 ||
        activeMotion.buttonCommand != Command::NONE)
    {
      gait.run(activeMotion);
    }
    else
    {
      gait.run(activeCommand);
    }
  }
  else if (state == RobotState::SITTING)
  {
    gait.run(activeMotion);
    delay(10);
  }
  else
  {
    MotionVector mv{};
    mv.buttonCommand = (state == RobotState::STANDING_UP) ? Command::STAND_UP : Command::STAND_DOWN;
    gait.run(mv);
    delay(10);
  }

  unsigned long loopUs = micros() - loopStart;
  if (loopUs > 50000)
  {
    unsigned long now = millis();
    if (now - lastLoopWarnMs > 1000)
    {
      logger.warn("Main", "Loop took " + String(loopUs) + "us");
      lastLoopWarnMs = now;
    }
  }
}