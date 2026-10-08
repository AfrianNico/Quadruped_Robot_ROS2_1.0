#pragma once

#include <Arduino.h>

/**
 * @brief Dynamixel UART and direction control pins.
 */
constexpr uint8_t DXL_DIR_PIN = 4;
constexpr int DXL_UART_RX_PIN = 26;
constexpr int DXL_UART_TX_PIN = 25;

/**
 * @brief PZEM-017 Modbus UART pins (uses Serial1).
 * NOTE: Serial1 is now shared with the Dynamixel bus on this robot,
 *       so PZEM is disabled via ENABLE_PZEM=0.
 */
constexpr int PZEM_RX_PIN = 27;
constexpr int PZEM_TX_PIN = 27;

/**
 * @brief IMU reserved UART pins (future use).
 */
constexpr int IMU_RX_PIN = 33;
constexpr int IMU_TX_PIN = 32;

/**
 * @brief SPI slave pins for Orange Pi / vision input.
 */
constexpr int GPIO_MOSI = 23;
constexpr int GPIO_MISO = 19;
constexpr int GPIO_SCLK = 18;
constexpr int GPIO_CS = 5;

/**
 * @brief SPI transaction buffer size (bytes).
 */
constexpr size_t SPI_BUFFER_SIZE = 32;