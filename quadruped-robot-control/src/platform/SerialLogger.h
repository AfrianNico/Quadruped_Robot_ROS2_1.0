#pragma once

#include <Arduino.h>

#if ENABLE_WIFI_LOGGING
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

#include "domain/ILogger.h"
#include "config/system_config.h"

/**
 * @brief Concrete logger implementation that writes to a HardwareSerial port.
 *
 * All service and driver code MUST log through this class instead of calling
 * Serial.print() directly. This preserves the Clean Architecture rule that
 * only platform code depends on hardware serial directly.
 */
class SerialLogger : public ILogger
{
public:
    /**
     * @param serial Reference to the serial port used for log output.
     * @param level Minimum log level to output (default: Debug = all).
     */
    explicit SerialLogger(HardwareSerial &serial, LogLevel level = LogLevel::Debug)
        : serial_(serial), minLevel_(level)
    {
    }

    /** Enable/disable log output (e.g. silence before XRCE-DDS takes the port). */
    void setEnabled(bool enabled) { enabled_ = enabled; }

#if ENABLE_WIFI_LOGGING
    bool beginWiFi()
    {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
            delay(500);
        if (WiFi.status() != WL_CONNECTED)
            return false;
        wifiUdp_.begin(WIFI_LOG_TARGET_PORT);
        return true;
    }
#endif

    void debug(const String &tag, const String &message) override
    {
#if APP_DEBUG
        if (minLevel_ <= LogLevel::Debug)
            log("DBG", tag, message);
#else
        (void)tag;
        (void)message;
#endif
    }

    void info(const String &tag, const String &message) override
    {
        if (minLevel_ <= LogLevel::Info)
            log("INF", tag, message);
    }

    void warn(const String &tag, const String &message) override
    {
        if (minLevel_ <= LogLevel::Warn)
            log("WRN", tag, message);
    }

    void error(const String &tag, const String &message) override
    {
        if (minLevel_ <= LogLevel::Error)
            log("ERR", tag, message);
    }

private:
    HardwareSerial &serial_;
    LogLevel minLevel_;
    bool enabled_ = true;
#if ENABLE_WIFI_LOGGING
    WiFiUDP wifiUdp_;
#endif

    void log(const char *level, const String &tag, const String &message)
    {
        if (!enabled_)
            return;

        serial_.print('[');
        serial_.print(level);
        serial_.print("][");
        serial_.print(tag);
        serial_.print("] ");
        serial_.println(message);

#if ENABLE_WIFI_LOGGING
        if (WiFi.status() == WL_CONNECTED)
        {
            String line = String("[") + level + "][" + tag + "] " + message + "\n";
            wifiUdp_.beginPacket(WIFI_LOG_TARGET_IP, WIFI_LOG_TARGET_PORT);
            wifiUdp_.print(line);
            wifiUdp_.endPacket();
        }
#endif
    }
};