#pragma once

#include <cstdint>

class String;

/**
 * @brief Logging levels for the ILogger interface.
 */
enum class LogLevel : uint8_t
{
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
};

/**
 * @brief Interface for all logging output.
 *
 * All service/driver code MUST log through this interface rather than
 * calling Serial.print() directly.
 */
class ILogger
{
public:
    virtual ~ILogger() = default;

    virtual void debug(const String &tag, const String &message) = 0;
    virtual void info(const String &tag, const String &message) = 0;
    virtual void warn(const String &tag, const String &message) = 0;
    virtual void error(const String &tag, const String &message) = 0;
};
