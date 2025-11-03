#pragma once

#define UNIQUE_LOGSUPPORT_NAME "LogSupport"

#include "ILogReport.h"

/**
 * @interface ILogSupportInner
 * @brief Core interface for logging functionality and configuration.
 *
 * This interface provides methods to configure log settings, set log reporting targets,
 * and output log messages with various severity levels.
 */
interface ILogSupportInner : public IBase {
    /**
     * @brief Sets the log report handler to receive log messages.
     *
     * @param pLogReport Pointer to the ILogReport implementation that will handle logs.
     */
    virtual void setLogReport(ILogReport * pLogReport) = 0;

    /**
     * @brief Retrieves the current maximum log level that will be processed.
     *
     * @return The current maximum log level.
     */
    virtual unsigned short getMaxLogLevel() = 0;

    /**
     * @brief Sets the maximum log level that will be processed.
     *
     * Log messages with a severity higher than this level will be ignored.
     *
     * @param uiLogLevel New maximum log level to set.
     */
    virtual void setMaxLogLevel(unsigned short uiLogLevel) = 0;

    /**
     * @brief Logs a formatted message with the specified log level.
     *
     * @param uiLogLevel Severity level of the log message.
     * @param strLogFormat Format string for the log message.
     * @param ... Arguments for the format string.
     */
    virtual void log(unsigned short uiLogLevel, const char *strLogFormat, ...) = 0;

    /**
     * @brief Logs a formatted message using a variable argument list.
     *
     * @param uiLogLevel Severity level of the log message.
     * @param strLogFormat Format string for the log message.
     * @param ap Variable argument list for the format string.
     */
    virtual void logV(unsigned short uiLogLevel, const char *strLogFormat, va_list ap) = 0;

    /**
     * @brief Enables or disables timestamping for log messages.
     *
     * @param bEnable true to enable timestamps; false to disable.
     */
    virtual void setTimeStampEnabled(bool bEnable) = 0;
};

ENABLE_RECOGNIZE_MACRO(ILogSupportInner)
