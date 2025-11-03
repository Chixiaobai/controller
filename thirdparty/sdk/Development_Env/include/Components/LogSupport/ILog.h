#pragma once

#include "Base/IBase.h"

/**
 * @interface ILog
 * @brief Interface for log message resolution.
 *
 * This interface provides methods to retrieve log messages along with their severity levels.
 */
interface ILog : public IBase {
    /**
     * @brief Retrieves a resolved log message and its associated log level.
     *
     * @param uiLogLevel Reference to store the log level (severity).
     * @param strLogMessage Buffer to receive the log message string.
     * @param ulLogSize On input, specifies buffer size; on output, receives actual length.
     * @return true if the log message was successfully retrieved; false otherwise.
     */
    virtual bool resolve(unsigned short &uiLogLevel, char *strLogMessage, unsigned long &ulLogSize) = 0;
};

ENABLE_RECOGNIZE_MACRO(ILog)
