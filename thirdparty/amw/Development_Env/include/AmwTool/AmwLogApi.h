#ifndef __AMW_LOG_API_H__
#define __AMW_LOG_API_H__
#include "stdafx.h"
#include "AmwToolDef.h"
#include "AmwLogDef.h"

/**
 * @file AmwLogApi.h
 * @brief Public API for the AMW logging system.
 *
 * This header defines functions to initialize, configure, and use the logging system.
 */

namespace amw {

/**
 * @brief Initializes the logging system.
 *
 * This function must be called before using any other logging functions.
 * It sets up internal resources such as log queues and worker threads.
 */
AMWTOOL_API void install_log_task();

/**
 * @brief Sets the minimum log level for output.
 *
 * Log messages with a severity level lower than this will be filtered out.
 *
 * @param level The minimum log level to display. See LogLevel for available levels.
 */
AMWTOOL_API void set_log_level(const LogLevel level);

/**
 * @brief Gets the current minimum log level for output.
 *
 * @return The current log level setting.
 */
AMWTOOL_API LogLevel get_log_level();

/**
 * @brief Pushes a formatted log message into the logging queue.
 *
 * This function accepts printf-style formatting.
 *
 * @param level Severity level of the log message.
 * @param strFormat Format string, followed by variable arguments.
 * @return true if the message was successfully added to the queue; false otherwise.
 */
AMWTOOL_API bool push_log(const LogLevel level, const char *strFormat, ...);

/**
 * @brief Shuts down and cleans up the logging system.
 *
 * This function should be called when logging is no longer needed,
 * typically at application exit.
 */
AMWTOOL_API void uninstall_log_task();

}  // namespace amw

#endif