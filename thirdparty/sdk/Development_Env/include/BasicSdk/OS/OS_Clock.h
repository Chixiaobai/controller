#ifndef __BASICSDK_OS_CLOCK_H__
#define __BASICSDK_OS_CLOCK_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Get the current time for a specified clock
 *
 * @param __clock_id Clock identifier
 * @param tp Pointer to a timespec structure to store the time
 * @param mode Real-time mode (default is USE_DEFAULT)
 * @return int 0 on success, or -1 on failure
 */
BASICSDK_API int os_clock_get_time(clockid_t __clock_id, timespec *tp, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Sleep for a specified duration in nanoseconds
 *
 * @param req Pointer to a timespec structure specifying the sleep duration
 * @param mode Real-time mode (default is USE_DEFAULT)
 */
BASICSDK_API void os_clock_nano_sleep(const timespec *req, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Calculate the time span between two timespec structures
 *
 * @param startTime Pointer to the start timespec structure
 * @param endTime Pointer to the end timespec structure
 * @param timeSpan Pointer to a timespec structure to store the time span
 */
BASICSDK_API void os_clock_get_clock_span(timespec *const startTime, timespec *const endTime, timespec *timeSpan);

/**
 * @brief Adjust the clock time
 *
 * @param tp Pointer to a timespec structure containing the new time
 */
BASICSDK_API void os_clock_adjust_clock_time(timespec *tp);

/**
 * @brief Convert real-time to monotonic time
 *
 * @param realTime Pointer to a timespec structure containing the real-time
 * @param monotonicTime Pointer to a timespec structure to store the monotonic time
 * @param mode Real-time mode (default is USE_DEFAULT)
 */
BASICSDK_API void os_clock_realtime_to_monotonic(timespec *realTime, timespec *monotonicTime,
                                                 RT_MODE mode = USE_DEFAULT);

/**
 * @brief Convert monotonic to real-time
 *
 * @param monotonicTime Pointer to a timespec structure containing the monotonic time
 * @param realTime Pointer to a timespec structure to store the real-time
 * @param mode Real-time mode (default is USE_DEFAULT)
 */
BASICSDK_API void os_clock_monotonic_to_realtime(timespec *monotonicTime, timespec *realTime,
                                                 RT_MODE mode = USE_DEFAULT);

/**
 * @brief Namespace for clock-related functions and classes
 */
namespace OS {
class Clock {
public:
    /**
     * @brief Get the current time for a specified clock
     *
     * @param __clock_id Clock identifier
     * @param tp Pointer to a timespec structure to store the time
     * @param mode Real-time mode (default is USE_DEFAULT)
     * @return int 0 on success, or -1 on failure
     */
    static int getTime(clockid_t __clock_id, timespec *tp, RT_MODE mode = USE_DEFAULT) {
        return os_clock_get_time(__clock_id, tp, mode);
    }

    /**
     * @brief Sleep for a specified duration in nanoseconds
     *
     * @param req Pointer to a timespec structure specifying the sleep duration
     * @param mode Real-time mode (default is USE_DEFAULT)
     */
    static void nanoSleep(const timespec *req, RT_MODE mode = USE_DEFAULT) { return os_clock_nano_sleep(req, mode); }

    /**
     * @brief Calculate the time span between two timespec structures
     *
     * @param startTime Pointer to the start timespec structure
     * @param endTime Pointer to the end timespec structure
     * @param timeSpan Pointer to a timespec structure to store the time span
     */
    static void getClockSpan(timespec *const startTime, timespec *const endTime, timespec *timeSpan) {
        os_clock_get_clock_span(startTime, endTime, timeSpan);
    }

    /**
     * @brief Adjust the clock time
     *
     * @param tp Pointer to a timespec structure containing the new time
     */
    static void adjustClockTime(timespec *tp) { os_clock_adjust_clock_time(tp); }

    /**
     * @brief Convert real-time to monotonic time
     *
     * @param realTime Pointer to a timespec structure containing the real-time
     * @param monotonicTime Pointer to a timespec structure to store the monotonic time
     * @param mode Real-time mode (default is USE_DEFAULT)
     */
    static void realtimeToMonotonic(timespec *realTime, timespec *monotonicTime, RT_MODE mode = USE_DEFAULT) {
        os_clock_realtime_to_monotonic(realTime, monotonicTime, mode);
    }

    /**
     * @brief Convert monotonic to real-time
     *
     * @param monotonicTime Pointer to a timespec structure containing the monotonic time
     * @param realTime Pointer to a timespec structure to store the real-time
     * @param mode Real-time mode (default is USE_DEFAULT)
     */
    static void monotonicToRealtime(timespec *monotonicTime, timespec *realTime, RT_MODE mode = USE_DEFAULT) {
        os_clock_monotonic_to_realtime(monotonicTime, realTime, mode);
    }

    /**
     * @brief Convert a time string to a time_t value
     *
     * The expected format of the time string is "YYYY/MM/DD HH:MM:SS".
     *
     * @param strTimeFormat Time string to convert
     * @return time_t Time value in seconds since the epoch
     */
    static time_t textToTime(const char *strTimeFormat) {
        struct tm localTime;
#ifdef _WIN32
        sscanf_s(strTimeFormat, "%4d/%2d/%2d %2d:%2d:%2d", &localTime.tm_year, &localTime.tm_mon, &localTime.tm_mday,
                 &localTime.tm_hour, &localTime.tm_min, &localTime.tm_sec);
#else
        sscanf(strTimeFormat, "%4d/%2d/%2d %2d:%2d:%2d", &localTime.tm_year, &localTime.tm_mon, &localTime.tm_mday,
               &localTime.tm_hour, &localTime.tm_min, &localTime.tm_sec);
#endif
        localTime.tm_year -= 1900;
        localTime.tm_mon -= 1;
        localTime.tm_isdst = -1;
        return mktime(&localTime);
    }

    /**
     * @brief Convert a time_t value to a time string
     *
     * The output format of the time string is "YYYY/MM/DD HH:MM:SS".
     *
     * @param t Time value in seconds since the epoch
     * @return string Time string
     */
    static string timeToText(const time_t &t) {
        char timeBuf[64] = {0};
        struct tm localTime;
#ifdef _WIN32
        localtime_s(&localTime, &t);
#else
        localtime_r(&t, &localTime);
#endif
        strftime(timeBuf, sizeof(timeBuf), "%Y/%m/%d %H:%M:%S", &localTime);
        return timeBuf;
    }
};
}  // namespace OS

/**
 * @brief Resolve the time span as seconds
 *
 * @param timeSpan Pointer to a timespec structure
 * @return int Time span in seconds
 */
#define RESOLVE_SPAN_AS_SEC(timeSpan) ((int)((timeSpan)->tv_sec))

/**
 * @brief Resolve the time span as milliseconds
 *
 * @param timeSpan Pointer to a timespec structure
 * @return int Time span in milliseconds
 */
#define RESOLVE_SPAN_AS_MSEC(timeSpan) \
    ((int)(((int)(timeSpan)->tv_sec) * MSEC_PER_SEC + (timeSpan)->tv_nsec / (NSEC_PER_SEC / MSEC_PER_SEC)))

/**
 * @brief Resolve the time span as microseconds
 *
 * @param timeSpan Pointer to a timespec structure
 * @return int Time span in microseconds
 */
#define RESOLVE_SPAN_AS_USEC(timeSpan) \
    ((int)(((int)(timeSpan)->tv_sec) * USEC_PER_SEC + (timeSpan)->tv_nsec / (NSEC_PER_SEC / USEC_PER_SEC)))

/**
 * @brief Resolve the time span as nanoseconds
 *
 * @param timeSpan Pointer to a timespec structure
 * @return int Time span in nanoseconds
 */
#define RESOLVE_SPAN_AS_NSEC(timeSpan) \
    ((int)(((int)(timeSpan)->tv_sec) * NSEC_PER_SEC + (timeSpan)->tv_nsec / (NSEC_PER_SEC / NSEC_PER_SEC)))

#endif