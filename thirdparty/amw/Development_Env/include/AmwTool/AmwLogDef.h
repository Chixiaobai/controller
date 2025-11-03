#ifndef _AMW_LOG_DEF_H_
#define _AMW_LOG_DEF_H_

namespace amw {

const static uint32_t MAX_LOG_LENGTH = 1024;
enum LogLevel { AMW_LOG_TEST, AMW_LOG_DEBUG, AMW_LOG_INFO, AMW_LOG_WARNING, AMW_LOG_ERROR };

struct LogItem {
    LogLevel m_intLogLevel = AMW_LOG_INFO;
    uint64_t m_ullMilliseconds;
    time_t m_ullSeconds;
    char m_strLogBuffer[MAX_LOG_LENGTH];
};

}  // namespace amw

#endif  // _AMW_LOG_DEF_H_