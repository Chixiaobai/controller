#pragma once

#include "ComponentStub.h"
#include "ILogSupportInner.h"
#include "Reference/Reference.h"

#define DECLARE_UNIQUE_LOGSUPPORT_STUB()                                           \
    DECLARE_UNIQUE_COMPONENT_STUB(LogSupport)                                      \
    class CUniqueLogSupport {                                                      \
    public:                                                                        \
        static void updateLogSupportReport(ILogReport *pLogReport);                \
        static unsigned short getMaxLogLevel();                                    \
        static void setMaxLogLevel(unsigned short uiLogLevel);                     \
        static void log(unsigned short uiLogLevel, const char *strLogFormat, ...); \
    };

#define IMPLEMENT_UNIQUE_LOGSUPPORT_STUB()                                                  \
    IMPLEMENT_UNIQUE_COMPONENT_STUB(LogSupport, UNIQUE_LOGSUPPORT_NAME)                     \
    void CUniqueLogSupport::updateLogSupportReport(ILogReport *pLogReport) {                \
        CReference<ILogSupportInner> logSupport = CLogSupport::getResource();               \
        if (logSupport.nil() == false) {                                                    \
            logSupport->setLogReport(pLogReport);                                           \
        }                                                                                   \
    }                                                                                       \
    unsigned short CUniqueLogSupport::getMaxLogLevel() {                                    \
        unsigned short uiLevel = 1;                                                         \
        CReference<ILogSupportInner> logSupport = CLogSupport::getResource();               \
        if (logSupport.nil() == false) {                                                    \
            uiLevel = logSupport->getMaxLogLevel();                                         \
        }                                                                                   \
        return uiLevel;                                                                     \
    }                                                                                       \
    void CUniqueLogSupport::setMaxLogLevel(unsigned short uiLogLevel) {                     \
        CReference<ILogSupportInner> logSupport = CLogSupport::getResource();               \
        if (logSupport.nil() == false) {                                                    \
            logSupport->setMaxLogLevel(uiLogLevel);                                         \
        }                                                                                   \
    }                                                                                       \
    void CUniqueLogSupport::log(unsigned short uiLogLevel, const char *strLogFormat, ...) { \
        CReference<ILogSupportInner> logSupport = CLogSupport::getResource();               \
        if (logSupport.nil() == false) {                                                    \
            va_list ap;                                                                     \
            va_start(ap, strLogFormat);                                                     \
            logSupport->logV(uiLogLevel, strLogFormat, ap);                                 \
            va_end(ap);                                                                     \
        } else {                                                                            \
            va_list ap;                                                                     \
            va_start(ap, strLogFormat);                                                     \
            vprintf(strLogFormat, ap);                                                      \
            va_end(ap);                                                                     \
        }                                                                                   \
    }

#define LOG_ERROR(format, ...) CUniqueLogSupport::log(0, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) CUniqueLogSupport::log(1, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) CUniqueLogSupport::log(2, format, ##__VA_ARGS__)
#define LOG_DEBUG(format, ...) CUniqueLogSupport::log(3, format, ##__VA_ARGS__)
