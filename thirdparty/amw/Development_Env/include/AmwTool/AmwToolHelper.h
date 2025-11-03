#ifndef __AMW_TOOL_HELPER_H__
#define __AMW_TOOL_HELPER_H__

#include "AmwLogHelper.h"

#define CALCULATE_COST_TIME(str) amwhelper::CCostTime cost(str)

namespace amwhelper {
class CCostTime {
public:
    CCostTime(const char *str) {
        DLOG("%s cost time:", str);
        OS::Clock::getTime(CLOCK_MONOTONIC, &m_startTime);
    }
    ~CCostTime() {
        OS::Clock::getTime(CLOCK_MONOTONIC, &m_endTime);
        OS::Clock::getClockSpan(&m_startTime, &m_endTime, &m_costTime);
        int intCostTime = RESOLVE_SPAN_AS_USEC(&m_costTime);
        DLOG("%dus", intCostTime);
    }

private:
    timespec m_startTime;
    timespec m_endTime;
    timespec m_costTime;
};

static inline string ipFromUrl(const string &strUrl) {
    size_t start = strUrl.find("//");
    if (start != strUrl.npos) {
        start += 2;
        size_t end = strUrl.find(':', start);
        if (end != strUrl.npos) {
            return strUrl.substr(start, end - start);
        } else {
            return strUrl.substr(start);
        }
    } else {
        size_t end = strUrl.find(':');
        if (end != strUrl.npos) {
            return strUrl.substr(0, end);
        } else {
            return strUrl.substr(0);
        }
    }
}

static inline unsigned int portFromUrl(const string &strUrl) {
    size_t start = strUrl.rfind(':');
    if (start != strUrl.npos) {
        return atoi(strUrl.substr(start + 1).c_str());
    } else {
        return atoi(strUrl.c_str());
    }
}

}  // namespace amwhelper

#endif