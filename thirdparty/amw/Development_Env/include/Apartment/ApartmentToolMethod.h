#pragma once

#include "string"
using std::string;

#include "OS/OS_Clock.h"
#include "OS/OS_Event.h"

class CApartmentToolMethod {
public:
    static inline timespec convertTimeoutMsValue(const unsigned int timeout) {
        timespec t;
        OS::Clock::getTime(CLOCK_MONOTONIC, &t);
        __int64 nsec = (__int64)t.tv_nsec + (__int64)timeout * 1000000;
        __int64 sec = (__int64)t.tv_sec;
        if (nsec >= 1000000000) {
            __int64 n = nsec / 1000000000;
            sec += n;
            nsec -= n * 1000000000;
        }
        t.tv_sec = (time_t)sec;
        t.tv_nsec = (long)nsec;
        return t;
    }

    static inline timespec convertTimeoutUsValue(const unsigned int timeout) {
        timespec t;
        OS::Clock::getTime(CLOCK_MONOTONIC, &t);
        __int64 nsec = (__int64)t.tv_nsec + (__int64)timeout * 1000;
        __int64 sec = (__int64)t.tv_sec;
        if (nsec >= 1000000000) {
            __int64 n = nsec / 1000000000;
            sec += n;
            nsec -= n * 1000000000;
        }
        t.tv_sec = (time_t)sec;
        t.tv_nsec = (long)nsec;
        return t;
    }

    static bool waitForEvent(OS::TEvent &e, const unsigned int maxWaitTime) {
        timespec timeout;
        OS::Clock::getTime(CLOCK_MONOTONIC, &timeout);
        __int64 nsec = (__int64)timeout.tv_nsec + (__int64)maxWaitTime * 1000000;
        __int64 sec = (__int64)timeout.tv_sec;
        if (nsec >= 1000000000) {
            __int64 n = nsec / 1000000000;
            sec += n;
            nsec -= n * 1000000000;
        }
        timeout.tv_sec = (time_t)sec;
        timeout.tv_nsec = (long)nsec;
        return e.wait(timeout);
    }
    static string ipFromUrl(const string &strUrl) {
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

    static unsigned int portFromUrl(const string &strUrl) {
        size_t start = strUrl.rfind(':');
        if (start != strUrl.npos) {
            return atoi(strUrl.substr(start + 1).c_str());
        } else {
            return atoi(strUrl.c_str());
        }
    }

    static string suitableApartmentUrl(const string &strServiceCenterUrl, const string &strTargetApartmentUrl) {
        if (ipFromUrl(strTargetApartmentUrl).compare("127.0.0.1") == 0) {
            string strIp = ipFromUrl(strServiceCenterUrl);
            unsigned int uintPort = portFromUrl(strTargetApartmentUrl);
            char _strTargetApartmentUrl[128] = {0};
            sprintf(_strTargetApartmentUrl, "tcp://%s:%u", strIp.c_str(), uintPort);
            return _strTargetApartmentUrl;
        }
        return strTargetApartmentUrl;
    }
};
