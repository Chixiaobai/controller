#ifndef __AMW_LOG_HELPER_H__
#define __AMW_LOG_HELPER_H__

#include "AmwLogApi.h"

#define LOG(level, format, ...) amw::push_log(level, format, ##__VA_ARGS__);
#define ILOG(format, ...) amw::push_log(amw::AMW_LOG_INFO, format, ##__VA_ARGS__);
#define ELOG(format, ...) amw::push_log(amw::AMW_LOG_ERROR, format, ##__VA_ARGS__);
#define WLOG(format, ...) amw::push_log(amw::AMW_LOG_WARNING, format, ##__VA_ARGS__);
#define DLOG(format, ...) amw::push_log(amw::AMW_LOG_DEBUG, format, ##__VA_ARGS__);
#define TLOG(format, ...) amw::push_log(amw::AMW_LOG_TEST, format, ##__VA_ARGS__);

using namespace amw;

#endif