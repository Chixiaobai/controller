#pragma once

#include "../AmwToolDef.h"

#define DEFAULT_SERVER_IP "127.0.0.1"
#define DEFAULT_SERVER_PORT 10081

const int MAX_PACKET_SIZE = 64 * 1024;
const int MAX_WAIT_TIME = 1000;  // milliseconds

namespace util {
enum class LogTarget : uint8_t {
    NODE = 1,
    HOST,
    SERVICECENTER,
};

AMWTOOL_API void ProtocolToString(LogTarget logTarget, const std::string &strProcessName, const std::string &log,
                                  char **data, int *datalen);

}  // namespace util
