#pragma once

#include "stdafx.h"

#include <string>
#include <map>
#if defined(_WIN32) || defined(_WINDOWS)
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "OS/OS_Sync.h"

#include "../Common.h"

namespace amw {

/////////////////////////////////////////////////////////////////////////////////////////
class CProcessRuntime {
public:
    enum class STATUS {
        UNKNOWN = 1,
        INIT,
        RUNNING,
        STOPPED,
    };

    struct ServiceInfo {};
    struct ProcessData {
        bool isPublisher = false;
        id_type m_pid = 0;
        id_type m_nodePid = 0;
        std::string m_hostUrl = "";
        STATUS m_status;
        std::string m_stdoutLog;
        std::string m_stderrLog;
        std::map<std::string /*topicUrl*/, bool> m_topicUrls;
    };

    void addService(const std::string& serviceName) { m_serviceMap[serviceName] = ServiceInfo{}; }
    void removeService(const std::string& serviceName) { m_serviceMap.erase(serviceName); }
    const std::map<std::string, ServiceInfo>& getServiceMap() const { return m_serviceMap; }
    void setPid(id_type pid) { m_processData.m_pid = pid; }
    id_type getPid() const { return m_processData.m_pid; }
    void setNodePid(id_type nodePid) { m_processData.m_nodePid = nodePid; }
    id_type getNodePid() const { return m_processData.m_nodePid; }
    void setHostUrl(const std::string& hostUrl) { m_processData.m_hostUrl = hostUrl; }
    const std::string& getHostUrl() const { return m_processData.m_hostUrl; }
    STATUS getStatus() const { return m_processData.m_status; }
    bool isPublisher(const std::string& topicUrl) const { return m_processData.m_topicUrls.at(topicUrl); }
    void setStatus(STATUS status) { m_processData.m_status = status; }
    void addTopicUrl(const std::string& topicUrl, bool isPublisher) {
        m_processData.m_topicUrls.insert(std::make_pair(topicUrl, isPublisher));
    }
    void removeTopicUrl(const std::string& topicUrl) { m_processData.m_topicUrls.erase(topicUrl); }
    const std::map<std::string, bool>& getTopicUrls() const { return m_processData.m_topicUrls; }
    const std::string& getStdoutLog() const { return m_processData.m_stdoutLog; };
    const std::string& getStderrLog() const { return m_processData.m_stderrLog; };
    void setStdoutLog(const std::string& log) { m_processData.m_stdoutLog.append(log); };
    void setStderrLog(const std::string& log) { m_processData.m_stderrLog.append(log); };

    static std::string getProcessName() {
#if defined(_WIN32) || defined(_WINDOWS)
        string processName;
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        processName = buffer;
        return processName;
#else
        char buffer[1024];
        ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
        if (len != -1) {
            buffer[len] = '\0';
            return std::string(basename(buffer));
        }
        return "";
#endif
}
private:
    ProcessData m_processData;
    std::map<std::string /*serviceName*/, ServiceInfo> m_serviceMap;
};
using RunTimeInfoMap = std::map<std::string, CProcessRuntime>;

}  // namespace amw