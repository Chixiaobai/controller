#pragma once

#include <string>
#include <map>
#include <algorithm>

#define DERMINAL "derminal"

namespace amw {
class CProcessConfig {
public:
    CProcessConfig() = default;
    ~CProcessConfig() = default;
    CProcessConfig(const CProcessConfig&) = delete;
    CProcessConfig& operator=(const CProcessConfig&) = delete;
    CProcessConfig(CProcessConfig&&) = default;
    CProcessConfig& operator=(CProcessConfig&&) = default;
    std::string getGroup() const { return m_group; }
    void setGroup(const std::string& newGroup) { m_group = newGroup; }
    void setHostUrl(const std::string& hostUrl) { m_host = hostUrl; }
    std::string getHostUrl() const { return m_host; }
    std::string getName() const { return m_name; }
    void setName(const std::string& name) { m_name = name; }
    std::string getWorkingDirectory() const { return m_workingDirectory; }
    void setWorkingDirectory(const std::string& newDirectory) { m_workingDirectory = newDirectory; }
    std::string getEnvironment() const { return m_environment; }
    void setEnvironment(const std::string& newEnvironment) { m_environment = newEnvironment; }
    std::string getCommand() const { return m_command; }
    void setCommand(const std::string& newCommand) { m_command = newCommand; }
    std::string getStartBefore() const { return m_startBefore; }
    void setStartBefore(const std::string& start) { m_startBefore = start; }
    bool isConfig() const { return m_isConfig; }
    void setConfig(bool isConfig) { this->m_isConfig = isConfig; }
    int getStartupTime() { return m_startupTime; }
    void setStartupTime(const int startupTime) { m_startupTime = startupTime; }
    void setRunAsRoot(const std::string& rootValue) {
        if (rootValue.empty() || !std::all_of(rootValue.begin(), rootValue.end(), ::isdigit)) {
            return;
        }
        int isRoot = std::stoi(rootValue);
        m_runasroot = isRoot == 1;
    }
    bool isRunAsRoot() const { return m_runasroot; }

private:
    std::string m_group = DERMINAL;
    std::string m_name = "";
    std::string m_workingDirectory = "";
    std::string m_command = "";
    std::string m_environment = "";
    std::string m_startBefore = "";
    std::string m_host = "other";
    int m_startupTime = 0;
    bool m_isConfig = false;
    bool m_runasroot = false;
};

using ProcessConfigMap = std::map<const std::string /*NodeName*/, amw::CProcessConfig>;
}  // namespace amw
