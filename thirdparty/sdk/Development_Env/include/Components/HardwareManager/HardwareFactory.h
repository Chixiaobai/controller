#pragma once

// This file is just designed for simplify driver implementation,you should bind it with FrameworkCoreStub and
// LogSupportStub.

#include "Environment/Environment.h"
#include "File/File.h"
#include "StringAssistant/StringAssistant.h"
#include "HardwareManager/IHardwareFactory.h"

DECLARE_STUB_INTERFACE(IHardwareFactoryStub, IHardwareFactory);

template <class hardwareclass>
class CHardwareFactory : public IHardwareFactoryStub {
public:
    CHardwareFactory(const char *strFactoryName, const char *strTemplateFile) {
        m_strFactoryName = strFactoryName;
        m_strTemplateFile = strTemplateFile;
        LOG_DEBUG("Factory[%s] is created\n", strFactoryName);
    }
    ~CHardwareFactory() { LOG_DEBUG("Factory[%s] is destroyed\n", m_strFactoryName.c_str()); }

public:
    virtual IHardwareCore *createHardware(const char *strHardwareName, const char *strWorksapce) {
        string strSTLWorkspace;
        if (strWorksapce == nullptr || strWorksapce[0] == '\0')
            strSTLWorkspace = CEnvironment::getApplicationDir() + "etc" PATH_SPLIT_STRING "Hardware" PATH_SPLIT_STRING;
        else
            strSTLWorkspace = string(strWorksapce) + PATH_SPLIT_STRING "Hardware" PATH_SPLIT_STRING;
        string strHardwareConfigrationDir = strSTLWorkspace + m_strFactoryName + PATH_SPLIT_STRING;
        string strHardwareConfigration = strHardwareConfigrationDir + strHardwareName + ".xml";
        if (OS::File::exist(strHardwareConfigration) == false) {
            OS::Folder::create(strHardwareConfigrationDir);
            if (OS::File::copy(m_strTemplateFile, strHardwareConfigration, true) == false) return nullptr;
        }
        hardwareclass *hardwareInstance = new hardwareclass();
        if (hardwareInstance->initHardware(strHardwareName, strHardwareConfigration.c_str()) == false) {
            hardwareInstance->releaseRef();
            hardwareInstance = nullptr;
        }
        return hardwareInstance;
    }

protected:
    string m_strFactoryName;
    string m_strTemplateFile;
};
