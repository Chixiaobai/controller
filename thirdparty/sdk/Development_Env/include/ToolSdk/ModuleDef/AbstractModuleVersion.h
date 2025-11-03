#ifndef __TOOLSDK_ABSTRACT_MODULE_VERSION_H__
#define __TOOLSDK_ABSTRACT_MODULE_VERSION_H__

#include "ModuleDef/IModuleVersion.h"
#include "StringAssistant/StringAssistant.h"
#include "PkgVersion.h"

#define SET_VERSION_METHOD(majorversion, minorversion, revision)         \
    virtual unsigned int getMajorVersionNum() { return (majorversion); } \
    virtual unsigned int getMinorVersionNum() { return (minorversion); } \
    virtual unsigned int getRevisionNum() { return (revision); }

DECLARE_STUB_INTERFACE(IModuleVersionStub, IModuleVersion)

class CAbstractModuleVersion : public IModuleVersionStub {
public:
    CAbstractModuleVersion() { m_strBuildTime = "请为模块调用UpdateBuildTime方法更新编译版本号"; }
    virtual bool getBuildVersion(char *strBuildVersion, unsigned long &ulVersionLen) override {
        return CStringAssistant::getBaseStringFromSTLString(m_strBuildTime, strBuildVersion, ulVersionLen);
    }
    virtual bool getFullVersion(char *strFullVersion, unsigned long &ulVersionLen) override {
        string strVersion = CStringAssistant::format("%d.%d.%d  BUILDTIME : %s", getMajorVersionNum(),
                                                     getMinorVersionNum(), getRevisionNum(), m_strBuildTime.c_str());
        return CStringAssistant::getBaseStringFromSTLString(strVersion, strFullVersion, ulVersionLen);
    }
    virtual bool getShortVersion(char *strShortVersion, unsigned long &ulVersionLen) override {
        string strVersion =
            CStringAssistant::format("%d.%d.%d", getMajorVersionNum(), getMinorVersionNum(), getRevisionNum());
        return CStringAssistant::getBaseStringFromSTLString(strVersion, strShortVersion, ulVersionLen);
    }

protected:
    void updateBuildTime() { m_strBuildTime = PKG_BUILD_TIME; }

protected:
    string m_strBuildTime;  //模块的编译时间
};

#endif
