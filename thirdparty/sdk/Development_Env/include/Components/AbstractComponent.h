#pragma once

#include "IResourceContext.h"
#include "ModuleDef/AbstractModuleVersion.h"
#include "StringAssistant/StringAssistant.h"

interface IComponentEx : public IBase {
    virtual bool initComponent() = 0;
    virtual void exitComponent() = 0;
};

ENABLE_RECOGNIZE_MACRO(IComponentEx)

DECLARE_STUB_CLASS(IComponentStub, CAbstractModuleVersion, IComponent)
DECLARE_STUB_CLASS(IComponentExStub, IComponentStub, IComponentEx)

class CAbstractComponent : public IComponentExStub {
public:
    CAbstractComponent() {}
    virtual ~CAbstractComponent() {}

protected:
    virtual string getComponentClassName() = 0;

protected:
    virtual bool initComponent() override { return true; }
    virtual void exitComponent() override {}
    virtual bool getComponentClassName(char *pName, LPDWORD dwNameLen) override final {
        return CStringAssistant::getBaseStringFromSTLString(getComponentClassName(), pName, *dwNameLen);
    }
    virtual bool getComponentName(char *pName, LPDWORD dwNameLen) override final {
        return CStringAssistant::getBaseStringFromSTLString(m_strObjectName, pName, *dwNameLen);
    }
    virtual void setComponentName(const char *pName) override final { m_strObjectName = pName; }
    virtual IComponentProperties *getComponentProperties() override { return nullptr; }

protected:
    string m_strObjectName;
};
