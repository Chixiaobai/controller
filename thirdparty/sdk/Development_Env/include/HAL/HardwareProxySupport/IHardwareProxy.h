#pragma once

#include "Base/IBase.h"

//实现方注意：IHardwareProxy的生命期应该交由IHardwareProxyWrapper管理
interface IHardwareProxy : public IBase {
    virtual bool getProxyName(char *strProxyName, DWORD &dwLen) = 0;
};

interface IHardwareProxyWrapper : public IBase {
    virtual IHardwareProxy *getHardwareProxy() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareProxy)
ENABLE_RECOGNIZE_MACRO(IHardwareProxyWrapper)
