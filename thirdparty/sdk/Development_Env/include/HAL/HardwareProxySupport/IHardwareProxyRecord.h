#pragma once

#include "IHardwareProxy.h"

#define PROXY_LIBRARY_PATH 1

interface IHardwareProxyRecord : public IBase {
    virtual bool getProxyRecordName(char *strProxyRecordName, DWORD &dwLen) = 0;
    virtual IHardwareProxyWrapper *getHardwareProxyWrapper() = 0;
    virtual bool getPropertyItem(UINT uPropertyId, char *strPropertyValue, DWORD &dwLen) = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareProxyRecord)

typedef IHardwareProxy *(*CREATE_HARDWARE_PROXY_FUNC)(IHardwareProxyWrapper *);
