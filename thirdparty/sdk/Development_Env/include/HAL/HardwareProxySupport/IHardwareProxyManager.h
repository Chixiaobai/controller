#pragma once

#include "IHardwareProxyTable.h"

interface IHardwareProxyManager : public IBase {
    virtual IHardwareProxyTable *getHardwareProxyTable(const char *strHardwareName) = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareProxyManager)
