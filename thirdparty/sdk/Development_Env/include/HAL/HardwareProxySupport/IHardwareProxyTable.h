#pragma once

#include "IHardwareProxyRecord.h"

interface IHardwareProxyTable : public IBase  //接口实例的代理表格
{
    virtual IHardwareProxyRecord *getHardwareProxyRecord(const char *strProxyRecordName) = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareProxyTable)
