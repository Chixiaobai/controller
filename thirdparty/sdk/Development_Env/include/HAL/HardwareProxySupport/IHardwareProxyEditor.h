#pragma once

#include "Base/IBase.h"

interface IHardwareProxyRecordEditor : public IBase {
    virtual void setProxyRecordName(const char *strProxyRecordName) = 0;
    virtual bool setPropertyItem(UINT uPropertyId, const char *strPropertyValue) = 0;
    virtual void removePropertyItem(UINT uPropertyId) = 0;
    virtual void removeAllPropertyItem() = 0;
};

interface IHardwareProxyTableEditor : public IBase {
    virtual IHardwareProxyRecordEditor *createHardwareProxyRecord() = 0;
    virtual bool addHardwareProxyRecord(const char *strProxyRecordName,
                                        IHardwareProxyRecordEditor *pHardwareProxyRecord) = 0;
    virtual void removeHardwareProxyRecord(const char *strProxyRecordName) = 0;
    virtual void removeAllHardwareProxyRecord() = 0;
};

interface IHardwareProxyManagerEditor : public IBase {
    virtual IHardwareProxyTableEditor *createHardwareProxyTable() = 0;
    virtual bool addHardwareProxyTable(const char *strHardwareName, IHardwareProxyTableEditor *pHardwareProxyTable) = 0;
    virtual void removeHardwareProxyTable(const char *strHardwareName) = 0;
    virtual void removeAllHardwareProxyTable() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareProxyRecordEditor)
ENABLE_RECOGNIZE_MACRO(IHardwareProxyTableEditor)
ENABLE_RECOGNIZE_MACRO(IHardwareProxyManagerEditor)
