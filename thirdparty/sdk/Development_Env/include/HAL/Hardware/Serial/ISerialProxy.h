#pragma once

#include "Base/IBase.h"

interface ISerialProxy : public IBase {
    virtual int mappingBaudIndex(int iBaud) = 0;
};
