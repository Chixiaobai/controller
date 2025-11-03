#pragma once

#include "Base/IBase.h"
#include "BoundedStream/BoundedStream.h"

interface IRpcRequest : public IBase {
    virtual bool processRequest(CBoundedStream & responseStream) = 0;
    virtual bool fromStream(CBoundedStream & requestStream) = 0;
};

interface IRpcResponse : public IBase {
    virtual bool toStream(CBoundedStream & responseStream) = 0;
};

interface IRpcNotify : public IBase {
    virtual bool toStream(CBoundedStream & notifyStream) = 0;
};

DECLARE_STUB_INTERFACE(IRpcRequestStub, IRpcRequest)
DECLARE_STUB_INTERFACE(IRpcResponseStub, IRpcResponse)
DECLARE_STUB_INTERFACE(IRpcNotifyStub, IRpcNotify)
