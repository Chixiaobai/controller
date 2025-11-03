#pragma once

#include "IRpcClientMethod.h"
#include "RpcService/RpcServantDef.h"
#include "BoundedStream/BoundedStream.h"
#include "Reference/Reference.h"

DECLARE_STUB_INTERFACE(IRpcRequestStub, IRpcRequest)
template <class requeststruct>
class CAbstractRpcRequest : public requeststruct, public IRpcRequestStub {
    virtual bool isEmpty() { return sizeof(requeststruct) == sizeof(RPC_CONTENT_HOLDER); }
};

DECLARE_STUB_INTERFACE(IRpcResponseStub, IRpcResponse)
template <class responsestruct>
class CAbstractRpcResponse : public responsestruct, public IRpcResponseStub {
    virtual bool isEmpty() { return sizeof(responsestruct) == sizeof(RPC_CONTENT_HOLDER); }
};

DECLARE_STUB_INTERFACE(IRpcNotifyStub, IRpcNotify)
template <class notifystruct>
class CAbstractRpcNotify : public notifystruct, public IRpcNotifyStub {
    virtual bool isEmpty() { return sizeof(notifystruct) == sizeof(RPC_CONTENT_HOLDER); }
};

template <class responseclass>
class CAsynRequestResultNotify : public responseclass, public IRpcNotify {
public:
    virtual bool queryInterface(const char *name, void **pInterface) {
        if (strcmp(name, "IRpcNotify") == 0) {
            *pInterface = (IRpcNotify *)this;
            addRef();
            return true;
        } else
            return responseclass::queryInterface(name, pInterface);
    }
    virtual ULONG addRef() { return responseclass::addRef(); }
    virtual ULONG releaseRef() { return responseclass::releaseRef(); }
    virtual bool processNotify() { return responseclass::processResponse(); }
    virtual bool fromStream(CBoundedStream &notifyStream) { return responseclass::fromStream(notifyStream); }
};

#define RPC_REQUEST_DEFINITION(classname, varname) CReference<classname> varname = new classname

#define RPC_RESPONSE_DEFINITION(classname, varname) CReference<classname> varname = new classname

#define RPC_NOTIFY_DEFINITION(classname, varname) CReference<classname> varname = new classname
