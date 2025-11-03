#pragma once

#include "Base/IBase.h"

interface IRpcServerSink : public IBase {
    virtual void initRpcObjectId(UINT uintRpcObjectId) = 0;
};

ENABLE_RECOGNIZE_MACRO(IRpcServerSink)

DECLARE_STUB_INTERFACE(IRpcServerSinkStub, IRpcServerSink)

interface IRpcServant;
//定义命令有关宏
typedef IRpcServerSink *(*CREATE_RPC_SINK_FUNC)(IRpcServant *servant);

struct CRpcSinkRuntime {
    USHORT uiRpcInterfaceId;
    CREATE_RPC_SINK_FUNC pfCreateRpcSink;
    CRpcSinkRuntime *pNextRpcSinkRuntimeClass;
    static CRpcSinkRuntime *pHeadRpcSinkRuntimeClass;
    IRpcServerSink *createObject(IRpcServant *servant) {
        if (pfCreateRpcSink == nullptr)
            return nullptr;
        else
            return (*pfCreateRpcSink)(servant);
    }
    static CRpcSinkRuntime *fromRpcInterfaceId(USHORT uiInterfaceId) {
        CRpcSinkRuntime *pCurRuntimeClass = pHeadRpcSinkRuntimeClass;
        while (pCurRuntimeClass != nullptr) {
            if (uiInterfaceId == pCurRuntimeClass->uiRpcInterfaceId) return pCurRuntimeClass;
            pCurRuntimeClass = pCurRuntimeClass->pNextRpcSinkRuntimeClass;
        }
        return nullptr;
    }
};

struct GCBindRpcSinkRuntimeClass {
    GCBindRpcSinkRuntimeClass(CRpcSinkRuntime *pRuntimeClass) {
        pRuntimeClass->pNextRpcSinkRuntimeClass = CRpcSinkRuntime::pHeadRpcSinkRuntimeClass;
        CRpcSinkRuntime::pHeadRpcSinkRuntimeClass = pRuntimeClass;
    }
};

#define DECLARE_RPC_SINK(classname)                        \
protected:                                                 \
    classname(IRpcServant *servant);                       \
    virtual ~classname();                                  \
                                                           \
public:                                                    \
    virtual IRpcServant *getServant();                     \
    virtual void initRpcObjectId(UINT uintRpcObjectId);    \
    virtual UINT getRpcObjectId();                         \
    virtual USHORT getRpcInterfaceId();                    \
                                                           \
protected:                                                 \
    UINT m_uintRpcObjectId;                                \
    IRpcServant *m_pRpcServant;                            \
                                                           \
public:                                                    \
    static CRpcSinkRuntime classname##RpcSinkRuntimeClass; \
                                                           \
public:                                                    \
    static IRpcServerSink *createRpcSink(IRpcServant *servant);

#define IMPLEMENT_RPC_SINK(interface_id, classname)                                                                \
    CRpcSinkRuntime classname::classname##RpcSinkRuntimeClass = {interface_id, classname::createRpcSink, nullptr}; \
    GCBindRpcSinkRuntimeClass classname##BindRpcSinkRuntimeClass(&classname::classname##RpcSinkRuntimeClass);      \
    classname::classname(IRpcServant *servant) {                                                                   \
        m_uintRpcObjectId = 0;                                                                                     \
        m_pRpcServant = servant;                                                                                   \
    }                                                                                                              \
    classname::~classname() {}                                                                                     \
    IRpcServant *classname::getServant() {                                                                         \
        if (m_pRpcServant != nullptr) m_pRpcServant->addRef();                                                     \
        return m_pRpcServant;                                                                                      \
    }                                                                                                              \
    void classname::initRpcObjectId(UINT uintRpcObjectId) { m_uintRpcObjectId = uintRpcObjectId; }                 \
    UINT classname::getRpcObjectId() { return m_uintRpcObjectId; }                                                 \
    USHORT classname::getRpcInterfaceId() { return interface_id; }                                                 \
    IRpcServerSink *classname::createRpcSink(IRpcServant *servant) { return new classname(servant); }

#define IMPLEMENT_RPC_SINK_RUNTIME() CRpcSinkRuntime *CRpcSinkRuntime::pHeadRpcSinkRuntimeClass = nullptr;
