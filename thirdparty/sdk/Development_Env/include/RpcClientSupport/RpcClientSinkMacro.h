#pragma once

#include "Base/IBase.h"

interface IRpcClientSink : public IBase {
    virtual void initImplSink(IBase * pImpl) = 0;
};

ENABLE_RECOGNIZE_MACRO(IRpcClientSink)

DECLARE_STUB_INTERFACE(IRpcClientSinkStub, IRpcClientSink)

//定义命令有关宏
typedef IRpcClientSink *(*CREATE_RPC_SINK_FUNC)(unsigned int uintObjectId);

struct CRpcSinkRuntime {
    USHORT uiRpcInterfaceId;
    CREATE_RPC_SINK_FUNC pfCreateRpcSink;
    CRpcSinkRuntime *pNextRpcSinkRuntimeClass;
    static CRpcSinkRuntime *pHeadRpcSinkRuntimeClass;
    IRpcClientSink *createObject(unsigned int uintObjectId) {
        if (pfCreateRpcSink == nullptr)
            return nullptr;
        else
            return (*pfCreateRpcSink)(uintObjectId);
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
    classname(unsigned int uintObjectId);                  \
    virtual ~classname();                                  \
                                                           \
public:                                                    \
    virtual UINT getRpcObjectId();                         \
    virtual USHORT getRpcInterfaceId();                    \
    virtual void initImplSink(IBase *pImpl);               \
                                                           \
protected:                                                 \
    virtual void onFinalRelease();                         \
                                                           \
protected:                                                 \
    UINT m_uintRpcObjectId;                                \
    CReference<IRpcClient> m_RpcSinkClient;                \
    IBase *m_SinkImpl;                                     \
                                                           \
public:                                                    \
    static CRpcSinkRuntime classname##RpcSinkRuntimeClass; \
                                                           \
public:                                                    \
    static IRpcClientSink *createRpcSink(unsigned int uintObjectId);

#define IMPLEMENT_RPC_SINK(servant_id, interface_id, classname)                                                    \
    CRpcSinkRuntime classname::classname##RpcSinkRuntimeClass = {interface_id, classname::createRpcSink, nullptr}; \
    GCBindRpcSinkRuntimeClass classname##BindRpcSinkRuntimeClass(&classname::classname##RpcSinkRuntimeClass);      \
    classname::classname(unsigned int uintObjectId) {                                                              \
        CReference<IRpcClientManager> rpcClientManager = getRpcClientManager();                                    \
        if (rpcClientManager.nil() == false) {                                                                     \
            m_uintRpcObjectId = uintObjectId;                                                                      \
            m_RpcSinkClient = rpcClientManager->createRpcClient(servant_id, uintObjectId);                         \
            rpcClientManager->registRpcClient(m_RpcSinkClient);                                                    \
            m_RpcSinkClient->setRpcNotifyCallback(this);                                                           \
        }                                                                                                          \
    }                                                                                                              \
    classname::~classname() {}                                                                                     \
    UINT classname::getRpcObjectId() { return m_uintRpcObjectId; }                                                 \
    USHORT classname::getRpcInterfaceId() { return interface_id; }                                                 \
    void classname::initImplSink(IBase *pImpl) { m_SinkImpl = pImpl; }                                             \
    void classname::onFinalRelease() {                                                                             \
        CReference<IRpcClientManager> rpcClientManager = getRpcClientManager();                                    \
        if (m_RpcSinkClient.nil() == false) {                                                                      \
            m_RpcSinkClient->setRpcNotifyCallback(nullptr);                                                        \
            rpcClientManager->unregistRpcClient(m_RpcSinkClient->getRpcServantId(),                                \
                                                m_RpcSinkClient->getRpcObjectId());                                \
            m_RpcSinkClient.release();                                                                             \
        }                                                                                                          \
    }                                                                                                              \
    IRpcClientSink *classname::createRpcSink(unsigned int uintObjectId) { return new classname(uintObjectId); }

#define IMPLEMENT_RPC_SINK_RUNTIME() CRpcSinkRuntime *CRpcSinkRuntime::pHeadRpcSinkRuntimeClass = nullptr;
