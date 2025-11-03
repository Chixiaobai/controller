#pragma once

#include "RpcMethod.h"

interface IRpcServant;
//定义命令有关宏
typedef IRpcRequest *(*CREATE_RPC_REQUEST_FUNC)(UINT uintObjectId, IRpcServant *servant);

struct CRuntimeRpcRequest {
    USHORT uiRpcInterfaceId;
    USHORT uiRpcMethodId;
    CREATE_RPC_REQUEST_FUNC pfCreateRpcRequest;
    CRuntimeRpcRequest *pNextRuntimeRpcRequestClass;
    static CRuntimeRpcRequest *pHeadRuntimeRpcRequestClass;
    IRpcRequest *createObject(UINT uintObjectId, IRpcServant *servant) {
        if (pfCreateRpcRequest == nullptr)
            return nullptr;
        else
            return (*pfCreateRpcRequest)(uintObjectId, servant);
    }
    static CRuntimeRpcRequest *fromRpcRequestId(USHORT uiInterfaceId, USHORT uiMethodId) {
        CRuntimeRpcRequest *pCurRuntimeClass = pHeadRuntimeRpcRequestClass;
        while (pCurRuntimeClass != nullptr) {
            if (uiInterfaceId == pCurRuntimeClass->uiRpcInterfaceId && uiMethodId == pCurRuntimeClass->uiRpcMethodId)
                return pCurRuntimeClass;
            pCurRuntimeClass = pCurRuntimeClass->pNextRuntimeRpcRequestClass;
        }
        return nullptr;
    }
};

struct GCBindRuntimeRpcRequestClass {
    GCBindRuntimeRpcRequestClass(CRuntimeRpcRequest *pRuntimeClass) {
        pRuntimeClass->pNextRuntimeRpcRequestClass = CRuntimeRpcRequest::pHeadRuntimeRpcRequestClass;
        CRuntimeRpcRequest::pHeadRuntimeRpcRequestClass = pRuntimeClass;
    }
};

#define DECLARE_RPC_REQUEST(classname)                           \
protected:                                                       \
    classname(UINT uintObjectId, IRpcServant *servant);          \
    virtual ~classname();                                        \
                                                                 \
public:                                                          \
    virtual IRpcServant *getServant();                           \
    virtual UINT getRpcObjectId();                               \
    virtual USHORT getRpcInterfaceId();                          \
    virtual USHORT getRpcMethodId();                             \
                                                                 \
protected:                                                       \
    UINT m_uintRpcRequestObjectId;                               \
    IRpcServant *m_pRpcServant;                                  \
                                                                 \
public:                                                          \
    static CRuntimeRpcRequest classname##RuntimeRpcRequestClass; \
                                                                 \
public:                                                          \
    static IRpcRequest *createRpcRequest(UINT uintObjectId, IRpcServant *servant);

#define IMPLEMENT_RPC_REQUEST(interface_id, method_id, classname)                                                      \
    CRuntimeRpcRequest classname::classname##RuntimeRpcRequestClass = {interface_id, method_id,                        \
                                                                       classname::createRpcRequest, nullptr};          \
    GCBindRuntimeRpcRequestClass classname##BindRuntimeRpcRequestClass(&classname::classname##RuntimeRpcRequestClass); \
    classname::classname(UINT uintObjectId, IRpcServant *servant) {                                                    \
        m_uintRpcRequestObjectId = uintObjectId;                                                                       \
        m_pRpcServant = servant;                                                                                       \
    }                                                                                                                  \
    classname::~classname() {}                                                                                         \
    IRpcServant *classname::getServant() {                                                                             \
        if (m_pRpcServant != nullptr) m_pRpcServant->addRef();                                                         \
        return m_pRpcServant;                                                                                          \
    }                                                                                                                  \
    UINT classname::getRpcObjectId() { return m_uintRpcRequestObjectId; }                                              \
    USHORT classname::getRpcInterfaceId() { return interface_id; }                                                     \
    USHORT classname::getRpcMethodId() { return method_id; }                                                           \
    IRpcRequest *classname::createRpcRequest(UINT uintObjectId, IRpcServant *servant) {                                \
        return new classname(uintObjectId, servant);                                                                   \
    }

//定义反馈有关宏
typedef IRpcResponse *(*CREATE_RPC_RESPONSE_FUNC)(UINT uintObjectId, IRpcServant *servant);

struct CRuntimeRpcResponse {
    USHORT uiRpcInterfaceId;
    USHORT uiRpcMethodId;
    CREATE_RPC_RESPONSE_FUNC pfCreateRpcResponse;
    CRuntimeRpcResponse *pNextRuntimeRpcResponseClass;
    static CRuntimeRpcResponse *pHeadRuntimeRpcResponseClass;
    IRpcResponse *createObject(UINT uintObjectId, IRpcServant *servant) {
        if (pfCreateRpcResponse == nullptr)
            return nullptr;
        else
            return (*pfCreateRpcResponse)(uintObjectId, servant);
    }
    static CRuntimeRpcResponse *fromRpcResponseId(USHORT uiInterfaceId, USHORT uiMethodId) {
        CRuntimeRpcResponse *pCurRuntimeClass = pHeadRuntimeRpcResponseClass;
        while (pCurRuntimeClass != nullptr) {
            if (uiInterfaceId == pCurRuntimeClass->uiRpcInterfaceId && uiMethodId == pCurRuntimeClass->uiRpcMethodId)
                return pCurRuntimeClass;
            pCurRuntimeClass = pCurRuntimeClass->pNextRuntimeRpcResponseClass;
        }
        return nullptr;
    }
};

struct GCBindRuntimeRpcResponseClass {
    GCBindRuntimeRpcResponseClass(CRuntimeRpcResponse *pRuntimeClass) {
        pRuntimeClass->pNextRuntimeRpcResponseClass = CRuntimeRpcResponse::pHeadRuntimeRpcResponseClass;
        CRuntimeRpcResponse::pHeadRuntimeRpcResponseClass = pRuntimeClass;
    }
};

#define DECLARE_RPC_RESPONSE(classname)                            \
protected:                                                         \
    classname(UINT uintObjectId, IRpcServant *servant);            \
    virtual ~classname();                                          \
                                                                   \
public:                                                            \
    virtual IRpcServant *getServant();                             \
    virtual UINT getRpcObjectId();                                 \
    virtual USHORT getRpcInterfaceId();                            \
    virtual USHORT getRpcMethodId();                               \
                                                                   \
protected:                                                         \
    UINT m_uintRpcResponseObjectId;                                \
    IRpcServant *m_pRpcServant;                                    \
                                                                   \
public:                                                            \
    static CRuntimeRpcResponse classname##RuntimeRpcResponseClass; \
                                                                   \
public:                                                            \
    static IRpcResponse *createRpcResponse(UINT uintObjectId, IRpcServant *servant);

#define IMPLEMENT_RPC_RESPONSE(interface_id, method_id, classname)                                               \
    CRuntimeRpcResponse classname::classname##RuntimeRpcResponseClass = {interface_id, method_id,                \
                                                                         classname::createRpcResponse, nullptr}; \
    GCBindRuntimeRpcResponseClass classname##BindRuntimeRpcResponseClass(                                        \
        &classname::classname##RuntimeRpcResponseClass);                                                         \
    classname::classname(UINT uintObjectId, IRpcServant *servant) {                                              \
        m_uintRpcResponseObjectId = uintObjectId;                                                                \
        m_pRpcServant = servant;                                                                                 \
    }                                                                                                            \
    classname::~classname() {}                                                                                   \
    IRpcServant *classname::getServant() {                                                                       \
        if (m_pRpcServant != nullptr) m_pRpcServant->addRef();                                                   \
        return m_pRpcServant;                                                                                    \
    }                                                                                                            \
    UINT classname::getRpcObjectId() { return m_uintRpcResponseObjectId; }                                       \
    USHORT classname::getRpcInterfaceId() { return interface_id; }                                               \
    USHORT classname::getRpcMethodId() { return method_id; }                                                     \
    IRpcResponse *classname::createRpcResponse(UINT uintObjectId, IRpcServant *servant) {                        \
        return new classname(uintObjectId, servant);                                                             \
    }

//定义反馈有关宏
typedef IRpcNotify *(*CREATE_RPC_NOTIFY_FUNC)(UINT uintObjectId, IRpcServant *servant);

struct CRuntimeRpcNotify {
    USHORT uiRpcInterfaceId;
    USHORT uiRpcNotifyId;
    CREATE_RPC_NOTIFY_FUNC pfCreateRpcNotify;
    CRuntimeRpcNotify *pNextRuntimeRpcNotifyClass;
    static CRuntimeRpcNotify *pHeadRuntimeRpcNotifyClass;
    IRpcNotify *createObject(UINT uintObjectId, IRpcServant *servant) {
        if (pfCreateRpcNotify == nullptr)
            return nullptr;
        else
            return (*pfCreateRpcNotify)(uintObjectId, servant);
    }
    static CRuntimeRpcNotify *fromRpcNotifyId(USHORT uiInterfaceId, USHORT uiNotifyId) {
        CRuntimeRpcNotify *pCurRuntimeClass = pHeadRuntimeRpcNotifyClass;
        while (pCurRuntimeClass != nullptr) {
            if (uiInterfaceId == pCurRuntimeClass->uiRpcInterfaceId && uiNotifyId == pCurRuntimeClass->uiRpcNotifyId)
                return pCurRuntimeClass;
            pCurRuntimeClass = pCurRuntimeClass->pNextRuntimeRpcNotifyClass;
        }
        return nullptr;
    }
};

struct GCBindRuntimeRpcNotifyClass {
    GCBindRuntimeRpcNotifyClass(CRuntimeRpcNotify *pRuntimeClass) {
        pRuntimeClass->pNextRuntimeRpcNotifyClass = CRuntimeRpcNotify::pHeadRuntimeRpcNotifyClass;
        CRuntimeRpcNotify::pHeadRuntimeRpcNotifyClass = pRuntimeClass;
    }
};

#define DECLARE_RPC_NOTIFY(classname)                          \
protected:                                                     \
    classname(UINT uintObjectId, IRpcServant *servant);        \
    virtual ~classname();                                      \
                                                               \
public:                                                        \
    virtual IRpcServant *getServant();                         \
    virtual UINT getRpcObjectId();                             \
    virtual USHORT getRpcInterfaceId();                        \
    virtual USHORT getRpcNotifyId();                           \
                                                               \
protected:                                                     \
    UINT m_uintRpcNotifyObjectId;                              \
    IRpcServant *m_pRpcServant;                                \
                                                               \
public:                                                        \
    static CRuntimeRpcNotify classname##RuntimeRpcNotifyClass; \
                                                               \
public:                                                        \
    static IRpcNotify *createRpcNotify(UINT uintObjectId, IRpcServant *servant);

#define IMPLEMENT_RPC_NOTIFY(interface_id, notify_id, classname)                                                    \
    CRuntimeRpcNotify classname::classname##RuntimeRpcNotifyClass = {interface_id, notify_id,                       \
                                                                     classname::createRpcNotify, nullptr};          \
    GCBindRuntimeRpcNotifyClass classname##BindRuntimeRpcNotifyClass(&classname::classname##RuntimeRpcNotifyClass); \
    classname::classname(UINT uintObjectId, IRpcServant *servant) {                                                 \
        m_uintRpcNotifyObjectId = uintObjectId;                                                                     \
        m_pRpcServant = servant;                                                                                    \
    }                                                                                                               \
    classname::~classname() {}                                                                                      \
    IRpcServant *classname::getServant() {                                                                          \
        if (m_pRpcServant != nullptr) m_pRpcServant->addRef();                                                      \
        return m_pRpcServant;                                                                                       \
    }                                                                                                               \
    UINT classname::getRpcObjectId() { return m_uintRpcNotifyObjectId; }                                            \
    USHORT classname::getRpcInterfaceId() { return interface_id; }                                                  \
    USHORT classname::getRpcNotifyId() { return notify_id; }                                                        \
    IRpcNotify *classname::createRpcNotify(UINT uintObjectId, IRpcServant *servant) {                               \
        return new classname(uintObjectId, servant);                                                                \
    }

#define IMPLEMENT_RPC_SERVICE_RUNTIME()                                               \
    CRuntimeRpcRequest *CRuntimeRpcRequest::pHeadRuntimeRpcRequestClass = nullptr;    \
    CRuntimeRpcResponse *CRuntimeRpcResponse::pHeadRuntimeRpcResponseClass = nullptr; \
    CRuntimeRpcNotify *CRuntimeRpcNotify::pHeadRuntimeRpcNotifyClass = nullptr;
