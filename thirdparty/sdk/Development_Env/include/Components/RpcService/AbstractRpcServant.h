#pragma once

// This file is just designed for simplify rpc servant implementation,you should call macro
// 'IMPLEMENT_RPC_SERVICE_RUNTIME()' in your cpp file.

#include "RpcMacro.h"
#include "IRpcServant.h"
#include "IRpcServiceInner.h"
#include "Reference/Reference.h"

template <class T>
class CAbstractRpcRequest : public T, public IRpcRequestStub {};
template <class T>
class CAbstractRpcResponse : public T, public IRpcResponseStub {};
template <class T>
class CAbstractRpcNotify : public T, public IRpcNotifyStub {};

DECLARE_STUB_INTERFACE(IRpcDescriptorStub, IRpcDescriptor)

class CRpcDescriptor : public IRpcDescriptorStub {
public:
    CRpcDescriptor(IRpcServant *pRpcServant, const unsigned int intObjectId, const unsigned short uiInterfaceId,
                   const unsigned short uiMethodId, const unsigned char *inputStream, int inputSize) {
        m_pRpcServant = pRpcServant;
        m_uintObjectId = intObjectId;
        m_uiInterfaceId = uiInterfaceId;
        m_uiMethodId = uiMethodId;
        if (inputStream != nullptr && inputSize > 0) {
            m_pInputStream = new unsigned char[inputSize];
            memcpy(m_pInputStream, inputStream, inputSize);
            m_intInputSize = inputSize;
        } else {
            m_pInputStream = nullptr;
            m_intInputSize = 0;
        }
        m_pOutputStream = nullptr;
        m_intOutputSize = 0;
    }

public:
    virtual const unsigned short getServantId() override {
        if (m_pRpcServant != nullptr) {
            return m_pRpcServant->getServantId();
        } else
            return 0;
    }
    virtual const unsigned int getObjectId() override { return m_uintObjectId; }
    virtual const unsigned short getInterfaceId() override { return m_uiInterfaceId; }
    virtual const unsigned short getMethodId() override { return m_uiMethodId; }
    virtual bool invoke() override {
        CBoundedStream requestStream;
        try {
            if (m_pInputStream != nullptr) requestStream.writeObject(m_pInputStream, m_intInputSize);
        } catch (CAbstractStream::CStreamException *pException) {
            delete pException;
            pException = nullptr;
        }
        CReference<IRpcRequest> rpcRequest =
            createRpcRequest(m_pRpcServant, m_uintObjectId, m_uiInterfaceId, m_uiMethodId);
        if (rpcRequest.nil() == false && rpcRequest->fromStream(requestStream) == true) {
            CBoundedStream responseStream;
            if (rpcRequest->processRequest(responseStream) == true) {
                m_intOutputSize = (int)responseStream.getLength();
                if (m_intOutputSize > 0) {
                    m_pOutputStream = new unsigned char[m_intOutputSize];
                    try {
                        responseStream.readObject(m_pOutputStream, m_intOutputSize);
                    } catch (CAbstractStream::CStreamException *pException) {
                        delete pException;
                        pException = nullptr;
                    }
                } else
                    m_pOutputStream = nullptr;
                return true;
            } else
                return false;
        } else
            return false;
    }
    virtual bool getReturn(unsigned char *outputStream, int &outputSize) override {
        if (outputStream == nullptr) {
            outputSize = m_intOutputSize;
            return true;
        } else {
            if (outputSize >= m_intOutputSize) {
                if (m_pOutputStream != nullptr) memcpy(outputStream, m_pOutputStream, m_intOutputSize);
                outputSize = m_intOutputSize;
                return true;
            } else
                return false;
        }
    }

protected:
    virtual void onFinalRelease() override {
        if (m_pOutputStream != nullptr) {
            delete[] m_pOutputStream;
            m_pOutputStream = nullptr;
        }
        m_intOutputSize = 0;
        if (m_pInputStream != nullptr) {
            delete[] m_pInputStream;
            m_pInputStream = nullptr;
        }
        m_intInputSize = 0;
    }

public:
    static IRpcRequest *createRpcRequest(IRpcServant *pRpcServant, UINT uintObjectId, USHORT uiInterfaceId,
                                         USHORT uiMethodId) {
        CRuntimeRpcRequest *pRuntimeRpcRequest = CRuntimeRpcRequest::fromRpcRequestId(uiInterfaceId, uiMethodId);
        if (pRuntimeRpcRequest != nullptr) {
            return pRuntimeRpcRequest->createObject(uintObjectId, pRpcServant);
        } else
            return nullptr;
    }
    static IRpcResponse *createRpcResponse(IRpcServant *pRpcServant, UINT uintObjectId, USHORT uiInterfaceId,
                                           USHORT uiMethodId) {
        CRuntimeRpcResponse *pRuntimeRpcResponse = CRuntimeRpcResponse::fromRpcResponseId(uiInterfaceId, uiMethodId);
        if (pRuntimeRpcResponse != nullptr) {
            return pRuntimeRpcResponse->createObject(uintObjectId, pRpcServant);
        } else
            return nullptr;
    }
    static IRpcNotify *createRpcNotify(IRpcServant *pRpcServant, UINT uintObjectId, USHORT uiInterfaceId,
                                       USHORT uiNotifyId) {
        CRuntimeRpcNotify *pRuntimeRpcNotify = CRuntimeRpcNotify::fromRpcNotifyId(uiInterfaceId, uiNotifyId);
        if (pRuntimeRpcNotify != nullptr) {
            return pRuntimeRpcNotify->createObject(uintObjectId, pRpcServant);
        } else
            return nullptr;
    }

protected:
    IRpcServant *m_pRpcServant;
    unsigned int m_uintObjectId;
    unsigned short m_uiInterfaceId;
    unsigned short m_uiMethodId;
    unsigned char *m_pInputStream;
    int m_intInputSize;
    unsigned char *m_pOutputStream;
    int m_intOutputSize;
};

DECLARE_STUB_INTERFACE(IRpcServantStub, IRpcServant)

class CAbstractRpcServant : public IRpcServantStub {
public:
    CAbstractRpcServant(IBase *pComponent, IRpcServiceInner *pRpcService) {
        if (pComponent != nullptr) pComponent->addRef();
        m_ServantComponent = pComponent;
        m_pRpcService = pRpcService;
    }

public:
    virtual IBase *getComponent() override {
        if (m_ServantComponent.nil() == false) m_ServantComponent->addRef();
        return m_ServantComponent;
    }
    virtual IRpcServiceInner *getRpcService() override {
        if (m_pRpcService != nullptr) m_pRpcService->addRef();
        return m_pRpcService;
    }
    virtual bool registRpcServant() override {
        if (m_pRpcService != nullptr) {
            return m_pRpcService->registServant(this);
        } else
            return false;
    }
    virtual void unregistRpcServant() override {
        if (m_pRpcService != nullptr) m_pRpcService->unregistServant(getServantId());
    }
    virtual IRpcDescriptor *createDescriptor(const unsigned int uintObjectId, const short uiInterfaceId,
                                             const unsigned short uiMethodId, const unsigned char *inputStream,
                                             int inputSize) override {
        return new CRpcDescriptor(this, uintObjectId, uiInterfaceId, uiMethodId, inputStream, inputSize);
    }
    virtual bool postNotify(const unsigned int uintObjectId, unsigned short uiInterfaceId, unsigned short uiNotifyCode,
                            IRpcNotify *notify) {
        if (m_pRpcService != nullptr) {
            CBoundedStream notifyStream;
            if (notify->toStream(notifyStream) == true) {
                size_t notifySize = notifyStream.getLength();
                if (notifySize > 0) {
                    bool ret = false;
                    unsigned char *notifyBuffer = new unsigned char[notifySize];
                    try {
                        notifyStream.readObject(notifyBuffer, notifySize);
                        ret = m_pRpcService->postNotify(getServantId(), uintObjectId, uiInterfaceId, uiNotifyCode,
                                                        notifyBuffer, (USHORT)notifySize);
                    } catch (CAbstractStream::CStreamException *pException) {
                        delete pException;
                        pException = nullptr;
                    }
                    delete[] notifyBuffer;
                    notifyBuffer = nullptr;
                    return ret;
                } else {
                    return m_pRpcService->postNotify(getServantId(), uintObjectId, uiInterfaceId, uiNotifyCode, nullptr,
                                                     (USHORT)notifySize);
                }
            } else
                return false;
        } else
            return false;
    }

protected:
    virtual void onFinalRelease() override { m_ServantComponent.release(); }

protected:
    CReference<IBase> m_ServantComponent;
    IRpcServiceInner *m_pRpcService;  //此处绝对不能为引用，因为在RpcService中已经引用了RpcServant
};
