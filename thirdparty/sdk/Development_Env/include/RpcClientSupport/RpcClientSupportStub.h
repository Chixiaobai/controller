#pragma once

#include "RpcClientSupport.h"
#include "IRpcCallback.h"
#include "IRemoteCallHelper.h"
#include "IRpcClient.h"
#include "IRpcClientManager.h"
#include "RpcClientMethod.h"
#include "RpcClientSinkMacro.h"

class CRpcClientSupport {
public:
    static bool invokeRpcMethod(IRpcClient *pRpcClient, unsigned short uiRpcInterfaceId, unsigned short uiRpcMethodId,
                                IRpcRequest *pRpcRequest, IRpcResponse *pRpcResponse, DWORD dwTimeout = 0) {
        if (pRpcClient != nullptr) {
            CReference<IRemoteCallHelper> remoteCallHelper = pRpcClient->getRemoteCallHelper();
            if (remoteCallHelper.nil() == false) {
                bool ret = true;
                CBoundedStream requestStream, responseStream;
                pRpcRequest->toStream(requestStream);
                int inputSize = (int)requestStream.getLength(), outputSize = 0;
                unsigned char *inputStream = nullptr;
                if (inputSize > 0) {
                    inputStream = new unsigned char[inputSize];
                    try {
                        requestStream.readObject(inputStream, inputSize);
                    } catch (CAbstractStream::CStreamException *pException) {
                        delete pException;
                        pException = nullptr;
                        ret = false;
                    }
                }
                if (ret == true) {
                    IRpcDescriptor *pRpcDescriptor = remoteCallHelper->createRpcDescriptor(
                        pRpcClient->getRpcServantId(), pRpcClient->getRpcObjectId(), uiRpcInterfaceId, uiRpcMethodId,
                        inputStream, inputSize, dwTimeout);
                    if (pRpcResponse != nullptr && pRpcResponse->isEmpty() == false)
                        pRpcDescriptor->initOutputDescriptor();
                    if (remoteCallHelper->invoke(pRpcDescriptor) == true) {
                        pRpcDescriptor->getReturn(nullptr, outputSize);
                        if (outputSize > 0) {
                            unsigned char *outputStream = new unsigned char[outputSize];
                            pRpcDescriptor->getReturn(outputStream, outputSize);
                            try {
                                responseStream.writeObject(outputStream, outputSize);
                                if (pRpcResponse->fromStream(responseStream) == true) {
                                    ret = pRpcResponse->processResponse();
                                }
                            } catch (CAbstractStream::CStreamException *pException) {
                                delete pException;
                                pException = nullptr;
                            }
                            delete[] outputStream;
                            outputStream = nullptr;
                        }
                    } else
                        ret = false;
                    pRpcDescriptor->releaseRef();
                }
                if (inputStream != nullptr) {
                    delete[] inputStream;
                    inputStream = nullptr;
                }
                return ret;
            } else
                return false;
        } else
            return false;
    }
};

template <class T>
class CRpcClientObjectLocator {
public:
    static T *getRpcObject(unsigned short uiServantId, unsigned int uintObjectId) {
        CReference<IRpcClientManager> rpcClientManager = getRpcClientManager();
        if (rpcClientManager.nil() == false) {
            T *pObject = (T *)rpcClientManager->getLocalObjectHandle(uiServantId, uintObjectId);
            if (pObject != nullptr) {
                pObject->addRef();
                return pObject;
            } else
                return new T(uiServantId, uintObjectId);
        } else
            return nullptr;
    }
};
