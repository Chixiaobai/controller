#pragma once

#include "Base/ICallbackPoint.h"

#define INVALID_SERVANT_ID 0
//定义RPC调用
/**
 * @interface IRpcDescriptor
 * @brief Interface for describing and invoking RPC method calls.
 *
 * This interface provides methods to retrieve information about an RPC call,
 * including the target servant, object, interface, and method IDs. It also supports
 * invocation of the method and retrieval of the return value.
 */
interface IRpcDescriptor : public IBase {
    /**
     * @brief Retrieves the ID of the target servant for this RPC call.
     *
     * @return The servant ID.
     */
    virtual const unsigned short getServantId() = 0;

    /**
     * @brief Retrieves the ID of the target object for this RPC call.
     *
     * @return The object ID.
     */
    virtual const unsigned int getObjectId() = 0;

    /**
     * @brief Retrieves the interface ID associated with this RPC call.
     *
     * @return The interface ID.
     */
    virtual const unsigned short getInterfaceId() = 0;

    /**
     * @brief Retrieves the method ID representing the specific operation to invoke.
     *
     * @return The method ID.
     */
    virtual const unsigned short getMethodId() = 0;

    /**
     * @brief Invokes the remote procedure call described by this descriptor.
     *
     * @return true if the invocation succeeded; false otherwise.
     */
    virtual bool invoke() = 0;

    /**
     * @brief Retrieves the output data from the RPC method invocation.
     *
     * @param outputStream Buffer to store the returned data.
     * @param outputSize On input, specifies buffer size; on output, receives actual length.
     * @return true if the return data was successfully retrieved; false otherwise.
     */
    virtual bool getReturn(unsigned char *outputStream, int &outputSize) = 0;
};

interface IRpcServiceInner;
//定义RPC服务端对象
interface IRpcServant : public IBase {
    virtual IBase *getComponent() = 0;
    virtual IRpcServiceInner *getRpcService() = 0;
    virtual const unsigned short getServantId() = 0;
    virtual bool registRpcServant() = 0;
    virtual void unregistRpcServant() = 0;
    virtual IRpcDescriptor *createDescriptor(const unsigned int uintObjectId, const short uiInterfaceId,
                                             const unsigned short uiMethodId, const unsigned char *inputStream,
                                             int inputSize) = 0;
    virtual ICallbackPoint *createCallbackPoint(unsigned short uiInterfaceId) = 0;
};

ENABLE_RECOGNIZE_MACRO(IRpcDescriptor)
ENABLE_RECOGNIZE_MACRO(IRpcServant)
