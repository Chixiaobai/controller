#pragma once

#include "IRpcCallback.h"

/**
 * @interface IRpcDescriptor
 * @brief Interface for managing RPC call descriptors.
 *
 * This interface provides methods to initialize and retrieve various properties of an RPC call,
 * including service identifiers, input data, and return handling.
 */
interface IRpcDescriptor : public IBase {
    /**
     * @brief Initializes the output descriptor for the RPC response.
     */
    virtual void initOutputDescriptor() = 0;

    /**
     * @brief Retrieves the servant ID associated with this RPC call.
     *
     * @return The servant ID.
     */
    virtual unsigned short getServantId() = 0;

    /**
     * @brief Retrieves the object ID associated with this RPC call.
     *
     * @return The object ID.
     */
    virtual unsigned int getObjectId() = 0;

    /**
     * @brief Retrieves the interface ID associated with this RPC call.
     *
     * @return The interface ID.
     */
    virtual unsigned short getInterfaceId() = 0;

    /**
     * @brief Retrieves the method ID associated with this RPC call.
     *
     * @return The method ID.
     */
    virtual unsigned short getMethodId() = 0;

    /**
     * @brief Retrieves the input stream containing serialized request data.
     *
     * @return Pointer to the input stream buffer.
     */
    virtual const unsigned char *getInputStream() = 0;

    /**
     * @brief Retrieves the size of the input stream.
     *
     * @return Size of the input stream in bytes.
     */
    virtual int getInputSize() = 0;

    /**
     * @brief Retrieves the result of the RPC call.
     *
     * @param outputStream Buffer to receive the output data.
     * @param outputSize Reference to an integer that on input contains the buffer size,
     *                   and on output receives the actual size of the returned data.
     * @return true if the return value was successfully retrieved; false otherwise.
     */
    virtual bool getReturn(unsigned char *outputStream, int &outputSize) = 0;
};

/**
 * @interface IRemoteCallHelper
 * @brief Helper interface for initiating and invoking remote procedure calls (RPC).
 *
 * This interface provides methods to create RPC descriptors and invoke remote calls
 * using those descriptors.
 */
interface IRemoteCallHelper : public IBase {
    /**
     * @brief Creates an RPC descriptor with the specified parameters.
     *
     * @param uiServantId The servant ID to identify the target service.
     * @param uintObjectId The object ID associated with the target instance.
     * @param uiInterfaceId The interface ID identifying the service interface.
     * @param uiMethodId The method ID identifying the specific method to call.
     * @param inputStream Pointer to the serialized input data.
     * @param inputSize Size of the input data in bytes.
     * @param dwTimeout Timeout value for the RPC call in milliseconds.
     * @return Pointer to a newly created IRpcDescriptor instance, or nullptr on failure.
     */
    virtual IRpcDescriptor *createRpcDescriptor(unsigned short uiServantId, const unsigned int uintObjectId,
                                                const short uiInterfaceId, unsigned short uiMethodId,
                                                const unsigned char *inputStream, int inputSize, DWORD dwTimeout) = 0;

    /**
     * @brief Invokes the remote procedure call described by the given descriptor.
     *
     * @param pRpcDescriptor Pointer to the RPC descriptor containing call details.
     * @return true if the RPC was successfully invoked and completed; false otherwise.
     */
    virtual bool invoke(IRpcDescriptor * pRpcDescriptor) = 0;
};

/// Enables runtime recognition of the IRpcDescriptor interface
ENABLE_RECOGNIZE_MACRO(IRpcDescriptor)

/// Enables runtime recognition of the IRemoteCallHelper interface
ENABLE_RECOGNIZE_MACRO(IRemoteCallHelper)