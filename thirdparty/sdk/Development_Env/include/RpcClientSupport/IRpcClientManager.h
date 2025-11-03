#pragma once

#include "Base/IBase.h"
#include "IRpcClient.h"

/**
 * @interface IRpcClientManager
 * @brief Interface for managing RPC client instances and their associated resources.
 *
 * This interface provides methods to create, register, retrieve, and unregister RPC clients.
 * It also allows associating and retrieving local object handles with specific RPC clients.
 */
interface IRpcClientManager : public IBase {
    /**
     * @brief Creates a new RPC client instance.
     *
     * @param uiRpcServantId The servant ID to associate with the client.
     * @param uintRpcObjectId The object ID to associate with the client.
     * @return Pointer to the newly created IRpcClient instance, or nullptr on failure.
     */
    virtual IRpcClient *createRpcClient(USHORT uiRpcServantId, UINT uintRpcObjectId) = 0;

    /**
     * @brief Registers an existing RPC client.
     *
     * @param pClient Pointer to the IRpcClient instance to register.
     * @return true if the client was successfully registered; false otherwise.
     */
    virtual bool registRpcClient(IRpcClient * pClient) = 0;

    /**
     * @brief Retrieves an RPC client by its identifiers.
     *
     * @param uiRpcServantId The servant ID of the client to retrieve.
     * @param uintRpcObjectId The object ID of the client to retrieve.
     * @return Pointer to the matching IRpcClient instance, or nullptr if not found.
     */
    virtual IRpcClient *getRpcClient(USHORT uiRpcServantId, UINT uintRpcObjectId) = 0;

    /**
     * @brief Checks whether an RPC client exists for the given identifiers.
     *
     * @param uiRpcServantId The servant ID to check.
     * @param uintRpcObjectId The object ID to check.
     * @return true if the client exists; false otherwise.
     */
    virtual bool existRpcClient(USHORT uiRpcServantId, UINT uintRpcObjectId) = 0;

    /**
     * @brief Unregisters an RPC client identified by its IDs.
     *
     * @param uiRpcServantId The servant ID of the client to unregister.
     * @param uintRpcObjectId The object ID of the client to unregister.
     * @return true if the client was successfully unregistered; false otherwise.
     */
    virtual bool unregistRpcClient(USHORT uiRpcServantId, UINT uintRpcObjectId) = 0;

    /**
     * @brief Associates a local object handle with an RPC client.
     *
     * @param uiRpcServantId The servant ID of the associated client.
     * @param uintRpcObjectId The object ID of the associated client.
     * @param hLocalObjectHandle Handle to the local object to associate.
     */
    virtual void setLocalObjectHandle(USHORT uiRpcServantId, UINT uintRpcObjectId,
                                      DWORD_PTR hLocalObjectHandle = (DWORD_PTR) nullptr) = 0;

    /**
     * @brief Retrieves the local object handle associated with an RPC client.
     *
     * @param uiRpcServantId The servant ID of the client.
     * @param uintRpcObjectId The object ID of the client.
     * @return The associated local object handle, or nullptr if none is associated.
     */
    virtual DWORD_PTR getLocalObjectHandle(USHORT uiRpcServantId, UINT uintRpcObjectId) = 0;
};

/// Enables runtime recognition of the IRpcClientManager interface
ENABLE_RECOGNIZE_MACRO(IRpcClientManager)
