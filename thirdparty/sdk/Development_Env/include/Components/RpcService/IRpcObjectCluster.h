#pragma once

#include "Base/IBase.h"

/**
 * @interface IRpcObjectCluster
 * @brief Interface for managing RPC object registration and lookup within a cluster.
 *
 * This interface provides methods to register, retrieve, and unregister RPC objects
 * using unique identifiers.
 */
interface IRpcObjectCluster : public IBase {
    /**
     * @brief Registers an RPC object with the cluster.
     *
     * @param pRpcObject Pointer to the IBase interface of the RPC object.
     * @param uintRpcObjectId Reference to store the assigned unique object ID.
     * @return true if registration succeeded; false otherwise.
     */
    virtual bool registRpcObject(IBase * pRpcObject, unsigned int &uintRpcObjectId) = 0;

    /**
     * @brief Retrieves an RPC object by its unique ID.
     *
     * @param uintRpcObjectId Unique ID of the RPC object to retrieve.
     * @return Pointer to the IBase interface of the RPC object if found; nullptr otherwise.
     */
    virtual IBase *getRpcObject(const unsigned int uintRpcObjectId) = 0;

    /**
     * @brief Unregisters an RPC object from the cluster.
     *
     * @param uintRpcObjectId Unique ID of the RPC object to unregister.
     * @return true if unregistration succeeded; false otherwise.
     */
    virtual bool unregistRpcObject(const unsigned int uintRpcObjectId) = 0;
};
