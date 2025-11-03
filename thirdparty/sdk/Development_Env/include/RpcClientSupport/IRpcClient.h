#pragma once

#include "IRpcClientNotifyCallback.h"

/**
 * @interface IRpcClient
 * @brief Interface for managing RPC client operations.
 *
 * This interface provides methods to retrieve RPC-related identifiers and helper objects,
 * as well as register notification callbacks for handling RPC events.
 */
interface IRemoteCallHelper;

interface IRpcClient : public IBase {
    /**
     * @brief Retrieves the servant ID associated with this RPC client.
     *
     * @return The servant ID.
     */
    virtual USHORT getRpcServantId() = 0;

    /**
     * @brief Retrieves the object ID associated with this RPC client.
     *
     * @return The object ID.
     */
    virtual UINT getRpcObjectId() = 0;

    /**
     * @brief Gets the remote call helper used to perform RPC invocations.
     *
     * @return Pointer to the IRemoteCallHelper instance.
     */
    virtual IRemoteCallHelper *getRemoteCallHelper() = 0;

    /**
     * @brief Sets the notification callback for RPC events.
     *
     * @param pRpcNotifyCallback Pointer to the IRpcClientNotifyCallback implementation.
     */
    virtual void setRpcNotifyCallback(IRpcClientNotifyCallback * pRpcNotifyCallback) = 0;
};

/// Enables runtime recognition of the IRpcClient interface
ENABLE_RECOGNIZE_MACRO(IRpcClient)