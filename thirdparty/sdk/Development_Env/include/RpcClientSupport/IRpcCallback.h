#pragma once

#include "Base/IBase.h"

/**
 * @interface IRpcCallback
 * @brief Interface for receiving RPC-related events.
 *
 * This interface defines callback methods that are invoked by the RPC system to notify
 * the client about disconnection events.
 */
interface IRpcCallback : public IBase {
    /**
     * @brief Called when the RPC connection is disconnected.
     *
     * Implementations should handle cleanup or notification logic upon disconnection.
     */
    virtual void onDisconnect() = 0;
};
