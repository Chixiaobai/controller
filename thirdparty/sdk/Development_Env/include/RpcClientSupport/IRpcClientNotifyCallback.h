#pragma once

#include "Base/IBase.h"

/**
 * @interface IRpcClientNotifyCallback
 * @brief Interface for receiving RPC notification events.
 *
 * This interface defines a callback method to handle incoming RPC notifications,
 * including interface ID, method ID, and serialized input data.
 */
interface IRpcClientNotifyCallback : public IBase {
    /**
     * @brief Handles an incoming RPC notification.
     *
     * @param uiInterfaceId The interface ID of the notification.
     * @param uiMethodId The method ID of the notification.
     * @param inputStream Pointer to the serialized input data.
     * @param inputSize Size of the input data in bytes.
     */
    virtual void onNotify(unsigned short uiInterfaceId, unsigned short uiMethodId, const unsigned char *inputStream,
                          unsigned short inputSize) = 0;
};

/// Enables runtime recognition of the IRpcClientNotifyCallback interface
ENABLE_RECOGNIZE_MACRO(IRpcClientNotifyCallback)

/**
 * @brief Stub interface declaration for IRpcClientNotifyCallback.
 *
 * This macro creates a stub implementation (IRpcClientNotifyCallbackStub) for use in
 * remote communication or testing scenarios.
 */
DECLARE_STUB_INTERFACE(IRpcClientNotifyCallbackStub, IRpcClientNotifyCallback)
