#pragma once

#include "Base/IBase.h"

/**
 * @interface IApartmentCallback
 * @brief Interface for handling global apartment-level events.
 *
 * This interface defines callback functions that are triggered in response to apartment-wide
 * events such as timer expiration, heartbeat loss, or receipt of raw messages.
 */
interface IApartmentCallback : public IBase {
    /**
     * @brief Called when a timer associated with the apartment expires.
     * @param uintTimerId The ID of the expired timer.
     */
    virtual void onTimer(unsigned int uintTimerId) = 0;

    /**
     * @brief Called when a heartbeat (keep-alive signal) from a remote apartment is lost.
     * @param strRemoteApartmentUrl URL of the remote apartment that became unreachable.
     */
    virtual void onLostHeartbeat(const char *strRemoteApartmentUrl) = 0;

    /**
     * @brief Called when a raw message frame is received at the apartment level.
     * @param uintFrameType Type of the received frame.
     * @param strRawBuffer Pointer to the raw data buffer.
     * @param uintBufLen Length of the data buffer.
     */
    virtual void onRawMessage(const unsigned int uintFrameType, const void *strRawBuffer, unsigned int uintBufLen) = 0;
};

/**
 * @interface IApartmentNodeCallback
 * @brief Interface for handling apartment node-specific events.
 *
 * This interface defines callbacks specific to individual apartment nodes,
 * including timers, connection loss, and incoming requests.
 */
interface IApartmentNodeCallback : public IBase {
    /**
     * @brief Called when a timer associated with this apartment node expires.
     * @param uintTimerId The ID of the expired timer.
     */
    virtual void onTimer(unsigned int uintTimerId) = 0;

    /**
     * @brief Called when an apartment connected to this node goes offline.
     * @param strSourceApartmentUrl URL of the apartment that disconnected.
     */
    virtual void onApartmentOffline(const char *strSourceApartmentUrl) = 0;

    /**
     * @brief Called when a request message is received by this apartment node.
     * @param strSourceApartmentUrl URL of the source apartment.
     * @param strSourceNodeUrl URL of the source node within the source apartment.
     * @param strTargetApartmentUrl Expected target apartment URL.
     * @param strTargetNodeUrl Expected target node URL within the target apartment.
     * @param uintRequestId Unique identifier for correlating responses with this request.
     * @param pRequestBuffer Pointer to the request data buffer.
     * @param uintBufLen Length of the request data buffer.
     * @param bRequireResponse true if the sender expects a response; false otherwise.
     */
    virtual void onRequestReceived(const char *strSourceApartmentUrl, const char *strSourceNodeUrl,
                                   const char *strTargetApartmentUrl, const char *strTargetNodeUrl,
                                   unsigned int uintRequestId, const void *pRequestBuffer, unsigned int uintBufLen,
                                   bool bRequireResponse) = 0;
};

ENABLE_RECOGNIZE_MACRO(IApartmentCallback)
ENABLE_RECOGNIZE_MACRO(IApartmentNodeCallback)
