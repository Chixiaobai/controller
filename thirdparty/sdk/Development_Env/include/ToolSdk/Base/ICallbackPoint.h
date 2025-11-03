/**
 * @file ICallbackPoint.h
 * @brief Defines the ICallbackPoint interface for callback management.
 *
 * This header file defines the `ICallbackPoint` interface, which provides methods for managing callbacks.
 * It includes methods to get the interface ID, advise a callback sink, and unadvise a callback sink.
 */

#ifndef __TOOLSDK_I_CALLBACK_POINT_H__
#define __TOOLSDK_I_CALLBACK_POINT_H__

#include "Base/IBase.h"

/**
 * @brief Interface for managing callbacks.
 *
 * This interface provides methods to manage callback sinks, allowing objects to register and unregister
 * for callbacks. It extends the `IBase` interface.
 */
interface ICallbackPoint : public IBase {
    /**
     * @brief Gets the interface ID.
     *
     * This method returns the unique identifier for the interface.
     *
     * @return The interface ID.
     */
    virtual unsigned short getInterfaceId() = 0;

    /**
     * @brief Advises a callback sink.
     *
     * This method registers a callback sink with the object. The sink is identified by a cookie ID.
     *
     * @param pBaseSink Pointer to the callback sink object.
     * @param uintCookieId Pointer to a variable that will receive the cookie ID for the sink.
     * @return True if the sink is successfully advised, false otherwise.
     */
    virtual bool advise(IBase * pBaseSink, unsigned int *uintCookieId) = 0;

    /**
     * @brief Unadvises a callback sink.
     *
     * This method unregisters a callback sink using its cookie ID.
     *
     * @param uintCookieId The cookie ID of the sink to be unadvised.
     */
    virtual void unadvise(unsigned int uintCookieId) = 0;
};

/**
 * @brief Macro to enable recognition of the ICallbackPoint interface.
 */
ENABLE_RECOGNIZE_MACRO(ICallbackPoint)

#endif