/**
 * @file ICallbackPointContainer.h
 * @brief Defines the ICallbackPointContainer interface for managing callback points.
 *
 * This header file defines the `ICallbackPointContainer` interface, which provides a method to find
 * callback points by their interface ID. It extends the `IBase` interface.
 */

#ifndef __TOOLSDK_I_CALLBACK_POINT_CONTAINER_H__
#define __TOOLSDK_I_CALLBACK_POINT_CONTAINER_H__

#include "Base/ICallbackPoint.h"

/**
 * @brief Interface for managing callback points.
 *
 * This interface provides a method to find callback points by their interface ID. It extends the `IBase` interface.
 */
interface ICallbackPointContainer : public IBase {
    /**
     * @brief Finds a callback point by interface ID.
     *
     * This method searches for a callback point with the specified interface ID and returns a pointer to it.
     *
     * @param uiInterfaceId The interface ID of the callback point to find.
     * @return Pointer to the `ICallbackPoint` object if found, otherwise nullptr.
     */
    virtual ICallbackPoint *findCallbackPoint(unsigned int uiInterfaceId) = 0;
};

/**
 * @brief Macro to enable recognition of the ICallbackPointContainer interface.
 */
ENABLE_RECOGNIZE_MACRO(ICallbackPointContainer)

#endif