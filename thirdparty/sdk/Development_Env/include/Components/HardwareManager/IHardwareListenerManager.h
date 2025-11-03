#pragma once

#include "IHardwareListener.h"

/**
 * @interface IHardwareListenerManager
 * @brief Interface for managing hardware event listeners.
 *
 * This interface provides methods to register, unregister, and notify hardware listeners
 * about interface state changes.
 */
interface IHardwareListenerManager : public IBase {
    /**
     * @brief Registers a hardware listener.
     *
     * @param pHardwareListener Pointer to the IHardwareListener implementation.
     */
    virtual void addListener(IHardwareListener * pHardwareListener) = 0;

    /**
     * @brief Unregisters a hardware listener.
     *
     * @param pHardwareListener Pointer to the IHardwareListener implementation.
     */
    virtual void removeListener(IHardwareListener * pHardwareListener) = 0;

    /**
     * @brief Removes all registered hardware listeners.
     */
    virtual void removeAllListener() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareListenerManager)
