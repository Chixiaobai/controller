#pragma once

#include "Base/IBase.h"

/**
 * @interface IHardwareListener
 * @brief Interface for receiving hardware interface state change notifications.
 *
 * This interface defines callback methods that are invoked when the hardware interface
 * is opened or closed.
 */
interface IHardwareListener : public IBase {
    /**
     * @brief Called when the hardware interface has been opened.
     *
     * @param bOpenOK true if the interface was successfully opened; false otherwise.
     */
    virtual void onInterfaceOpen(bool bOpenOK) = 0;

    /**
     * @brief Called when the hardware interface has been closed.
     */
    virtual void onInterfaceClose() = 0;
};
ENABLE_RECOGNIZE_MACRO(IHardwareListener)
