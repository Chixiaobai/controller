#pragma once

#include "Base/IBase.h"

/**
 * @interface IHardwareLink
 * @brief Interface for monitoring and managing hardware link status.
 *
 * This interface provides methods to check if a hardware link is broken and
 * to notify about link breakage events.
 */
interface IHardwareLink : public IBase {
    /**
     * @brief Checks whether the hardware link is currently broken.
     *
     * @return true if the link is broken; false otherwise.
     */
    virtual bool isLinkBroken() = 0;

    /**
     * @brief Notifies that the hardware link has been broken.
     */
    virtual void brokenLink() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareLink)
