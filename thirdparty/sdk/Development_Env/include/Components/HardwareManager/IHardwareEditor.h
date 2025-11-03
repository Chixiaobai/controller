#pragma once

#include "Base/IBase.h"

/**
 * @interface IHardwareEditor
 * @brief Interface for modifying hardware properties.
 *
 * This interface provides methods to set configuration properties for hardware devices.
 */
interface IHardwareEditor : public IBase {
    /**
     * @brief Sets a hardware property value.
     *
     * @param strPropertyName Name of the property to set.
     * @param strPropertyValue New value for the property.
     * @return true if the property was successfully set; false otherwise.
     */
    virtual bool setProperty(const char *strPropertyName, const char *strPropertyValue) = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareEditor)
