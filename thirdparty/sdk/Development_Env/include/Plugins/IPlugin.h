#pragma once

#include "Base/IBase.h"

/**
 * @interface IPlugin
 * @brief Interface class for all plugins.
 *
 * This interface inherits from IBase and provides a pure virtual method to retrieve
 * the plugin's entry name, which is used during plugin registration and identification.
 */
interface IPlugin : public IBase {
    /**
     * @brief Retrieves the entry name of the plugin.
     *
     * Implementations must provide this method to return the plugin's entry name.
     *
     * @param pName Pointer to a buffer where the entry name will be copied.
     * @param dwNameLen Pointer to a DWORD indicating the size of the buffer on input,
     *                  and receives the actual length of the name on output.
     * @return true if the name was successfully retrieved; false otherwise.
     */
    virtual bool getEntryName(char *pName, LPDWORD dwNameLen) = 0;
};

/// @brief Macro to enable runtime recognition of the IPlugin interface
ENABLE_RECOGNIZE_MACRO(IPlugin)

/**
 * @brief Function pointer type for plugin creation functions.
 *
 * Plugins should export a function of this type named 'createPlugin' to allow
 * dynamic instantiation by the plugin loader.
 */
typedef IPlugin *(*CREATE_PLUGIN_FUNC)();
