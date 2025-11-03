#pragma once

#include "Base/IBase.h"

/**
 * @interface IPluginLoaderCallback
 * @brief Callback interface for tracking plugin loading, unloading, creation, and destruction events.
 *
 * This interface provides methods that are invoked by the plugin loader to notify clients
 * about significant lifecycle events of plugins.
 */
interface IPluginLoaderCallback : public IBase {
    /**
     * @brief Called when a plugin is successfully loaded into the system.
     *
     * @param strPluginEntryName The entry name of the loaded plugin.
     */
    virtual void onPluginLoad(const char *strPluginEntryName) = 0;

    /**
     * @brief Called when a plugin is unloaded from the system.
     *
     * @param strPluginEntryName The entry name of the unloaded plugin.
     */
    virtual void onPluginUnload(const char *strPluginEntryName) = 0;

    /**
     * @brief Called when a new instance of a plugin is created.
     *
     * @param strPluginEntryName The entry name of the plugin class.
     * @param strPluginName The name assigned to the newly created plugin instance.
     */
    virtual void onPluginCreate(const char *strPluginEntryName, const char *strPluginName) = 0;

    /**
     * @brief Called when a plugin instance is destroyed.
     *
     * @param strPluginEntryName The entry name of the plugin class.
     * @param strPluginName The name of the destroyed plugin instance.
     */
    virtual void onPluginDestroy(const char *strPluginEntryName, const char *strPluginName) = 0;
};
