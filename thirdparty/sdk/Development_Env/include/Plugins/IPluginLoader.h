#pragma once

#include "IPlugin.h"
#include "IPluginLoaderCallback.h"
#include "NameEnum/INameEnum.h"

/**
 * @interface IPluginLoader
 * @brief Interface for managing plugin lifecycle and manifest creation.
 *
 * This interface provides methods to create, find, and destroy plugin instances,
 * as well as manage plugin manifests. It supports both single-instance and
 * multi-instance plugin models and allows thread-safe operations where specified.
 */
interface IPluginLoader : public IBase {
    /**
     * @brief Creates a plugin manifest from a manifest file.
     *
     * @param strPluginManifestFile Path to the plugin manifest file.
     * @param strPostFix Optional postfix used to filter or modify manifest entries.
     * @param callback Callback interface for handling plugin loading events.
     * @return Pointer to an INameEnum object containing plugin names, or nullptr on failure.
     */
    virtual INameEnum *createPluginManifest(const char *strPluginManifestFile, const char *strPostFix,
                                            IPluginLoaderCallback *callback) = 0;

    /**
     * @brief Creates a plugin instance.
     *
     * @param strPluginClassName Name of the plugin class to instantiate.
     * @param strPostFix Optional postfix used to customize the plugin instance.
     * @param strPluginName Name to assign to this plugin instance.
     * @param callback Callback interface for handling plugin creation events.
     * @return true if the plugin was successfully created; false otherwise.
     */
    virtual bool createPlugin(const char *strPluginClassName, const char *strPostFix, const char *strPluginName,
                              IPluginLoaderCallback *callback) = 0;

    /**
     * @brief Finds a plugin instance by class and name.
     *
     * @param strPluginClassName Name of the plugin class.
     * @param strPluginName Name of the plugin instance.
     * @return Pointer to the found IPlugin instance, or nullptr if not found.
     */
    virtual IPlugin *findPlugin(const char *strPluginClassName, const char *strPluginName) = 0;

    /**
     * @brief Creates a unique plugin instance (single-instance mode only).
     *
     * @param strPluginClassName Name of the plugin class to instantiate.
     * @param strPostFix Optional postfix used to customize the plugin instance.
     * @param callback Callback interface for handling plugin creation events.
     * @return true if the unique plugin was created successfully; false otherwise.
     */
    virtual bool createUniquePlugin(
        const char *strPluginClassName, const char *strPostFix,
        IPluginLoaderCallback *callback) = 0;

    /**
     * @brief Finds a unique plugin instance by class name (thread-safe).
     *
     * This is the only method in this interface that can be safely called across threads.
     *
     * @param strPluginClassName Name of the plugin class.
     * @return Pointer to the found IPlugin instance, or nullptr if not found.
     */
    virtual IPlugin *findUniquePlugin(
        const char *strPluginClassName) = 0;

    /**
     * @brief Destroys a specific plugin instance.
     *
     * @param strPluginClassName Name of the plugin class.
     * @param strPluginName Name of the plugin instance to destroy.
     * @param callback Callback interface for handling plugin destruction events.
     * @return true if the plugin was successfully destroyed; false otherwise.
     */
    virtual bool destroyPlugin(const char *strPluginClassName, const char *strPluginName,
                               IPluginLoaderCallback *callback) = 0;

    /**
     * @brief Destroys a unique plugin instance (single-instance mode only).
     *
     * @param strPluginClassName Name of the plugin class.
     * @param callback Callback interface for handling plugin destruction events.
     * @return true if the unique plugin was destroyed successfully; false otherwise.
     */
    virtual bool destroyUniquePlugin(
        const char *strPluginClassName,
        IPluginLoaderCallback *callback) = 0;

    /**
     * @brief Destroys all instances of a given plugin class.
     *
     * @param strPluginClassName Name of the plugin class.
     * @param callback Callback interface for handling plugin destruction events.
     * @return true if all instances were successfully destroyed; false otherwise.
     */
    virtual bool destroyAllPlugin(const char *strPluginClassName,
                                  IPluginLoaderCallback *callback) = 0;

    /**
     * @brief Forces destruction of all loaded plugins.
     *
     * This method should be used to ensure all plugin resources are released.
     *
     * @param callback Callback interface for handling plugin destruction events.
     */
    virtual void destroyAllPlugin(IPluginLoaderCallback * callback) = 0;
};
