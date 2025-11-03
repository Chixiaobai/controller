#pragma once

#include "IComponent.h"
#include "IComponentLoaderCallback.h"
#include "NameEnum/INameEnum.h"

/**
 * @interface IComponentSupport
 * @brief Interface for managing component support operations.
 * @extends IBase
 */
interface IComponentSupport : public IBase {
    /**
     * @brief Sets the component workspace.
     * @param strComponentWorkspace Path to the component workspace.
     */
    virtual void setComponentWorkspace(const char *strComponentWorkspace) = 0;

    /**
     * @brief Loads the component manifest file and creates component instances based on the manifest.
     * @param strComponentManifestFile Path to the component manifest file.
     * @param callback Callback for component loading operations.
     */
    virtual void loadComponentManifest(const char *strComponentManifestFile, IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Registers a component folder for loading.
     * @param strComponentFolderPath Path to the component folder.
     */
    virtual void registComponentFolder(const char *strComponentFolderPath) = 0;

    /**
     * @brief Registers a component file.
     * @param strComponentFile Path to the component file.
     * @param strPostFix Optional postfix for the component file.
     * @return True if the component file is registered successfully, false otherwise.
     */
    virtual bool registComponentFile(const char *strComponentFile, const char *strPostFix = nullptr) = 0;

    /**
     * @brief Registers a component class.
     * @param strComponentClassFile Path to the component class file.
     * @param strComponentClassDescFile Path to the component class description file.
     * @return True if the component class is registered successfully, false otherwise.
     */
    virtual bool registComponentClass(const char *strComponentClassFile, const char *strComponentClassDescFile) = 0;

    /**
     * @brief Unregisters a component class and destroys all instances of the component class.
     * @param strComponentClassName Name of the component class.
     * @param callback Callback for component unloading operations.
     * @return True if the component class is unregistered successfully, false otherwise.
     */
    virtual bool unregistComponentClass(const char *strComponentClassName, IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Unregisters all component classes and destroys all component instances.
     * @param callback Callback for component unloading operations.
     */
    virtual void unregistAllComponentClass(IComponentLoaderCallback * callback) = 0;

    /**
     * @brief Creates a component instance.
     * @param strComponentClassName Name of the component class.
     * @param strComponentName Name of the component instance.
     * @param callback Callback for component creation operations.
     * @return True if the component is created successfully, false otherwise.
     */
    virtual bool createComponent(const char *strComponentClassName, const char *strComponentName,
                                 IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Finds a component instance.
     * @param strComponentClassName Name of the component class.
     * @param strComponentName Name of the component instance.
     * @return Pointer to the component instance if found, nullptr otherwise.
     */
    virtual IComponent *findComponent(const char *strComponentClassName, const char *strComponentName) = 0;

    /**
     * @brief Creates a unique component instance (only for single-instance component classes).
     * @param strComponentClassName Name of the component class.
     * @param callback Callback for component creation operations.
     * @return True if the unique component is created successfully, false otherwise.
     */
    virtual bool createUniqueComponent(const char *strComponentClassName, IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Finds a unique component instance (only for single-instance component classes).
     * @param strComponentClassName Name of the component class.
     * @return Pointer to the unique component instance if found, nullptr otherwise.
     */
    virtual IComponent *findUniqueComponent(const char *strComponentClassName) = 0;

    /**
     * @brief Destroys a component instance.
     * @param strComponentClassName Name of the component class.
     * @param strComponentName Name of the component instance.
     * @param callback Callback for component destruction operations.
     */
    virtual void destroyComponent(const char *strComponentClassName, const char *strComponentName,
                                  IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Destroys a unique component instance (only for single-instance component classes).
     * @param strComponentClassName Name of the component class.
     * @param callback Callback for component destruction operations.
     */
    virtual void destroyUniqueComponent(const char *strComponentClassName, IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Destroys all instances of a component class.
     * @param strComponentClassName Name of the component class.
     * @param callback Callback for component destruction operations.
     */
    virtual void destroyAllComponent(const char *strComponentClassName, IComponentLoaderCallback *callback) = 0;

    /**
     * @brief Forces the component to update all resources it is using.
     */
    virtual void updateComponentResource() = 0;

    /**
     * @brief Checks if a component is currently loaded.
     * @param strComponentClassName Name of the component class.
     * @return True if the component is loaded, false otherwise.
     */
    virtual bool isComponentLoaded(const char *strComponentClassName) = 0;

    /**
     * @brief Checks if a component is designed as a singleton.
     * @param strComponentClassName Name of the component class.
     * @return True if the component is a singleton, false otherwise.
     */
    virtual bool isComponentSington(const char *strComponentClassName) = 0;

    /**
     * @brief Enumerates all components that the specified component depends on.
     * @param strComponentClassName Name of the component class.
     * @return Pointer to an INameEnum object containing the names of the dependent components.
     */
    virtual INameEnum *enumerateDependentComponents(const char *strComponentClassName) = 0;

    /**
     * @brief Sorts a given list of components based on their actual loading sequence.
     * @param pNameEnum Pointer to an INameEnum object containing the names of the components to be sorted.
     * @return Pointer to an INameEnum object containing the sorted names of the components.
     */
    virtual INameEnum *getComponentsLoadingSeq(INameEnum * pNameEnum) = 0;
};
