#pragma once

#include "Base/IBase.h"
#include "IComponent.h"

/**
 * @interface IResourceContext
 * @brief Interface for managing resource context.
 * @extends IBase
 */
interface IResourceContext : public IBase {
    /**
     * @brief Initializes the resource context.
     * @return True if the resource context is initialized successfully, false otherwise.
     */
    virtual bool initResourceContext() = 0;
};

/**
 * @interface IComponentLookup
 * @brief Interface for looking up components.
 * @extends IBase
 */
interface IComponentLookup : public IBase {
    /**
     * @brief Finds a component by its class name and optionally by its name.
     * @param strComponentClassName Name of the component class.
     * @param strComponentName Name of the component instance. If nullptr, it will look for a unique component.
     * @return Pointer to the IComponent object if found, nullptr otherwise.
     */
    virtual IComponent *findComponent(const char *strComponentClassName, const char *strComponentName) = 0;
};

ENABLE_RECOGNIZE_MACRO(IResourceContext)
ENABLE_RECOGNIZE_MACRO(IComponentLookup)
