#pragma once

#include "Base/IBase.h"

/**
 * @interface IComponentLoaderCallback
 * @brief Interface for callbacks related to component loading, unloading, creation, and destruction.
 * @extends IBase
 */
interface IComponentLoaderCallback : public IBase {
    /**
     * @brief Callback method called when a component is loaded.
     * @param strComponentClassName Name of the component class that was loaded.
     */
    virtual void onComponentLoad(const char *strComponentClassName) = 0;

    /**
     * @brief Callback method called when a component is unloaded.
     * @param strComponentClassName Name of the component class that was unloaded.
     */
    virtual void onComponentUnload(const char *strComponentClassName) = 0;

    /**
     * @brief Callback method called when a component is created.
     * @param strComponentClassName Name of the component class that was created.
     * @param strComponentName Name of the component instance that was created.
     */
    virtual void onComponentCreate(const char *strComponentClassName, const char *strComponentName) = 0;

    /**
     * @brief Callback method called when a component is destroyed.
     * @param strComponentClassName Name of the component class that was destroyed.
     * @param strComponentName Name of the component instance that was destroyed.
     */
    virtual void onComponentDestroy(const char *strComponentClassName, const char *strComponentName) = 0;
};
