#pragma once

#include "Base/IBase.h"

/**
 * @interface IAppThread
 * @brief Interface representing an application thread.
 *
 * This interface provides access to the main application object associated with the thread.
 */
interface IAppObject;

interface IAppThread : public IBase {
    /**
     * @brief Retrieves the main application object associated with this thread.
     *
     * @return Pointer to the IAppObject instance representing the thread's main object.
     */
    virtual IAppObject *getMainObject() = 0;
};

/**
 * @interface IAppThreadCallback
 * @brief Callback interface for monitoring application thread lifecycle events.
 */
interface IAppThreadCallback : public IBase {
    /**
     * @brief Called when the application thread is initializing.
     *
     * @param pAppThread Pointer to the IAppThread instance being initialized.
     * @return true if initialization should proceed; false to cancel thread start.
     */
    virtual bool onAppThreadIniting(IAppThread * pAppThread) = 0;

    /**
     * @brief Called when the application thread is exiting.
     *
     * @param pAppThread Pointer to the IAppThread instance that is exiting.
     */
    virtual void onAppThreadExiting(IAppThread * pAppThread) = 0;
};

/// Enables runtime recognition of the IAppThread interface
ENABLE_RECOGNIZE_MACRO(IAppThread)

/// Enables runtime recognition of the IAppThreadCallback interface
ENABLE_RECOGNIZE_MACRO(IAppThreadCallback)
