#pragma once

#define UNIQUE_FRAMEWORKCORE_NAME "FrameworkCore"

#include "IAppThread.h"
#include "IAppObject.h"
#include "IAppTimer.h"
#include "MsgDef.h"

interface IEventObject;

/**
 * @interface IEventObject
 * @brief Forward declaration for event-capable object interface.
 *
 * This serves as a placeholder for interfaces that support event handling
 * within the framework core.
 */

/**
 * @interface IFrameworkCoreInner
 * @brief Core framework management interface.
 *
 * This interface defines methods for initializing and managing the application framework,
 * including thread and object creation, as well as timer management.
 */
interface IFrameworkCoreInner : public IBase {
    /**
     * @brief Starts the framework core with the provided application callback.
     *
     * @param pAppCallback Pointer to the IAppThreadCallback implementation.
     * @return An integer status code indicating the result of the operation.
     */
    virtual int startFrameCore(IAppThreadCallback * pAppCallback) = 0;

    /**
     * @brief Creates a new application thread.
     *
     * @param pAppCallback Pointer to the IAppThreadCallback implementation.
     * @param osThreadHandle Optional pointer to receive the OS-specific thread handle.
     * @return Pointer to the newly created IAppThread instance.
     */
    virtual IAppThread *createAppThread(IAppThreadCallback * pAppCallback, OS_THREAD_HANDLE * osThreadHandle) = 0;

    /**
     * @brief Creates a new application object associated with the specified thread.
     *
     * @param pThread Pointer to the IAppThread that will own the object.
     * @return Pointer to the newly created IAppObject instance.
     */
    virtual IAppObject *createAppObject(IAppThread * pThread) = 0;

    /**
     * @brief Finds an object by its handle.
     *
     * @param hObjectHandle Handle to the object to find.
     * @return Pointer to the IHandleObject if found; otherwise nullptr.
     */
    virtual IHandleObject *findObject(HANDLE hObjectHandle) = 0;

    /**
     * @brief Retrieves the global timer manager instance.
     *
     * @return Pointer to the IAppTimerManager for managing timers.
     */
    virtual IAppTimerManager *getAppTimerManager() = 0;
};

#ifndef FrameworkCore_EXPORTS
/**
 * @brief Stub interface declaration for IAppObjectCallback.
 *
 * Used for remote communication or testing purposes when not building the FrameworkCore library.
 */
DECLARE_STUB_INTERFACE(IAppObjectCallbackStub, IAppObjectCallback)

/**
 * @brief Stub interface declaration for IAppThreadCallback.
 *
 * Used for remote communication or testing purposes when not building the FrameworkCore library.
 */
DECLARE_STUB_INTERFACE(IAppThreadCallbackStub, IAppThreadCallback)
#endif

/// Enables runtime recognition of the IFrameworkCoreInner interface
ENABLE_RECOGNIZE_MACRO(IFrameworkCoreInner)
