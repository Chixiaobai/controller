#pragma once

#include "Base/IBase.h"

/**
 * @interface IAppTimerThreadCallback
 * @brief Callback interface for handling timer events in a thread.
 *
 * This interface defines the method that must be implemented to receive notifications
 * when a timer triggers.
 */
interface IAppTimerThreadCallback : public IBase {
    /**
     * @brief Called when a timer event occurs.
     *
     * @param hOwnerObject Handle to the object that owns the timer.
     * @param timerId Unique identifier of the timer.
     * @param lpTimerData Optional pointer to user-defined data associated with the timer.
     */
    virtual void onTimer(HANDLE hOwnerObject, UINT_PTR timerId, LPVOID lpTimerData) = 0;
};

/**
 * @interface IAppTimerThreadInfo
 * @brief Interface providing information about a timer thread.
 *
 * This interface allows access to the thread's hash code and associated callback handler.
 */
interface IAppTimerThreadInfo : public IBase {
    /**
     * @brief Retrieves a unique hash code representing the thread.
     *
     * @return The thread's hash code.
     */
    virtual UINT_PTR getHashCode() = 0;

    /**
     * @brief Retrieves the callback interface associated with this thread.
     *
     * @return Pointer to the IAppTimerThreadCallback implementation.
     */
    virtual IAppTimerThreadCallback *getCallback() = 0;
};

/**
 * @interface IAppTimerManager
 * @brief Interface for managing timers across threads.
 *
 * This interface provides methods to start and stop timers, as well as manage thread
 * participation in the application timer system.
 */
interface IAppTimerManager : public IBase {
    /**
     * @brief Registers the current thread with the timer manager.
     *
     * @param pThreadInfo Optional pointer to thread-specific timer information.
     * @return true if the thread was successfully registered; false otherwise.
     */
    virtual bool joinApplication(IAppTimerThreadInfo *pThreadInfo = nullptr) = 0;

    /**
     * @brief Unregisters the current thread from the timer manager.
     *
     * @param pThreadInfo Optional pointer to thread-specific timer information.
     */
    virtual void exit(IAppTimerThreadInfo *pThreadInfo = nullptr) = 0;

    /**
     * @brief Starts a new timer.
     *
     * @param hOwnerObject Handle to the object that owns the timer.
     * @param ulInterval Interval in milliseconds between timer events.
     * @param ulLagTime Maximum allowed deviation from the interval (for precision control).
     * @param mode Timer precision mode (low, normal, high).
     * @param lpData Optional pointer to user-defined data passed with timer events.
     * @param pThreadInfo Optional pointer to thread-specific timer information.
     * @return Unique timer identifier.
     */
    virtual UINT_PTR setTimer(HANDLE hOwnerObject, ULONG ulInterval, ULONG ulLagTime, TIMER_MODE mode,
                              LPVOID lpData = nullptr, IAppTimerThreadInfo *pThreadInfo = nullptr) = 0;

    /**
     * @brief Stops a running timer.
     *
     * @param hTimerID Identifier of the timer to stop.
     * @param pThreadInfo Optional pointer to thread-specific timer information.
     */
    virtual void killTimer(UINT_PTR hTimerID, IAppTimerThreadInfo *pThreadInfo = nullptr) = 0;
};

/// Enables runtime recognition of the IAppTimerThreadCallback interface
ENABLE_RECOGNIZE_MACRO(IAppTimerThreadCallback)

/// Enables runtime recognition of the IAppTimerThreadInfo interface
ENABLE_RECOGNIZE_MACRO(IAppTimerThreadInfo)

/// Enables runtime recognition of the IAppTimerManager interface
ENABLE_RECOGNIZE_MACRO(IAppTimerManager)
