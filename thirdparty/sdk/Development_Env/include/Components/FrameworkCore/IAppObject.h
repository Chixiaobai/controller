#pragma once

#include "Base/IBase.h"

/**
 * @file IAppObject.h
 * @brief Core application object interfaces and related definitions.
 *
 * This header defines interfaces for application objects, event handling,
 * timers, and inter-object communication within the framework.
 */

// Forward declaration for timer thread interface
interface IAppTimerThreadInfo;

/**
 * @enum TIMER_MODE
 * @brief Specifies the precision level of a timer.
 */
enum TIMER_MODE {
    LOW_PRECISION_TIMER = 0,    /**< Low-precision timer (least accurate) */
    NORMAL_PRECISION_TIMER = 1, /**< Normal-precision timer (default accuracy) */
    HIGH_PRECISION_TIMER = 2,   /**< High-precision timer (most accurate) */
};

/**
 * @enum OBJECT_NOTIFICATION
 * @brief Types of notifications that can be sent to object listeners.
 */
enum OBJECT_NOTIFICATION {
    APPEND_NOTIFICATION,    /**< Notification for adding an item */
    REMOVE_NOTIFICATION,    /**< Notification for removing an item */
    CLEAR_ALL_NOTIFICATION, /**< Notification for clearing all items */
};

/**
 * @interface IObjectListener
 * @brief Interface for receiving object change notifications.
 */
interface IObjectListener : public IBase {
    /**
     * @brief Called when the object state changes.
     *
     * @param uNotification Type of notification received.
     * @param wParam Optional parameter providing additional information.
     * @param lParam Optional parameter providing additional information.
     * @return true if the notification was handled; false otherwise.
     */
    virtual bool notifyChanged(OBJECT_NOTIFICATION uNotification, WPARAM wParam, LPARAM lParam) = 0;
};

/**
 * @interface IObject
 * @brief Base interface for observable objects that support listener registration.
 */
interface IObject : public IBase {
    /**
     * @brief Adds a listener to receive object change notifications.
     *
     * @param pListener Pointer to the IObjectListener implementation.
     */
    virtual void addListener(IObjectListener * pListener) = 0;

    /**
     * @brief Removes a previously registered listener.
     *
     * @param pListener Pointer to the IObjectListener implementation.
     */
    virtual void removeListener(IObjectListener * pListener) = 0;
};

/**
 * @interface IHandleObject
 * @brief Interface for retrieving a safe handle associated with an object.
 */
interface IHandleObject : public IBase {
    /**
     * @brief Retrieves a safe handle representing the object.
     *
     * @return The safe handle.
     */
    virtual HANDLE getSafeHandle() = 0;
};

// Forward declaration for AppObject callback interface
interface IAppObject;
interface IAppObjectCallback : public IBase {
    /**
     * @brief Handles custom messages dispatched to the object.
     *
     * @param pObject Pointer to the IAppObject instance receiving the message.
     * @param uMsg Message identifier.
     * @param wParam First message parameter.
     * @param lParam Second message parameter.
     * @return true if the message was handled; false otherwise.
     */
    virtual bool onMessage(IAppObject * pObject, UINT uMsg, WPARAM wParam, LPARAM lParam) = 0;

    /**
     * @brief Handles timer events.
     *
     * @param pObject Pointer to the IAppObject instance receiving the timer event.
     * @param uTimerId Timer identifier.
     * @return true if the timer event was handled; false otherwise.
     */
    virtual bool onTimer(IAppObject * pObject, UINT_PTR uTimerId) = 0;

    /**
     * @brief Handles key press or release events.
     *
     * @param pObject Pointer to the IAppObject instance receiving the key event.
     * @param uKey Key code.
     * @param bPressed true if the key is pressed, false if released.
     * @return true if the key event was handled; false otherwise.
     */
    virtual bool onKeyEvent(IAppObject * pObject, USHORT uKey, bool bPressed) = 0;
};

/**
 * @interface IAppObject
 * @brief Core application object interface supporting messaging, timers, and callbacks.
 */
interface IAppObject : public IBase {
    /**
     * @brief Sets the callback interface for handling messages and events.
     *
     * @param pAppObjectCallback Pointer to the IAppObjectCallback implementation.
     */
    virtual void setCallback(IAppObjectCallback * pAppObjectCallback) = 0;

    /**
     * @brief Starts a timer associated with this object.
     *
     * @param ulInterval Interval in milliseconds between timer events.
     * @param ulLagTime Maximum allowed deviation from the interval (for precision control).
     * @param mode Timer precision mode (low, normal, high).
     * @param lpData Optional pointer to user-defined data passed with timer events.
     * @param pTimerThreadInfo Optional pointer to a thread info object for dedicated timer threads.
     * @return Unique timer identifier.
     */
    virtual UINT_PTR setTimer(ULONG ulInterval, ULONG ulLagTime, TIMER_MODE mode, LPVOID lpData,
                              IAppTimerThreadInfo * pTimerThreadInfo) = 0;

    /**
     * @brief Stops a running timer.
     *
     * @param uTimerId Identifier of the timer to stop.
     */
    virtual void killTimer(UINT_PTR uTimerId) = 0;

    /**
     * @brief Sends a synchronous message to the object.
     *
     * @param uMsg Message identifier.
     * @param wParam First message parameter.
     * @param lParam Second message parameter.
     * @return true if the message was successfully processed; false otherwise.
     */
    virtual bool sendMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) = 0;

    /**
     * @brief Posts an asynchronous message to the object.
     *
     * @param uMsg Message identifier.
     * @param wParam First message parameter.
     * @param lParam Second message parameter.
     * @return true if the message was successfully posted; false otherwise.
     */
    virtual bool postMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) = 0;

    /**
     * @brief Sets the active state of the object.
     *
     * Implementations may use this flag to control internal processing.
     *
     * @param bActive true to activate the object; false to deactivate it.
     */
    virtual void setActive(bool bActive) = 0;
};

/// Enables runtime recognition of the IObjectListener interface
ENABLE_RECOGNIZE_MACRO(IObjectListener)

/// Enables runtime recognition of the IObject interface
ENABLE_RECOGNIZE_MACRO(IObject)

/// Enables runtime recognition of the IHandleObject interface
ENABLE_RECOGNIZE_MACRO(IHandleObject)

/// Enables runtime recognition of the IAppObjectCallback interface
ENABLE_RECOGNIZE_MACRO(IAppObjectCallback)

/// Enables runtime recognition of the IAppObject interface
ENABLE_RECOGNIZE_MACRO(IAppObject)
