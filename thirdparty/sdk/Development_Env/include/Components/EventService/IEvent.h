#pragma once

#include "Base/IBase.h"

/**
 * @interface IEvent
 * @brief Interface representing a generic event.
 *
 * This interface provides methods to reset or initialize event states.
 */
interface IEvent : public IBase {
    /**
     * @brief Resets the event state.
     *
     * Implementations should use this method to reinitialize any internal state
     * or resources associated with the event.
     */
    virtual void resetEvent() = 0;
};

/**
 * @interface IEventCallback
 * @brief Callback interface for handling events.
 *
 * This interface defines the method that must be implemented to receive and process events.
 */
interface IEventCallback : public IBase {
    /**
     * @brief Called when an event occurs.
     *
     * @param event Pointer to the IEvent instance that triggered the callback.
     * @return true if the event was handled successfully; false otherwise.
     */
    virtual bool onEvent(IEvent * event) = 0;
};

/**
 * @interface IEventGenerator
 * @brief Interface for managing event callbacks and generating events.
 *
 * This interface provides methods to register, unregister, and notify event callbacks,
 * as well as reset the event generator's state.
 */
interface IEventGenerator : public IBase {
    /**
     * @brief Registers an event callback.
     *
     * @param callback Pointer to the IEventCallback implementation.
     * @return true if the callback was successfully added; false otherwise.
     */
    virtual bool addEventCallback(IEventCallback * callback) = 0;

    /**
     * @brief Unregisters an event callback.
     *
     * @param callback Pointer to the IEventCallback implementation.
     * @return true if the callback was successfully removed; false otherwise.
     */
    virtual bool removeEventCallback(IEventCallback * callback) = 0;

    /**
     * @brief Removes all registered event callbacks.
     */
    virtual void removeAllEventCallback() = 0;

    /**
     * @brief Resets the event generator state.
     *
     * Implementations should use this method to reinitialize any internal state
     * or resources associated with the event generator.
     */
    virtual void resetEventGenerator() = 0;
};

/// Enables runtime recognition of the IEvent interface
ENABLE_RECOGNIZE_MACRO(IEvent)

/// Enables runtime recognition of the IEventCallback interface
ENABLE_RECOGNIZE_MACRO(IEventCallback)

/// Enables runtime recognition of the IEventGenerator interface
ENABLE_RECOGNIZE_MACRO(IEventGenerator)
