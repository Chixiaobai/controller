#pragma once

#include "Base/IBase.h"

/**
 * @enum KEY_EVENT_TYPE
 * @brief Types of key events supported by the system.
 */
enum KEY_EVENT_TYPE {
    NONE_KEY_EVENT = 0,      /**< No valid key event */
    KEY_UP_EVENT = 1,        /**< Key release event */
    KEY_DOWN_EVENT = 2,      /**< Key press event */
    KEY_CLICKED_EVENT = 3,   /**< Key click event (press and release combination) */
};

/**
 * @enum KEY_FLAG
 * @brief Flags indicating key state transitions.
 */
enum KEY_FLAG {
    KEY_UP_FLAG = 0x00,   /**< Key is released */
    KEY_DOWN_FLAG = 0x01, /**< Key is pressed */
};

/**
 * @interface IKeyEvent
 * @brief Interface representing a key event.
 *
 * This interface provides methods to query key event properties such as key ID,
 * event type, and whether it's a repeated event.
 */
interface IKeyEvent : public IBase {
    /**
     * @brief Retrieves the identifier of the key associated with this event.
     *
     * @return The key ID.
     */
    virtual int getKeyId() = 0;

    /**
     * @brief Checks if this event is a repeated key press.
     *
     * @param uintRepeatCount Reference to an unsigned integer that receives the repeat count.
     * @return true if this is a repeat event; false otherwise.
     */
    virtual bool isRepeatEvent(unsigned int &uintRepeatCount) = 0;

    /**
     * @brief Retrieves the type of this key event.
     *
     * @return The KEY_EVENT_TYPE value indicating the event type.
     */
    virtual KEY_EVENT_TYPE getKeyEventType() = 0;
};

/**
 * @interface IKeyEventGenerator
 * @brief Interface for generating key events.
 *
 * This interface provides methods to simulate or inject key events into the system.
 */
interface IKeyEventGenerator : public IBase {
    /**
     * @brief Generates a key event.
     *
     * @param intKeyId The identifier of the key to generate the event for.
     * @param keyFlag The KEY_FLAG indicating whether the key is pressed or released.
     */
    virtual void generate(int intKeyId, KEY_FLAG keyFlag) = 0;
};

/// Enables runtime recognition of the IKeyEvent interface
ENABLE_RECOGNIZE_MACRO(IKeyEvent)

/// Enables runtime recognition of the IKeyEventGenerator interface
ENABLE_RECOGNIZE_MACRO(IKeyEventGenerator)
