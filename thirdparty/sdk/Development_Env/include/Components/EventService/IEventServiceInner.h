#pragma once

/**
 * @def UNIQUE_EVENTSERVICE_NAME
 * @brief Unique identifier name for the EventService component.
 *
 * This macro defines the unique name used to identify the EventService component
 * within the system, useful for logging, debugging, and service lookup.
 */
#define UNIQUE_EVENTSERVICE_NAME "EventService"

#include "IEvent.h"

/**
 * @enum EVENT_GENERATOR_TYPE
 * @brief Types of event generators supported by the event service.
 */
enum EVENT_GENERATOR_TYPE {
    KEY_EVENT_GENERATOR = 1, /**< Generator for keyboard-related events */
};

/**
 * @interface IEventServiceInner
 * @brief Interface for accessing and managing event generators within the event service.
 *
 * This interface provides methods to retrieve specific types of event generators
 * that can be used to subscribe to and receive various kinds of events.
 */
interface IEventServiceInner : public IBase {
    /**
     * @brief Retrieves an event generator of the specified type.
     *
     * @param type The EVENT_GENERATOR_TYPE indicating which generator to retrieve.
     * @return Pointer to the IEventGenerator instance if available; nullptr otherwise.
     */
    virtual IEventGenerator *getEventGenerator(EVENT_GENERATOR_TYPE type) = 0;
};

/// Enables runtime recognition of the IEventServiceInner interface
ENABLE_RECOGNIZE_MACRO(IEventServiceInner)
