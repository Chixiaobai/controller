#ifndef __TOOLSDK_I_SYNC_OBJECT_H__
#define __TOOLSDK_I_SYNC_OBJECT_H__

#include "Base/IBase.h"

/**
 * @file ISyncObject.h
 * @brief Header file for the IEventObject interface.
 *
 * This file contains the definition of the IEventObject interface, which provides methods for event synchronization.
 */

/**
 * @interface IEventObject
 * @brief Interface for event synchronization objects.
 *
 * The IEventObject interface defines methods for setting, resetting, and waiting on events. This interface is used
 * to provide a standardized way of handling event synchronization in the system.
 */
interface IEventObject : public IBase {
    /**
     * @brief Sets the event.
     *
     * Sets the event, signaling any threads that are waiting on this event.
     */
    virtual void setEvent() = 0;

    /**
     * @brief Resets the event.
     *
     * Resets the event, making any threads that are waiting on this event continue to wait.
     */
    virtual void resetEvent() = 0;

    /**
     * @brief Waits indefinitely for the event to be set.
     *
     * Blocks the calling thread until the event is set.
     *
     * @return true if the event was set, false if an error occurred.
     */
    virtual bool wait() = 0;

    /**
     * @brief Waits for the event to be set within a specified time period.
     *
     * Blocks the calling thread until the event is set or the specified time period elapses.
     *
     * @param tv_sec The number of seconds to wait.
     * @param tv_nsec The number of nanoseconds to wait.
     * @return true if the event was set within the specified time period, false if the time period elapsed or an error
     * occurred.
     */
    virtual bool wait(unsigned long tv_sec, long tv_nsec) = 0;
};

#endif