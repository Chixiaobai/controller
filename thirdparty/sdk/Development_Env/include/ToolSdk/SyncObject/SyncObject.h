#ifndef __TOOLSDK_SYNC_OBJECT_H__
#define __TOOLSDK_SYNC_OBJECT_H__

#include "ToolSdkDef.h"
#include "SyncObject/ISyncObject.h"
#include "OS/OS_Sync.h"
#include "OS/OS_Clock.h"

/**
 * @file SyncObject.h
 * @brief Header file for the CSyncObject class.
 *
 * This file contains the definition of the CSyncObject class, which provides a concrete implementation of the
 * IEventObject interface for event synchronization.
 */

DECLARE_STUB_INTERFACE(IEventObjectStub, IEventObject)

/**
 * @class CSyncObject
 * @brief Concrete implementation of the IEventObject interface.
 *
 * The CSyncObject class provides a concrete implementation of the IEventObject interface, using operating system
 * event mechanisms for synchronization. It supports both manual-reset and auto-reset events.
 */
class CSyncObject : public IEventObjectStub {
public:
    /**
     * @brief Constructor for CSyncObject.
     *
     * Initializes a new CSyncObject with the specified manual reset and initial state.
     *
     * @param bManualReset If true, the event remains signaled until it is explicitly reset. If false, the event
     *                     automatically resets after a single waiting thread is released.
     * @param bInitial If true, the event is initially in the signaled state. If false, the event is initially in the
     *                 non-signaled state.
     */
    CSyncObject(bool bManualReset, bool bInitial) { m_pEventHandle = os_event_create(bManualReset, bInitial); }

    /**
     * @brief Destructor for CSyncObject.
     *
     * Cleans up the CSyncObject by destroying the event handle.
     */
    virtual ~CSyncObject() { os_event_destroy(m_pEventHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CSyncObject objects to ensure proper management of resources.
     */
    CSyncObject(const CSyncObject &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CSyncObject objects to ensure proper management of resources.
     */
    CSyncObject &operator=(const CSyncObject &) = delete;
#endif

public:
    /**
     * @brief Sets the event.
     *
     * Sets the event, signaling any threads that are waiting on this event.
     */
    virtual void setEvent() { return os_event_set_event(m_pEventHandle); }

    /**
     * @brief Resets the event.
     *
     * Resets the event, making any threads that are waiting on this event continue to wait.
     */
    virtual void resetEvent() { return os_event_reset_event(m_pEventHandle); }

    /**
     * @brief Waits indefinitely for the event to be set.
     *
     * Blocks the calling thread until the event is set.
     *
     * @return true if the event was set, false if an error occurred.
     */
    virtual bool wait() { return os_event_wait(m_pEventHandle); }

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
    virtual bool wait(unsigned long tv_sec, long tv_nsec) {
        timespec timeout;
        os_clock_get_time(CLOCK_MONOTONIC, &timeout);
        timeout.tv_sec += tv_sec;
        timeout.tv_nsec += tv_nsec;
        os_clock_adjust_clock_time(&timeout);
        return os_event_wait_timeout(m_pEventHandle, timeout);
    }

protected:
    /**
     * @var event_handle *m_pEventHandle
     * @brief The handle to the operating system event.
     *
     * This handle is used to manage the event synchronization object.
     */
    event_handle *m_pEventHandle;
};

#endif