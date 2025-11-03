#ifndef __BASICSDK_OS_EVENT_H__
#define __BASICSDK_OS_EVENT_H__

#ifndef __APPLE__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

struct event_handle;

/**
 * @brief Create an event object
 *
 * @param bManualReset If true, the event must be manually reset; if false, the event is automatically reset after a
 * single waiting thread is released
 * @param bInitial Initial state of the event (true for signaled, false for nonsignaled)
 * @param mode Real-time mode (default is USE_DEFAULT)
 * @param strEventName Name of the event (optional)
 * @return event_handle* Pointer to the created event handle
 */
BASICSDK_API event_handle *os_event_create(bool bManualReset, bool bInitial, RT_MODE mode = USE_DEFAULT,
                                           const char *strEventName = nullptr);

/**
 * @brief Destroy an event object
 *
 * @param handle Pointer to the event handle to destroy
 */
BASICSDK_API void os_event_destroy(event_handle *&handle);

/**
 * @brief Set the event to the signaled state
 *
 * @param handle Pointer to the event handle
 */
BASICSDK_API void os_event_set_event(event_handle *handle);

/**
 * @brief Reset the event to the nonsignaled state
 *
 * @param handle Pointer to the event handle
 */
BASICSDK_API void os_event_reset_event(event_handle *handle);

/**
 * @brief Wait indefinitely for the event to be signaled
 *
 * @param handle Pointer to the event handle
 * @return bool True if the event is signaled, false if the wait fails
 */
BASICSDK_API bool os_event_wait(event_handle *handle);

/**
 * @brief Wait for the event to be signaled with a timeout
 *
 * @param handle Pointer to the event handle
 * @param timeout Timespec structure specifying the timeout duration
 * @return bool True if the event is signaled, false if the wait times out or fails
 */
BASICSDK_API bool os_event_wait_timeout(event_handle *handle, timespec timeout);

/**
 * @brief Namespace for event-related functions and classes
 */
namespace OS {
class TEvent {
public:
    /**
     * @brief Constructor for TEvent
     *
     * @param bManualReset If true, the event must be manually reset; if false, the event is automatically reset after a
     * single waiting thread is released
     * @param bInitial Initial state of the event (true for signaled, false for nonsignaled)
     * @param mode Real-time mode (default is USE_DEFAULT)
     * @param strEventName Name of the event (optional)
     */
    TEvent(bool bManualReset, bool bInitial, RT_MODE mode = USE_DEFAULT, const char *strEventName = nullptr) {
        m_pEventHandle = os_event_create(bManualReset, bInitial, mode, strEventName);
    }

    /**
     * @brief Destructor for TEvent
     */
    ~TEvent() { os_event_destroy(m_pEventHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TEvent(const TEvent &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TEvent &operator=(const TEvent &) = delete;
#endif

public:
    /**
     * @brief Set the event to the signaled state
     */
    void setEvent() { return os_event_set_event(m_pEventHandle); }

    /**
     * @brief Reset the event to the nonsignaled state
     */
    void resetEvent() { return os_event_reset_event(m_pEventHandle); }

    /**
     * @brief Wait indefinitely for the event to be signaled
     *
     * @return bool True if the event is signaled, false if the wait fails
     */
    bool wait() { return os_event_wait(m_pEventHandle); }

    /**
     * @brief Wait for the event to be signaled with a timeout
     *
     * @param timeout Timespec structure specifying the timeout duration
     * @return bool True if the event is signaled, false if the wait times out or fails
     */
    bool wait(timespec timeout) { return os_event_wait_timeout(m_pEventHandle, timeout); }

protected:
    event_handle *m_pEventHandle; /**< Pointer to the event handle */
};
}  // namespace OS

#endif

#endif