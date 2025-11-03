#ifndef __BASICSDK_OS_SEMAPHORE_H__
#define __BASICSDK_OS_SEMAPHORE_H__

#ifndef __APPLE__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

struct semaphore_handle;

/**
 * @brief Create a semaphore
 *
 * @param intInitialCount Initial count of the semaphore
 * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
 * @param strSemaphoreName Name of the semaphore (optional)
 * @return semaphore_handle* Pointer to the created semaphore handle
 */
BASICSDK_API semaphore_handle *os_semaphore_create(int intInitialCount, RT_MODE mode = USE_DEFAULT,
                                                   const char *strSemaphoreName = nullptr);

/**
 * @brief Destroy a semaphore
 *
 * @param handle Pointer to the semaphore handle to destroy
 */
BASICSDK_API void os_semaphore_destroy(semaphore_handle *&handle);

/**
 * @brief Set undo tracking for a semaphore
 *
 * @param handle Pointer to the semaphore handle
 * @param bUndoTrack If true, enable undo tracking; if false, disable undo tracking
 * @return bool True if successful, false otherwise
 */
BASICSDK_API bool os_semaphore_set_undo_track(semaphore_handle *handle, bool bUndoTrack);

/**
 * @brief Wait indefinitely for a semaphore
 *
 * @param handle Pointer to the semaphore handle
 * @return bool True if the semaphore is acquired, false otherwise
 */
BASICSDK_API bool os_semaphore_wait(semaphore_handle *handle);

/**
 * @brief Wait for a semaphore with a timeout
 *
 * @param handle Pointer to the semaphore handle
 * @param timeout Timespec structure specifying the timeout duration
 * @return bool True if the semaphore is acquired, false if the wait times out
 */
BASICSDK_API bool os_semaphore_wait_timeout(semaphore_handle *handle, timespec timeout);

/**
 * @brief Release a semaphore
 *
 * @param handle Pointer to the semaphore handle
 */
BASICSDK_API void os_semaphore_release(semaphore_handle *handle);

/**
 * @brief Namespace for semaphore-related functions and classes
 */
namespace OS {
class TSemaphore {
public:
    /**
     * @brief Constructor for TSemaphore
     *
     * @param intInitialCount Initial count of the semaphore
     * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
     * @param strSemaphoreName Name of the semaphore (optional)
     */
    TSemaphore(int intInitialCount, RT_MODE mode = USE_DEFAULT, const char *strSemaphoreName = nullptr) {
        m_pSemaphoreHandle = os_semaphore_create(intInitialCount, mode, strSemaphoreName);
    }

    /**
     * @brief Destructor for TSemaphore
     */
    virtual ~TSemaphore() { os_semaphore_destroy(m_pSemaphoreHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TSemaphore(const TSemaphore &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TSemaphore &operator=(const TSemaphore &) = delete;
#endif

public:
    /**
     * @brief Get the semaphore handle
     *
     * @return semaphore_handle* Pointer to the semaphore handle
     */
    semaphore_handle *handle() { return m_pSemaphoreHandle; }

    /**
     * @brief Set undo tracking for the semaphore
     *
     * @param bUndoTrack If true, enable undo tracking; if false, disable undo tracking
     * @return bool True if successful, false otherwise
     */
    bool setUndoTrack(bool bUndoTrack) { return os_semaphore_set_undo_track(m_pSemaphoreHandle, bUndoTrack); }

    /**
     * @brief Wait indefinitely for the semaphore
     *
     * @return bool True if the semaphore is acquired, false otherwise
     */
    bool wait() { return os_semaphore_wait(m_pSemaphoreHandle); }

    /**
     * @brief Wait for the semaphore with a timeout
     *
     * @param timeout Timespec structure specifying the timeout duration
     * @return bool True if the semaphore is acquired, false if the wait times out
     */
    bool wait(timespec timeout) { return os_semaphore_wait_timeout(m_pSemaphoreHandle, timeout); }

    /**
     * @brief Release the semaphore
     */
    void release() { os_semaphore_release(m_pSemaphoreHandle); }

protected:
    semaphore_handle *m_pSemaphoreHandle; /**< Pointer to the semaphore handle */
};
}  // namespace OS

#endif

#endif