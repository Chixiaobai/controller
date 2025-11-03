#ifndef __APPLE__

#ifndef __BASICSDK_OS_CONDITION_H__
#define __BASICSDK_OS_CONDITION_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"
#include "OS/OS_Mutex.h"

/**
 * @brief Enumeration for condition wait results
 */
enum CONDITION_WAIT_RESULT {
    WAIT_CONDITION_OK = 0,       /**< Wait completed successfully */
    WAIT_CONDITION_TIMEOUT = 1,  /**< Wait timed out */
    WAIT_CONDITION_FAILED = 2    /**< Wait failed */
};

struct mutex_handle;
struct condition_handle;

/**
 * @brief Create a condition variable
 * 
 * Note: Condition variables cannot be used between processes because it will lead to deadlock if some process crashes
 * while holding the mutex.
 * 
 * Note: It is advised that an application should not use mutexes with MUTEX_RECURSIVE, because the mutex may not
 * actually be released if it has been locked multiple times when calling os_condition_wait or os_condition_wait_timeout.
 * 
 * @param lck_handle Pointer to the mutex handle
 * @param mode Real-time mode (default is USE_DEFAULT)
 * @return condition_handle* Pointer to the created condition handle
 */
BASICSDK_API condition_handle *os_condition_create(mutex_handle *lck_handle, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Destroy a condition variable
 * 
 * @param handle Pointer to the condition handle to destroy
 */
BASICSDK_API void os_condition_destroy(condition_handle *&handle);

/**
 * @brief Notify one waiting thread
 * 
 * @param handle Pointer to the condition handle
 */
BASICSDK_API void os_condition_notify(condition_handle *handle);

/**
 * @brief Notify all waiting threads
 * 
 * @param handle Pointer to the condition handle
 */
BASICSDK_API void os_condition_notify_all(condition_handle *handle);

/**
 * @brief Wait indefinitely for the condition variable to be signaled
 * 
 * @param handle Pointer to the condition handle
 * @return int Result of the wait operation (WAIT_CONDITION_OK, WAIT_CONDITION_TIMEOUT, WAIT_CONDITION_FAILED)
 */
BASICSDK_API int os_condition_wait(condition_handle *handle);

/**
 * @brief Wait for the condition variable to be signaled with a timeout
 * 
 * @param handle Pointer to the condition handle
 * @param timeout Timespec structure specifying the timeout duration
 * @return int Result of the wait operation (WAIT_CONDITION_OK, WAIT_CONDITION_TIMEOUT, WAIT_CONDITION_FAILED)
 */
BASICSDK_API int os_condition_wait_timeout(condition_handle *handle, timespec timeout);

/**
 * @brief Namespace for condition variable-related functions and classes
 */
namespace OS {
class TCondition {
public:
    /**
     * @brief Constructor for TCondition
     * 
     * Note: Condition variables cannot be used between processes because it will lead to deadlock if some process crashes
     * while holding the mutex.
     * 
     * @param pMutex Pointer to the mutex (default is nullptr)
     * @param mode Real-time mode (default is USE_DEFAULT)
     */
    TCondition(OS::TBaseMutex *pMutex = nullptr, RT_MODE mode = USE_DEFAULT) {
        if (pMutex != nullptr) {
            m_ConditionHandle = os_condition_create(pMutex->handle(), mode);
        } else {
            m_ConditionHandle = nullptr;
        }
        m_intMode = mode;
    }

    /**
     * @brief Destructor for TCondition
     */
    ~TCondition() {
        if (m_ConditionHandle != nullptr) {
            os_condition_destroy(m_ConditionHandle);
            m_ConditionHandle = nullptr;
        }
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TCondition(const TCondition &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TCondition &operator=(const TCondition &) = delete;
#endif

public:
    /**
     * @brief Get the condition handle
     * 
     * @return condition_handle* Pointer to the condition handle
     */
    condition_handle *handle() { return m_ConditionHandle; }

    /**
     * @brief Bind a mutex to the condition variable
     * 
     * @param pMutex Pointer to the mutex
     * @return bool True if binding is successful, false otherwise
     */
    bool bindMutex(OS::TBaseMutex *pMutex) {
        if (m_ConditionHandle == nullptr && pMutex != nullptr) {
            m_ConditionHandle = os_condition_create(pMutex->handle(), m_intMode);
            return m_ConditionHandle != nullptr;
        } else {
            return false;
        }
    }

    /**
     * @brief Notify one waiting thread
     */
    void notify() { return os_condition_notify(m_ConditionHandle); }

    /**
     * @brief Notify all waiting threads
     */
    void notifyAll() { return os_condition_notify_all(m_ConditionHandle); }

    /**
     * @brief Wait indefinitely for the condition variable to be signaled
     * 
     * @return int Result of the wait operation (WAIT_CONDITION_OK, WAIT_CONDITION_TIMEOUT, WAIT_CONDITION_FAILED)
     */
    int wait() { return os_condition_wait(m_ConditionHandle); }

    /**
     * @brief Wait for the condition variable to be signaled with a timeout
     * 
     * @param timeout Timespec structure specifying the timeout duration
     * @return int Result of the wait operation (WAIT_CONDITION_OK, WAIT_CONDITION_TIMEOUT, WAIT_CONDITION_FAILED)
     */
    int wait(timespec timeout) { return os_condition_wait_timeout(m_ConditionHandle, timeout); }

protected:
    condition_handle *m_ConditionHandle;  /**< Pointer to the condition handle */
    RT_MODE m_intMode;                    /**< Real-time mode */
};
}  // namespace OS

#endif

#endif