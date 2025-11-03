#ifndef __APPLE__

#ifndef __BASICSDK_OS_RWMUTEX_H__
#define __BASICSDK_OS_RWMUTEX_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

struct rw_mutex_handle;

/**
 * @brief Create a read-write mutex
 *
 * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
 * @param strMutexName Name of the mutex (optional)
 * @return rw_mutex_handle* Pointer to the created read-write mutex handle
 */
BASICSDK_API rw_mutex_handle *os_rw_mutex_create(RT_MODE mode = USE_DEFAULT, const char *strMutexName = nullptr);

/**
 * @brief Destroy a read-write mutex
 *
 * @param handle Pointer to the read-write mutex handle to destroy
 */
BASICSDK_API void os_rw_mutex_destroy(rw_mutex_handle *&handle);

/**
 * @brief Lock a read-write mutex for reading
 *
 * @param handle Pointer to the read-write mutex handle
 */
BASICSDK_API void os_rw_mutex_lock_rd(rw_mutex_handle *handle);

/**
 * @brief Lock a read-write mutex for writing
 *
 * @param handle Pointer to the read-write mutex handle
 */
BASICSDK_API void os_rw_mutex_lock_wr(rw_mutex_handle *handle);

/**
 * @brief Unlock a read-write mutex
 *
 * @param handle Pointer to the read-write mutex handle
 */
BASICSDK_API void os_rw_mutex_unlock(rw_mutex_handle *handle);

/**
 * @brief Namespace for read-write mutex-related functions and classes
 */
namespace OS {
class TRWMutex {
public:
    /**
     * @brief Constructor for TRWMutex
     *
     * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
     * @param strMutexName Name of the mutex (optional)
     */
    TRWMutex(RT_MODE mode = USE_DEFAULT, const char *strMutexName = nullptr) {
        m_pRWMutexHandle = os_rw_mutex_create(mode, strMutexName);
    }

    /**
     * @brief Destructor for TRWMutex
     */
    virtual ~TRWMutex() { os_rw_mutex_destroy(m_pRWMutexHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TRWMutex(const TRWMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TRWMutex &operator=(const TRWMutex &) = delete;
#endif

public:
    /**
     * @brief Get the read-write mutex handle
     *
     * @return rw_mutex_handle* Pointer to the read-write mutex handle
     */
    rw_mutex_handle *handle() { return m_pRWMutexHandle; }

    /**
     * @brief Lock the read-write mutex for reading
     */
    void lock_rd() { return os_rw_mutex_lock_rd(m_pRWMutexHandle); }

    /**
     * @brief Lock the read-write mutex for writing
     */
    void lock_wr() { return os_rw_mutex_lock_wr(m_pRWMutexHandle); }

    /**
     * @brief Unlock the read-write mutex
     */
    void unlock() { return os_rw_mutex_unlock(m_pRWMutexHandle); }

protected:
    rw_mutex_handle *m_pRWMutexHandle; /**< Pointer to the read-write mutex handle */
};
}  // namespace OS

/**
 * @brief Template class for automatic read-write mutex locking and unlocking
 */
class TAutoRWMutex {
public:
    /**
     * @brief Constructor for TAutoRWMutex
     *
     * @param pMutex Pointer to the read-write mutex
     * @param bUsedForRead If true, lock for reading; if false, lock for writing
     */
    TAutoRWMutex(OS::TRWMutex *pMutex, bool bUsedForRead = true) {
        m_pMutex = pMutex;
        if (m_pMutex != nullptr) {
            if (bUsedForRead == true) {
                m_pMutex->lock_rd();
            } else {
                m_pMutex->lock_wr();
            }
        }
    }

    /**
     * @brief Destructor for TAutoRWMutex
     */
    ~TAutoRWMutex() {
        if (m_pMutex != nullptr) m_pMutex->unlock();
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TAutoRWMutex(const TAutoRWMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TAutoRWMutex &operator=(const TAutoRWMutex &) = delete;
#endif

protected:
    OS::TRWMutex *m_pMutex; /**< Pointer to the read-write mutex */
};

#endif

#endif