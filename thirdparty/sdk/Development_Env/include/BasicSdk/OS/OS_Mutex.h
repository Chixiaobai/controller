#ifndef __BASICSDK_OS_MUTEX_H__
#define __BASICSDK_OS_MUTEX_H__

#ifndef __APPLE__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Mutex flag indicating no special attributes
 */
constexpr unsigned int MUTEX_NONE = 0x00000000;

/**
 * @brief Mutex flag indicating recursive behavior
 */
constexpr unsigned int MUTEX_RECURSIVE = 0x00000001;

/**
 * @brief Mutex flag indicating priority inheritance
 */
constexpr unsigned int MUTEX_PRIO_INHERIT = 0x80000000;

#ifndef _WIN32
// Linux platform mutex defaults to non-recursive and no priority inheritance
constexpr unsigned int DEFAULT_MUTEX_ATTRIBUTE = 0;
#else
// Windows platform mutex is recursive by default and does not support priority inheritance
constexpr unsigned int DEFAULT_MUTEX_ATTRIBUTE = MUTEX_RECURSIVE;
#endif

struct mutex_handle;

/**
 * @brief Create a mutex
 *
 * @param uintFlag Mutex flags (MUTEX_NONE, MUTEX_RECURSIVE, MUTEX_PRIO_INHERIT)
 * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
 * @param strMutexName Name of the mutex (optional)
 * @return mutex_handle* Pointer to the created mutex handle
 */
BASICSDK_API mutex_handle *os_mutex_create(unsigned int uintFlag = DEFAULT_MUTEX_ATTRIBUTE, RT_MODE mode = USE_DEFAULT,
                                           const char *strMutexName = nullptr);

/**
 * @brief Destroy a mutex
 *
 * @param handle Pointer to the mutex handle to destroy
 */
BASICSDK_API void os_mutex_destroy(mutex_handle *&handle);

/**
 * @brief Lock a mutex
 *
 * @param handle Pointer to the mutex handle
 */
BASICSDK_API void os_mutex_lock(mutex_handle *handle);

/**
 * @brief Unlock a mutex
 *
 * @param handle Pointer to the mutex handle
 */
BASICSDK_API void os_mutex_unlock(mutex_handle *handle);

/**
 * @brief Namespace for mutex-related functions and classes
 */
namespace OS {
class TBaseMutex {
public:
    /**
     * @brief Constructor for TBaseMutex
     *
     * @param uintFlag Mutex flags (MUTEX_NONE, MUTEX_RECURSIVE, MUTEX_PRIO_INHERIT)
     * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
     * @param strMutexName Name of the mutex (optional)
     */
    TBaseMutex(unsigned int uintFlag = DEFAULT_MUTEX_ATTRIBUTE, RT_MODE mode = USE_DEFAULT,
               const char *strMutexName = nullptr) {
        m_pMutexHandle = os_mutex_create(uintFlag, mode, strMutexName);
    }

    /**
     * @brief Destructor for TBaseMutex
     */
    virtual ~TBaseMutex() { os_mutex_destroy(m_pMutexHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TBaseMutex(const TBaseMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TBaseMutex &operator=(const TBaseMutex &) = delete;
#endif

public:
    /**
     * @brief Get the mutex handle
     *
     * @return mutex_handle* Pointer to the mutex handle
     */
    mutex_handle *handle() { return m_pMutexHandle; }

    /**
     * @brief Lock the mutex
     */
    void lock() { return os_mutex_lock(m_pMutexHandle); }

    /**
     * @brief Unlock the mutex
     */
    void unlock() { return os_mutex_unlock(m_pMutexHandle); }

protected:
    mutex_handle *m_pMutexHandle; /**< Pointer to the mutex handle */
};

/**
 * @brief Recursive mutex class
 */
class TRecursiveMutex : public TBaseMutex {
public:
    /**
     * @brief Constructor for TRecursiveMutex
     *
     * @param strMutexName Name of the mutex (optional)
     */
    TRecursiveMutex(const char *strMutexName = nullptr) : TBaseMutex(MUTEX_RECURSIVE, USE_STD, strMutexName) {}

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Default destructor for TRecursiveMutex
     */
    virtual ~TRecursiveMutex() = default;

    /**
     * @brief Delete copy constructor to prevent copying
     */
    TRecursiveMutex(const TRecursiveMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TRecursiveMutex &operator=(const TRecursiveMutex &) = delete;
#endif
};

#ifdef _WIN32
/**
 * @brief Mutex alias for Windows platform
 */
#define TMutex TRecursiveMutex

/**
 * @brief Real-time mutex alias for Windows platform
 */
#define TRtMutex TRecursiveMutex
#else
/**
 * @brief Mutex class for Linux platform
 */
class TMutex : public TBaseMutex {
public:
    /**
     * @brief Constructor for TMutex
     *
     * @param strMutexName Name of the mutex (optional)
     */
    TMutex(const char *strMutexName = nullptr) : TBaseMutex(MUTEX_NONE, USE_STD, strMutexName) {}

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Default destructor for TMutex
     */
    virtual ~TMutex() = default;

    /**
     * @brief Delete copy constructor to prevent copying
     */
    TMutex(const TMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TMutex &operator=(const TMutex &) = delete;
#endif
};

/**
 * @brief Real-time mutex class for Linux platform
 */
class TRtMutex : public TBaseMutex {
public:
    /**
     * @brief Constructor for TRtMutex
     *
     * @param strMutexName Name of the mutex (optional)
     */
    TRtMutex(const char *strMutexName = nullptr) : TBaseMutex(MUTEX_PRIO_INHERIT, USE_RT, strMutexName) {}

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Default destructor for TRtMutex
     */
    virtual ~TRtMutex() = default;

    /**
     * @brief Delete copy constructor to prevent copying
     */
    TRtMutex(const TRtMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TRtMutex &operator=(const TRtMutex &) = delete;
#endif
};
#endif
}  // namespace OS

#else

#include <mutex>
namespace OS {
/**
 * @brief Recursive mutex type for Apple platform
 */
typedef ::std::recursive_mutex TRecursiveMutex;
}  // namespace OS

#endif

/**
 * @brief Template class for automatic mutex locking and unlocking
 *
 * @tparam T Mutex type
 */
template <class T>
class TAutoMutex {
public:
    /**
     * @brief Constructor for TAutoMutex
     *
     * @param pMutex Pointer to the mutex
     */
    TAutoMutex(T *pMutex) {
        m_pMutex = pMutex;
        if (m_pMutex != nullptr) m_pMutex->lock();
    }

    /**
     * @brief Destructor for TAutoMutex
     */
    ~TAutoMutex() {
        if (m_pMutex != nullptr) m_pMutex->unlock();
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    TAutoMutex(const TAutoMutex &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    TAutoMutex &operator=(const TAutoMutex &) = delete;
#endif

protected:
    T *m_pMutex; /**< Pointer to the mutex */
};

#endif