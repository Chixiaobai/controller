/**
 * @file Atomic.h
 * @brief Provides a template class for atomic operations on complex objects.
 *
 * This header file defines a template class `CAtomic` that performs atomic operations on complex objects.
 * Unlike `std::atomic`, this class uses a lock-free implementation suitable for complex objects.
 */

#ifndef __TOOLSDK_ATOMIC_H__
#define __TOOLSDK_ATOMIC_H__

#include "OS/OS_Thread.h"
#include "OS/OS_Clock.h"

/**
 * @brief Template class for atomic operations on complex objects.
 *
 * This class provides atomic operations for complex objects using a lock-free mechanism.
 * It supports loading and storing values atomically with optional spin limits.
 *
 * @tparam T The type of the object to be managed atomically.
 */
template <class T>
class CAtomic {
public:
    /**
     * @brief Constructor.
     *
     * Initializes the atomic object with the specified real-time mode.
     *
     * @param mode The real-time mode (default is USE_DEFAULT).
     */
    CAtomic(RT_MODE mode = USE_DEFAULT) {
        if (mode == USE_RT || (mode == USE_DEFAULT && get_default_rt_mode() == USE_RT)) {
            m_RtMode = USE_RT;
        } else {
            m_RtMode = USE_STD;
        }
        m_lModifiedFlag[0] = 0;
        m_lModifiedFlag[1] = 0;
        bzero(&m_MyVal, sizeof(m_MyVal));
    }

    /**
     * @brief Destructor.
     */
    virtual ~CAtomic() {}

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of the atomic object.
     */
    CAtomic(const CAtomic &) = delete;

    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of the atomic object.
     */
    CAtomic &operator=(const CAtomic &) = delete;
#endif

    /**
     * @brief Loads the value atomically.
     *
     * This method loads the value atomically. It should not be used in real-time threads.
     * Real-time threads should use `tryLoad` with appropriate parameters.
     *
     * @return The loaded value.
     */
    T load() {
        T val;
        tryLoad(val, -1);
        return val;
    }

    /**
     * @brief Stores the value atomically.
     *
     * This method stores the value atomically. It should not be used in real-time threads.
     * Real-time threads should use `tryStore` with appropriate parameters.
     *
     * @param val The value to be stored.
     */
    void store(const T &val) { tryStore(val, -1); }

    /**
     * @brief Attempts to load the value atomically with a spin limit.
     *
     * This method attempts to load the value atomically with an optional spin limit.
     *
     * @param val The variable to store the loaded value.
     * @param uintMaxSpinNumber The maximum number of spins (default is -1 for infinite spins).
     * @return True if the load is successful, false if the spin limit is exceeded.
     */
    bool tryLoad(T &val, const unsigned int uintMaxSpinNumber = -1) {
        bool bInfinite = (uintMaxSpinNumber == (unsigned int)-1 ? true : false);
        unsigned int uintRetryTimes = uintMaxSpinNumber + 1;
        T load_val;
        long flag1 = 0, flag2 = 0;
        bzero(&load_val, sizeof(load_val));
        do {
            flag1 = m_lModifiedFlag[0];
            flag2 = m_lModifiedFlag[1];
            if (flag1 != flag2) {
                uintRetryTimes--;
                continue;
            }
            load_val = m_MyVal;
            OS::Thread::barrier();
            if (m_lModifiedFlag[0] == flag1) {
                break;
            } else {
                uintRetryTimes--;
            }
        } while (bInfinite == true || uintRetryTimes != 0);
        if (bInfinite == false && uintRetryTimes == 0) {
            if (uintMaxSpinNumber != 0) {
                printf("Number of spins exceeds limit while loading data.\n");
            }
            return false;
        } else {
            val = load_val;
            return true;
        }
    }

    /**
     * @brief Attempts to store the value atomically with a spin limit.
     *
     * This method attempts to store the value atomically with an optional spin limit.
     *
     * @param val The value to be stored.
     * @param uintMaxSpinNumber The maximum number of spins (default is -1 for infinite spins).
     * @return True if the store is successful, false if the spin limit is exceeded.
     */
    bool tryStore(const T &val, const unsigned int uintMaxSpinNumber = -1) {
        bool bInfinite = (uintMaxSpinNumber == (unsigned int)-1 ? true : false);
        unsigned int uintRetryTimes = uintMaxSpinNumber + 1;
        long flag1 = 0, flag2 = 0;
        do {
            flag1 = m_lModifiedFlag[0];
            flag2 = m_lModifiedFlag[1];
            if (flag1 != flag2) {
                uintRetryTimes--;
                continue;
            }
            if (os_thread_sync_compare_and_swap(&m_lModifiedFlag[0], flag1, flag1 + 1)) {
                break;
            } else {
                uintRetryTimes--;
            }
        } while (bInfinite == true || uintRetryTimes != 0);
        if (bInfinite == false && uintRetryTimes == 0) {
            if (uintMaxSpinNumber != 0) {
                printf("Number of spins exceeds limit while storing data.\n");
            }
            return false;
        } else {
            m_MyVal = val;
            os_thread_sync_compare_and_swap(&m_lModifiedFlag[1], flag2, flag2 + 1);
            return true;
        }
    }

private:
    /**
     * @brief Flags to indicate modification status.
     */
    volatile long m_lModifiedFlag[2];

    /**
     * @brief Real-time mode.
     */
    RT_MODE m_RtMode;

    /**
     * @brief The value managed atomically.
     */
    T m_MyVal;
};

#endif