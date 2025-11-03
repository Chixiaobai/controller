#ifndef __TOOLSDK_CACHE_H__
#define __TOOLSDK_CACHE_H__

#include "OS/OS_Sync.h"

/// Default cache size if not specified.
#define DEFAULT_CACHE_SIZE 128

// This class must not be used across real-time threads and non-real-time threads
// because its internal implementation relies on mutexes and semaphores, which may
// not be allowed in such operations (e.g., in Xenomai).

/**
 * @class CCache
 * @brief Template class for a cache implementation.
 *
 * This class provides a thread-safe cache implementation that can store objects of type T.
 * It uses mutexes and events for synchronization to ensure safe access across multiple threads.
 */
template <class T>
class CCache {
public:
    /**
     * @brief Constructor for CCache.
     *
     * Initializes the cache with a specified size and optional real-time mode.
     *
     * @param lCacheSize The size of the cache (default is DEFAULT_CACHE_SIZE).
     * @param mode The real-time mode (default is USE_DEFAULT).
     */
    CCache(long lCacheSize, RT_MODE mode = USE_DEFAULT)
        : m_CacheMutex(MUTEX_RECURSIVE, mode, nullptr), m_CacheNotEmptyEvent(false, false, mode) {
        m_lCacheSize = (lCacheSize == 0 ? DEFAULT_CACHE_SIZE : lCacheSize);
        m_pCache = new T[m_lCacheSize];
        m_lReadOffset = -1;
        m_lWriteOffset = -1;
        m_bInterruptPop = false;
    }

    /**
     * @brief Destructor for CCache.
     *
     * Cleans up the cache by deleting the allocated memory.
     */
    virtual ~CCache() {
        if (m_pCache != nullptr) {
            delete[] m_pCache;
            m_pCache = nullptr;
        }
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CCache objects.
     */
    CCache(const CCache &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CCache objects.
     */
    CCache &operator=(const CCache &) = delete;
#endif

public:
    /**
     * @brief Resets the cache.
     *
     * Clears the cache and reinitializes it with the current cache size.
     */
    void reset() {
        if (m_pCache != nullptr) {
            delete[] m_pCache;
            m_pCache = nullptr;
        }
        m_lReadOffset = -1;
        m_lWriteOffset = -1;
        m_bInterruptPop = false;
        m_pCache = new T[m_lCacheSize];
    }

    /**
     * @brief Pushes an item into the cache.
     *
     * Adds an item to the cache. If the cache is full, it fails.
     *
     * @param data The item to be pushed into the cache.
     * @return true if the item was successfully pushed, false otherwise.
     */
    bool push(const T &data) { return push_ex(data, true); }

    /**
     * @brief Pushes an item into the cache with overflow handling.
     *
     * Adds an item to the cache. If the cache is full and bFailedIfOverFlow is false,
     * it attempts to pop an item and then push the new item.
     *
     * @param data The item to be pushed into the cache.
     * @param bFailedIfOverFlow If true, the operation fails if the cache is full.
     * @return true if the item was successfully pushed, false otherwise.
     */
    bool push_ex(const T &data, bool bFailedIfOverFlow) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_CacheMutex);
        bool ret = push_p(data);
        if (bFailedIfOverFlow == false && ret == false) {
            T rmData;
            if (tryPop(rmData) == true) ret = push_p(data);
        }
        return ret;
    }

    /**
     * @brief Tries to pop an item from the cache without blocking.
     *
     * Attempts to remove an item from the cache. If the cache is empty, it returns false.
     *
     * @param data The item to be popped from the cache.
     * @return true if an item was successfully popped, false otherwise.
     */
    bool tryPop(T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_CacheMutex);
        if (m_lReadOffset == -1)
            return false;
        else {
            data = m_pCache[m_lReadOffset];
            m_pCache[m_lReadOffset] = m_NullObject;
#ifdef _DEBUG
            T *CachePt = mCachePtList.front();
            mCachePtList.pop_front();
            if (CachePt != (m_pCache + m_lReadOffset)) {
                printf("Cache has error.Application maybe crash!\n");
            }
#endif
            m_lReadOffset = (m_lReadOffset + 1) % m_lCacheSize;
            if (m_lReadOffset == m_lWriteOffset) {
                m_lReadOffset = -1;
                m_lWriteOffset = -1;
            }
            return true;
        }
    }

    /**
     * @brief Pops an item from the cache with an optional timeout.
     *
     * Removes an item from the cache. If the cache is empty, it waits until an item is available
     * or the timeout expires.
     *
     * @param data The item to be popped from the cache.
     * @param intTimeout The timeout in milliseconds (default is -1, which means wait indefinitely).
     * @return true if an item was successfully popped, false otherwise.
     */
    bool pop(T &data) { return pop(data, -1); }

    /**
     * @brief Pops an item from the cache with a specified timeout.
     *
     * Removes an item from the cache. If the cache is empty, it waits until an item is available
     * or the timeout expires.
     *
     * @param data The item to be popped from the cache.
     * @param intTimeout The timeout in milliseconds.
     * @return true if an item was successfully popped, false otherwise.
     */
    bool pop(T &data, int intTimeout) {
        timespec timeout;
        if (intTimeout >= 0) {
            OS::Clock::getTime(CLOCK_MONOTONIC, &timeout);
            timeout.tv_sec += (intTimeout / 1000);
            timeout.tv_nsec += (intTimeout % 1000) * 1000000;
            OS::Clock::adjustClockTime(&timeout);
        } else {
            timeout.tv_sec = -1;
            timeout.tv_nsec = -1;
        }
        return pop(data, timeout);
    }

    /**
     * @brief Pops an item from the cache with a specified timeout.
     *
     * Removes an item from the cache. If the cache is empty, it waits until an item is available
     * or the timeout expires.
     *
     * @param data The item to be popped from the cache.
     * @param timeout The timeout as a timespec structure.
     * @return true if an item was successfully popped, false otherwise.
     */
    bool pop(T &data, timespec timeout) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_CacheMutex);
        m_bInterruptPop = false;
        while (m_lReadOffset == -1) {
            if (m_bInterruptPop == true) return false;
            m_CacheMutex.unlock();
            if (timeout.tv_sec == -1 && timeout.tv_nsec == -1) {
                if (m_CacheNotEmptyEvent.wait() == false) {
                    return false;
                }
            } else {
                if (m_CacheNotEmptyEvent.wait(timeout) == false) {
                    return false;
                }
            }
            m_CacheMutex.lock();
        }
        data = m_pCache[m_lReadOffset];
        m_pCache[m_lReadOffset] = m_NullObject;
#ifdef _DEBUG
        T *CachePt = mCachePtList.front();
        mCachePtList.pop_front();
        if (CachePt != (m_pCache + m_lReadOffset)) {
            printf("Cache has error.Application maybe crash!\n");
        }
#endif
        m_lReadOffset = (m_lReadOffset + 1) % m_lCacheSize;
        if (m_lReadOffset == m_lWriteOffset) {
            m_lReadOffset = -1;
            m_lWriteOffset = -1;
        }
        return true;
    }

    /**
     * @brief Interrupts a pending pop operation.
     *
     * Signals the cache not empty event to interrupt any waiting pop operations.
     */
    void interruptPop() {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_CacheMutex);
        m_bInterruptPop = true;
        m_CacheNotEmptyEvent.setEvent();
    }

    /**
     * @brief Gets the number of items in the cache.
     *
     * Returns the current number of items stored in the cache.
     *
     * @return The number of items in the cache.
     */
    long getItemCount() {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_CacheMutex);
        if (m_lReadOffset > m_lWriteOffset) {
            return m_lWriteOffset + m_lCacheSize - m_lReadOffset;
        } else {
            if (m_lReadOffset == -1 && m_lWriteOffset == -1) {
                return 0;
            } else if (m_lReadOffset == m_lWriteOffset) {
                return m_lCacheSize;
            } else {
                return m_lWriteOffset - m_lReadOffset;
            }
        }
    }

    /**
     * @brief Gets an item from the cache by index.
     *
     * Returns the item at the specified index in the cache. If the index is out of range,
     * it returns a null object.
     *
     * @param index The index of the item to retrieve.
     * @return The item at the specified index, or a null object if the index is out of range.
     */
    T getItem(const long index) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_CacheMutex);
        long lMaxIndex = 0;
        if (m_lReadOffset > m_lWriteOffset) {
            lMaxIndex = m_lWriteOffset + m_lCacheSize - m_lReadOffset;
        } else {
            if (m_lReadOffset == -1 && m_lWriteOffset == -1) {
                lMaxIndex = 0;
            } else if (m_lReadOffset == m_lWriteOffset) {
                lMaxIndex = m_lCacheSize;
            } else {
                lMaxIndex = m_lWriteOffset - m_lReadOffset;
            }
        }
        if (index < lMaxIndex) {
            return m_pCache[(m_lReadOffset + index) % m_lCacheSize];
        } else {
            return m_NullObject;
        }
    }

private:
    /**
     * @brief Pushes an item into the cache.
     *
     * Adds an item to the cache. This is a private helper function used by push and push_ex.
     *
     * @param data The item to be pushed into the cache.
     * @return true if the item was successfully pushed, false otherwise.
     */
    bool push_p(const T &data) {
        if (m_lWriteOffset == -1) {
            m_pCache[0] = data;
#ifdef _DEBUG
            mCachePtList.push_back(m_pCache);
#endif
            m_lWriteOffset = 1 % m_lCacheSize;
            m_lReadOffset = 0;
            m_CacheNotEmptyEvent.setEvent();
            return true;
        } else {
            if (m_lReadOffset == m_lWriteOffset)
                return false;
            else {
                m_pCache[m_lWriteOffset] = data;
#ifdef _DEBUG
                mCachePtList.push_back(m_pCache + m_lWriteOffset);
#endif
                m_lWriteOffset = (m_lWriteOffset + 1) % m_lCacheSize;
                m_CacheNotEmptyEvent.setEvent();
                return true;
            }
        }
    }

protected:
    /// Flag to indicate if a pop operation should be interrupted.
    bool m_bInterruptPop;

    /// The read offset in the cache.
    long m_lReadOffset;

    /// The write offset in the cache.
    long m_lWriteOffset;

    /// The null object used to clear cache slots.
    T m_NullObject;

    /// Pointer to the cache array.
    T *m_pCache;

    /// The size of the cache.
    long m_lCacheSize;

    /// Mutex for synchronizing access to the cache.
    OS::TBaseMutex m_CacheMutex;

    /// Event to signal when the cache is not empty.
    OS::TEvent m_CacheNotEmptyEvent;

#ifdef _DEBUG
    /// List of pointers to cache items for debugging purposes.
    list<T *> mCachePtList;
#endif
};

#endif