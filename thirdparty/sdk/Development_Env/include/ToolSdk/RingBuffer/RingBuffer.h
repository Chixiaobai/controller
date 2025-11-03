/**
 * @file RingBuffer.h
 * @brief Header file for ring buffer management.
 *
 * This file defines a template class for managing a ring buffer with synchronization mechanisms.
 * The ring buffer can be used in multi-threaded and multi-process environments and supports
 * inter-process communication when a shared memory name is provided.
 */

#ifndef __TOOLSDK_RINGBUFFER_H__
#define __TOOLSDK_RINGBUFFER_H__

#include "OS/OS_Clock.h"
#include "OS/OS_Thread.h"
#include "OS/OS_ShareMemory.h"

#define DEFAULT_RING_BUFFER_SIZE 128

/**
 * @class CRingBuffer
 * @brief Template class for managing a ring buffer.
 *
 * This class provides a lock-free ring buffer implementation that can be used in multi-threaded
 * and multi-process environments. It supports inter-process communication when a shared memory
 * name is provided. The ring buffer does not call constructors or destructors for elements of type T.
 * In real-time environments, this class is suitable for single-writer, multiple-reader scenarios.
 * Multiple writers can cause performance degradation due to internal sleep operations.
 *
 * @tparam T The type of elements stored in the ring buffer.
 * @tparam _SupportSizeMethod (C++11 only) Whether to support the size() method.
 */
#ifdef STD_CPLUSPLUS_11
template <class T, bool _SupportSizeMethod = true>
#else
template <class T>
#endif
class CRingBuffer {
    /**
     * @struct _RING_BUFFER_CB
     * @brief Control block structure for the ring buffer.
     *
     * This structure contains control information for the ring buffer, including indices and sizes.
     */
    typedef struct _RING_BUFFER_CB {
        volatile unsigned long m_lUsedCount;  // Number of elements in the buffer
        volatile long m_lBufferSize;          // Maximum number of elements the buffer can hold
        volatile long m_lWriteIndex;          // Current write index
        volatile long m_lReadIndex;           // Current read index
        volatile long m_lMaximumReadIndex;    // Maximum read index
        int32_t m_intItemSize;                // Size of each element
        int32_t m_intBufferTag;               // Type tag for the ring buffer
#ifdef _WIN32
        int32_t m_intAlignTo16Bytes;  // Alignment for 16 bytes (Windows specific)
#endif
    } RING_BUFFER_CB;

public:
    /**
     * @brief Constructor for CRingBuffer with specified buffer size and optional shared memory name.
     *
     * Initializes the ring buffer with the specified buffer size. If a shared memory name is provided,
     * the buffer is created in shared memory for inter-process communication.
     *
     * @param lBufferSize The size of the buffer.
     * @param strShmName The name of the shared memory (optional).
     */
    CRingBuffer(long lBufferSize, const char *strShmName = nullptr) {
        m_pBufferData = nullptr;
        m_hShareMemory = nullptr;
        m_bMemoryOwner = false;
        m_pRingBufferCB = nullptr;
        m_pCacheData = nullptr;
        createBuffer(lBufferSize, strShmName);
    }

    /**
     * @brief Constructor for CRingBuffer with optional shared memory name.
     *
     * Initializes the ring buffer using an existing shared memory buffer. If a shared memory name is
     * provided, the buffer is opened for inter-process communication.
     *
     * @param strShmName The name of the shared memory (optional).
     */
    CRingBuffer(const char *strShmName = nullptr) {
        m_pBufferData = nullptr;
        m_hShareMemory = nullptr;
        m_bMemoryOwner = false;
        m_pRingBufferCB = nullptr;
        m_pCacheData = nullptr;
        openBuffer(strShmName);
    }

    /**
     * @brief Destructor for CRingBuffer.
     *
     * Destroys the ring buffer and frees all associated resources. If the buffer is the owner of the
     * memory, it destroys the buffer; otherwise, it closes the buffer.
     */
    virtual ~CRingBuffer() {
        if (m_bMemoryOwner) {
            destroyBuffer();
        } else {
            closeBuffer();
        }
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CRingBuffer objects to ensure proper management of resources.
     */
    CRingBuffer(const CRingBuffer &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CRingBuffer objects to ensure proper management of resources.
     */
    CRingBuffer &operator=(const CRingBuffer &) = delete;
#endif

    /**
     * @brief Pushes an element into the ring buffer.
     *
     * Adds an element to the ring buffer. If the buffer is full, the operation fails.
     *
     * @param data The element to be pushed into the buffer.
     * @param mode The real-time mode for synchronization operations (default is USE_DEFAULT).
     * @return true if the element was successfully pushed, false if the buffer is full.
     */
    bool push(const T &data, RT_MODE mode = USE_DEFAULT) {
        if (m_pBufferData == nullptr) {
            return false;
        }
        long currentWriteIndex = 0, currentReadIndex = 0;
        // Get the position to write
        do {
            currentWriteIndex = m_pRingBufferCB->m_lWriteIndex;
            currentReadIndex = m_pRingBufferCB->m_lReadIndex;
            if (modIndex(currentWriteIndex + 1, m_pRingBufferCB->m_lBufferSize) ==
                modIndex(currentReadIndex, m_pRingBufferCB->m_lBufferSize)) {
                return false;  // Buffer is full
            }
        } while (!os_thread_sync_compare_and_swap(&m_pRingBufferCB->m_lWriteIndex, currentWriteIndex,
                                                  currentWriteIndex + 1));
        // Write data to the buffer
        m_pCacheData[modIndex(currentWriteIndex, m_pRingBufferCB->m_lBufferSize)] = data;
        while (!os_thread_sync_compare_and_swap(&m_pRingBufferCB->m_lMaximumReadIndex, currentWriteIndex,
                                                currentWriteIndex + 1)) {
            if (mode == USE_STD || (mode == USE_DEFAULT && get_default_rt_mode() == USE_STD)) {
                // Yield the thread in non-real-time mode
                os_thread_yield(mode);
            } else {
                // Sleep briefly in real-time mode to avoid deadlock
                timespec nextTime;
                os_clock_get_time(CLOCK_MONOTONIC, &nextTime, USE_RT);
                nextTime.tv_nsec += 100000;  // Sleep for 100us
                os_clock_adjust_clock_time(&nextTime);
                os_clock_nano_sleep(&nextTime, USE_RT);
            }
        }
#ifdef STD_CPLUSPLUS_11
        if (_SupportSizeMethod == true) {
            os_thread_sync_fetch_add(&m_pRingBufferCB->m_lUsedCount);
        }
#else
        os_thread_sync_fetch_add(&m_pRingBufferCB->m_lUsedCount);
#endif
        return true;
    }

    /**
     * @brief Pops an element from the ring buffer.
     *
     * Removes and returns an element from the ring buffer. If the buffer is empty, the operation fails.
     *
     * @param data Reference to the variable where the popped element will be stored.
     * @return true if an element was successfully popped, false if the buffer is empty.
     */
    bool pop(T &data) {
        if (m_pBufferData == nullptr) {
            return false;
        }
        long currentMaximumReadIndex = 0, currentReadIndex = 0;
        do {
            currentReadIndex = m_pRingBufferCB->m_lReadIndex;
            currentMaximumReadIndex = m_pRingBufferCB->m_lMaximumReadIndex;
            if (modIndex(currentReadIndex, m_pRingBufferCB->m_lBufferSize) ==
                modIndex(currentMaximumReadIndex, m_pRingBufferCB->m_lBufferSize)) {
                // Buffer is empty
                return false;
            }
            data = m_pCacheData[modIndex(currentReadIndex, m_pRingBufferCB->m_lBufferSize)];
            if (os_thread_sync_compare_and_swap(&m_pRingBufferCB->m_lReadIndex, currentReadIndex,
                                                currentReadIndex + 1)) {
#ifdef STD_CPLUSPLUS_11
                if (_SupportSizeMethod == true) {
                    os_thread_sync_fetch_sub(&m_pRingBufferCB->m_lUsedCount);
                }
#else
                os_thread_sync_fetch_sub(&m_pRingBufferCB->m_lUsedCount);
#endif
                return true;
            }
        } while (true);
        return false;  // This line is never reached
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Retrieves the number of elements in the ring buffer.
     *
     * Returns the number of elements currently in the ring buffer. This method is only available if
     * _SupportSizeMethod is true.
     *
     * @return The number of elements in the buffer.
     */
    template <bool B = _SupportSizeMethod>
    typename std::enable_if<B, long>::type size() {
        if (m_pRingBufferCB == nullptr) {
            return 0;
        }
        return m_pRingBufferCB->m_lUsedCount;
    }
#else
    /**
     * @brief Retrieves the number of elements in the ring buffer.
     *
     * Returns the number of elements currently in the ring buffer.
     *
     * @return The number of elements in the buffer.
     */
    inline long size() {
        if (m_pRingBufferCB == nullptr) {
            return 0;
        }
        return m_pRingBufferCB->m_lUsedCount;
    }
#endif

private:
    /**
     * @brief Returns the current class tag.
     *
     * Returns a unique tag for the class based on the _SupportSizeMethod template parameter.
     *
     * @return The class tag.
     */
    int currentClassTag() {
        if (_SupportSizeMethod == true) {
            return 20231020;
        } else {
            return -20231020;
        }
    }

    /**
     * @brief Calculates a suitable buffer size.
     *
     * Adjusts the buffer size to the nearest power of two to optimize performance.
     *
     * @param lBufferSize The requested buffer size.
     * @return The adjusted buffer size.
     */
    long suitableBufferSize(long lBufferSize) {
        // The actual buffer size must be a power of two to use bitwise operations instead of modulo
        if (lBufferSize <= 0) {
            lBufferSize = DEFAULT_RING_BUFFER_SIZE;
        } else {
            long lSize = (long)roundUpPowerOf2((uint64_t)lBufferSize);
            if (lSize <= 0) {
                lBufferSize = DEFAULT_RING_BUFFER_SIZE;
            } else {
                lBufferSize = lSize;
            }
        }
        return lBufferSize;
    }

    /**
     * @brief Creates a new ring buffer.
     *
     * Initializes a new ring buffer with the specified size and optional shared memory name.
     *
     * @param lBufferSize The size of the buffer.
     * @param strShmName The name of the shared memory (optional).
     * @return true if the buffer was successfully created, false otherwise.
     */
    bool createBuffer(long lBufferSize, const char *strShmName = nullptr) {
        if (m_hShareMemory != nullptr || m_pBufferData != nullptr) {
            return false;
        }
        long lSuitableBufferSize = suitableBufferSize(lBufferSize);
        // The lock-free implementation results in one less usable slot, so the buffer size must be at least 2
        if (lSuitableBufferSize == 1) {
            lSuitableBufferSize = 2;
        }
        if (strShmName != nullptr) {
            m_hShareMemory =
                os_sharememory_create(strShmName, sizeof(RING_BUFFER_CB) + lSuitableBufferSize * sizeof(T));
            m_pBufferData = os_sharememory_get_buffer(m_hShareMemory);
            bzero(m_pBufferData, sizeof(RING_BUFFER_CB) + lSuitableBufferSize * sizeof(T));
        } else {
            m_hShareMemory = nullptr;
            m_pBufferData = new char[sizeof(RING_BUFFER_CB) + lSuitableBufferSize * sizeof(T)];
            bzero(m_pBufferData, sizeof(RING_BUFFER_CB) + lSuitableBufferSize * sizeof(T));
        }
        if (m_pBufferData != nullptr) {
            m_pRingBufferCB = (RING_BUFFER_CB *)m_pBufferData;
            m_pRingBufferCB->m_lBufferSize = lSuitableBufferSize;
            m_pRingBufferCB->m_lUsedCount = 0;
            m_pRingBufferCB->m_lWriteIndex = 0;
            m_pRingBufferCB->m_lReadIndex = 0;
            m_pRingBufferCB->m_lMaximumReadIndex = 0;
            m_pRingBufferCB->m_intItemSize = sizeof(T);
            m_pRingBufferCB->m_intBufferTag = currentClassTag();
            m_pCacheData = (T *)((char *)m_pBufferData + sizeof(RING_BUFFER_CB));
            m_bMemoryOwner = true;
            return true;
        } else {
            destroyBuffer();
            return false;
        }
    }

    /**
     * @brief Opens an existing ring buffer.
     *
     * Opens an existing ring buffer using the specified shared memory name.
     *
     * @param strShmName The name of the shared memory.
     * @return true if the buffer was successfully opened, false otherwise.
     */
    bool openBuffer(const char *strShmName) {
        if (m_hShareMemory != nullptr || m_pBufferData != nullptr) {
            return false;
        } else if (strShmName == nullptr) {
            return false;
        } else {
            m_hShareMemory = os_sharememory_open(strShmName);
            if (m_hShareMemory != nullptr) {
                m_pBufferData = os_sharememory_get_buffer(m_hShareMemory);
                RING_BUFFER_CB *cb = (RING_BUFFER_CB *)m_pBufferData;
                if (cb->m_intItemSize != sizeof(T) || cb->m_intBufferTag != currentClassTag()) {
                    // Ensure the shared memory is created by the same class
                    os_sharememory_close(m_hShareMemory, false);
                    os_sharememory_destroy(m_hShareMemory);
                    m_hShareMemory = nullptr;
                    m_pBufferData = nullptr;
                    m_pRingBufferCB = nullptr;
                    m_pCacheData = nullptr;
                    return false;
                } else {
                    m_pRingBufferCB = cb;
                    m_pCacheData = (T *)((char *)m_pBufferData + sizeof(RING_BUFFER_CB));
                    m_bMemoryOwner = false;
                    return true;
                }
            } else {
                return false;
            }
        }
    }

    /**
     * @brief Closes the ring buffer.
     *
     * Closes the ring buffer if it is not the owner of the memory.
     *
     * @return true if the buffer was successfully closed, false otherwise.
     */
    bool closeBuffer() {
        if (m_bMemoryOwner == true) {
            return false;
        } else {
            if (m_hShareMemory != nullptr) {
                os_sharememory_close(m_hShareMemory, false);
                os_sharememory_destroy(m_hShareMemory);
                m_pBufferData = nullptr;
                m_hShareMemory = nullptr;
            }
            m_pRingBufferCB = nullptr;
            m_pCacheData = nullptr;
            return true;
        }
    }

    /**
     * @brief Destroys the ring buffer.
     *
     * Destroys the ring buffer if it is the owner of the memory.
     *
     * @return true if the buffer was successfully destroyed, false otherwise.
     */
    bool destroyBuffer() {
        if (m_bMemoryOwner == false) {
            return false;
        } else {
            if (m_hShareMemory != nullptr) {
                os_sharememory_close(m_hShareMemory, true);
                os_sharememory_destroy(m_hShareMemory);
                m_pBufferData = nullptr;
                m_hShareMemory = nullptr;
            } else {
                if (m_pBufferData != nullptr) {
                    delete[] (char *)m_pBufferData;
                    m_pBufferData = nullptr;
                }
            }
            m_bMemoryOwner = false;
            m_pRingBufferCB = nullptr;
            m_pCacheData = nullptr;
            return true;
        }
    }

private:
    /**
     * @brief Rounds up a value to the nearest power of two.
     *
     * Rounds up the given value to the nearest power of two.
     *
     * @param val The value to round up.
     * @return The rounded-up value.
     */
    static uint64_t roundUpPowerOf2(uint64_t val) {
        if ((val & (val - 1)) == 0) {
            // Already a power of two or zero, return as is
            return val;
        }
        uint64_t testV = 0x8000000000000000;
        while ((testV & val) == 0) {
            testV = testV >> 1;
        }
        return testV << 1;
    }

    /**
     * @brief Calculates the modulo index.
     *
     * Calculates the modulo index using bitwise operations for efficiency.
     *
     * @param lIndex The index to calculate.
     * @param lBufferSize The buffer size.
     * @return The calculated modulo index.
     */
    static long modIndex(long lIndex, long lBufferSize) {
        // Ensure parameters are valid: lIndex >= 0 && lBufferSize >= 0
        return (long)((unsigned long)lIndex) & ((unsigned long)lBufferSize - 1);
    }

protected:
    /**
     * @var void *m_pBufferData
     * @brief Pointer to the buffer data.
     */
    void *m_pBufferData;

    /**
     * @var sharememory_handle *m_hShareMemory
     * @brief Handle to the shared memory.
     */
    sharememory_handle *m_hShareMemory;

    /**
     * @var bool m_bMemoryOwner
     * @brief Flag indicating whether the buffer owns the memory.
     */
    bool m_bMemoryOwner;

    /**
     * @var RING_BUFFER_CB *m_pRingBufferCB
     * @brief Pointer to the control block structure.
     *
     * This pointer points to the control block structure that contains control information for the ring buffer,
     * including indices and sizes.
     */
    RING_BUFFER_CB *m_pRingBufferCB;

    /**
     * @var T *m_pCacheData
     * @brief Pointer to the cached data in the ring buffer.
     *
     * This pointer points to the start of the cached data area in the ring buffer where elements of type T are stored.
     */
    T *m_pCacheData;
};

#endif
