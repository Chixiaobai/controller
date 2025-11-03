/**
 * @file Queue.h
 * @brief Header file for queue management.
 *
 * This file defines a template class for managing queues with synchronization mechanisms.
 */

#ifndef __TOOLSDK_QUEUE_H__
#define __TOOLSDK_QUEUE_H__

#include "OS/OS_Sync.h"

/**
 * @class CQueue
 * @brief Template class for managing queues.
 *
 * This class provides a thread-safe queue implementation using mutexes and events for synchronization.
 * It is important to note that this class should not be used across real-time threads and non-real-time threads
 * due to the use of mutexes and semaphores, which may not be allowed in such contexts (e.g., in Xenomai).
 *
 * @tparam T The type of elements stored in the queue.
 */
template <class T>
class CQueue {
public:
    /**
     * @brief Constructor for CQueue.
     *
     * Initializes the queue with the specified real-time mode.
     *
     * @param mode The real-time mode for synchronization objects (default is USE_DEFAULT).
     */
    CQueue(RT_MODE mode = USE_DEFAULT)
        : m_QueueMutex(MUTEX_RECURSIVE, mode, nullptr), m_QueueNotEmptyEvent(false, false, mode) {
        m_bInterruptPop = false;
    }

    /**
     * @brief Destructor for CQueue.
     */
    virtual ~CQueue() {}

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CQueue objects to ensure proper management of synchronization objects.
     */
    CQueue(const CQueue &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CQueue objects to ensure proper management of synchronization objects.
     */
    CQueue &operator=(const CQueue &) = delete;
#endif

public:
    /**
     * @brief Resets the queue.
     *
     * Clears all elements from the queue and resets the interrupt pop flag.
     */
    void reset() {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        m_bInterruptPop = false;
        m_ElementQueue.clear();
    }

    /**
     * @brief Adds an element to the queue.
     *
     * Adds the specified element to the end of the queue and signals that the queue is not empty.
     *
     * @param data The element to be added to the queue.
     */
    void push(const T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        m_ElementQueue.push_back(data);
        m_QueueNotEmptyEvent.setEvent();
    }

    /**
     * @brief Attempts to remove and return the front element from the queue without blocking.
     *
     * Tries to pop the front element from the queue if it is not empty.
     *
     * @param data Reference to the variable where the popped element will be stored.
     * @return true if an element was successfully popped, false if the queue was empty.
     */
    bool tryPop(T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        if (m_ElementQueue.empty() == true)
            return false;
        else {
            data = m_ElementQueue.front();
            m_ElementQueue.pop_front();
            return true;
        }
    }

    /**
     * @brief Removes and returns the front element from the queue, blocking if necessary.
     *
     * Pops the front element from the queue, waiting indefinitely if the queue is empty.
     *
     * @param data Reference to the variable where the popped element will be stored.
     * @return true if an element was successfully popped, false if the operation was interrupted.
     */
    bool pop(T &data) { return pop(data, -1); }

    /**
     * @brief Removes and returns the front element from the queue with a timeout.
     *
     * Pops the front element from the queue, waiting up to the specified timeout if the queue is empty.
     *
     * @param data Reference to the variable where the popped element will be stored.
     * @param intTimeout Timeout in milliseconds (-1 for indefinite wait).
     * @return true if an element was successfully popped, false if the timeout expired or the operation was
     * interrupted.
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
     * @brief Removes and returns the front element from the queue with a specified timeout.
     *
     * Pops the front element from the queue, waiting up to the specified timeout if the queue is empty.
     *
     * @param data Reference to the variable where the popped element will be stored.
     * @param timeout The timeout value as a timespec structure.
     * @return true if an element was successfully popped, false if the timeout expired or the operation was
     * interrupted.
     */
    bool pop(T &data, timespec timeout) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        m_bInterruptPop = false;
        while (m_ElementQueue.empty() == true) {
            if (m_bInterruptPop == true) return false;
            m_QueueMutex.unlock();
            if (timeout.tv_sec == -1 && timeout.tv_nsec == -1) {
                if (m_QueueNotEmptyEvent.wait() == false) {
                    return false;
                }
            } else {
                if (m_QueueNotEmptyEvent.wait(timeout) == false) {
                    return false;
                }
            }
            m_QueueMutex.lock();
        }
        data = m_ElementQueue.front();
        m_ElementQueue.pop_front();
        return true;
    }

    /**
     * @brief Interrupts any waiting pop operations.
     *
     * Sets the interrupt pop flag and signals the queue not empty event to wake up any waiting threads.
     */
    void interruptPop() {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        m_bInterruptPop = true;
        m_QueueNotEmptyEvent.setEvent();
    }

    /**
     * @brief Retrieves the current size of the queue.
     *
     * Returns the number of elements currently in the queue.
     *
     * @return The size of the queue.
     */
    unsigned int getQueueSize() {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        return (unsigned int)m_ElementQueue.size();
    }

    /**
     * @brief Retrieves an element at a specified index in the queue.
     *
     * Retrieves the element at the specified index in the queue if it exists.
     *
     * @param uintIndex The index of the element to retrieve.
     * @param data Reference to the variable where the retrieved element will be stored.
     * @return true if the element was successfully retrieved, false if the index is out of bounds.
     */
    bool getQueueElement(unsigned int uintIndex, T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_QueueMutex);
        if (uintIndex <= (unsigned int)m_ElementQueue.size()) {
            typename list<T>::iterator it = m_ElementQueue.begin();
            unsigned int uintCurrentIndex = 0;
            while (uintCurrentIndex < uintIndex) {
                uintCurrentIndex++;
                it++;
            }
            data = *it;
            return true;
        } else
            return false;
    }

protected:
    /**
     * @var bool m_bInterruptPop
     * @brief Flag to indicate if pop operations should be interrupted.
     */
    bool m_bInterruptPop;

    /**
     * @var list<T> m_ElementQueue
     * @brief The internal list storing the queue elements.
     */
    list<T> m_ElementQueue;

    /**
     * @var OS::TBaseMutex m_QueueMutex
     * @brief Mutex for synchronizing access to the queue.
     */
    OS::TBaseMutex m_QueueMutex;  // 队列访问条件锁

    /**
     * @var OS::TEvent m_QueueNotEmptyEvent
     * @brief Event to signal when the queue is not empty.
     */
    OS::TEvent m_QueueNotEmptyEvent;
};

#endif  // __TOOLSDK_QUEUE_H__