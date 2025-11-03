#ifndef __TOOLSDK_STACK_H__
#define __TOOLSDK_STACK_H__

#include "OS/OS_Sync.h"

/**
 * @file Stack.h
 * @brief Header file for the CStack template class.
 *
 * This file contains the definition of the CStack template class, which provides a thread-safe stack implementation
 * using mutexes and events for synchronization. Note that this class should not be used across real-time threads and
 * non-real-time threads due to the limitations of mutexes and semaphores in such environments (e.g., Xenomai).
 */

// This class should not be used across real-time threads and non-real-time threads because its internal
// implementation relies on mutexes and semaphores, which may not allow such operations (e.g., in Xenomai).
template <class T>
class CStack {
public:
    /**
     * @brief Constructor for CStack.
     *
     * Initializes a new CStack object with the specified real-time mode. The stack mutex is initialized as recursive,
     * and the stack not-empty event is initialized as a manual reset event.
     *
     * @param mode The real-time mode for the stack (default is USE_DEFAULT).
     */
    CStack(RT_MODE mode = USE_DEFAULT)
        : m_StackMutex(MUTEX_RECURSIVE, mode, nullptr), m_StackNotEmptyEvent(false, false, mode) {
        m_bInterruptPop = false;
    }

    /**
     * @brief Destructor for CStack.
     *
     * Cleans up the CStack object.
     */
    virtual ~CStack() {}

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CStack objects to ensure proper management of resources.
     */
    CStack(const CStack &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CStack objects to ensure proper management of resources.
     */
    CStack &operator=(const CStack &) = delete;
#endif

public:
    /**
     * @brief Resets the stack.
     *
     * Clears all elements from the stack and resets the interrupt pop flag.
     */
    void reset() {
        m_bInterruptPop = false;
        m_ElementStack.clear();
    }

    /**
     * @brief Pushes an element onto the stack.
     *
     * Pushes the specified element onto the stack and signals the stack not-empty event.
     *
     * @param data The element to push onto the stack.
     */
    void push(T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_StackMutex);
        m_ElementStack.push_back(data);
        m_StackNotEmptyEvent.setEvent();
    }

    /**
     * @brief Tries to pop an element from the stack.
     *
     * Attempts to pop an element from the stack. If the stack is empty, it returns false without blocking.
     *
     * @param data Reference to the element where the popped data will be stored.
     * @return true if an element was successfully popped, false if the stack was empty.
     */
    bool tryPop(T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_StackMutex);
        if (m_ElementStack.empty() == true)
            return false;
        else {
            data = m_ElementStack.back();
            m_ElementStack.pop_back();
            return true;
        }
    }

    /**
     * @brief Pops an element from the stack.
     *
     * Pops an element from the stack. If the stack is empty, it waits for the stack not-empty event before attempting
     * to pop the element. If the interrupt pop flag is set, it returns false without popping an element.
     *
     * @param data Reference to the element where the popped data will be stored.
     * @return true if an element was successfully popped, false if the stack was empty and the interrupt pop flag was
     * set.
     */
    bool pop(T &data) {
        TAutoMutex<OS::TBaseMutex> m_AutoMutex(&m_StackMutex);
        m_bInterruptPop = false;
        while (m_ElementStack.empty() == true) {
            if (m_bInterruptPop == true) return false;
            m_StackMutex.unlock();
            m_StackNotEmptyEvent.wait();
            m_StackMutex.lock();
        }
        data = m_ElementStack.back();
        m_ElementStack.pop_back();
        return true;
    }

    /**
     * @brief Interrupts the pop operation.
     *
     * Sets the interrupt pop flag and signals the stack not-empty event to interrupt any waiting pop operations.
     */
    void interruptPop() {
        m_bInterruptPop = true;
        m_StackNotEmptyEvent.setEvent();
    }

    /**
     * @brief Gets the size of the stack.
     *
     * Returns the number of elements currently in the stack.
     *
     * @return The size of the stack.
     */
    unsigned int getStackSize() { return (unsigned int)m_ElementStack.size(); }

    /**
     * @brief Gets an element from the stack by index.
     *
     * Retrieves the element at the specified index in the stack. If the index is out of range, it returns false.
     *
     * @param uintIndex The index of the element to retrieve.
     * @param data Reference to the element where the retrieved data will be stored.
     * @return true if the element was successfully retrieved, false if the index was out of range.
     */
    bool getStackElement(unsigned int uintIndex, T &data) {
        if (uintIndex <= (unsigned int)m_ElementStack.size()) {
            typename list<T>::iterator it = m_ElementStack.begin();
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
     * @brief Flag to interrupt the pop operation.
     *
     * When set to true, it interrupts any waiting pop operations.
     */
    bool m_bInterruptPop;

    /**
     * @var list<T> m_ElementStack
     * @brief The internal list that stores the stack elements.
     */
    list<T> m_ElementStack;

    /**
     * @var OS::TBaseMutex m_StackMutex
     * @brief The mutex used to synchronize access to the stack.
     *
     * This mutex is used to ensure that only one thread can access the stack at a time.
     */
    OS::TBaseMutex m_StackMutex;  // Stack access condition lock

    /**
     * @var OS::TEvent m_StackNotEmptyEvent
     * @brief The event used to signal that the stack is not empty.
     *
     * This event is set when an element is pushed onto the stack and is used to wake up any threads waiting for an
     * element.
     */
    OS::TEvent m_StackNotEmptyEvent;
};

#endif