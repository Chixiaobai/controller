/**
 * @file ResourcePool.h
 * @brief Header file for resource pool management.
 *
 * This file defines a template class for managing a pool of resources with synchronization mechanisms.
 */

#ifndef __TOOLSDK_RESOURCE_POOL_H__
#define __TOOLSDK_RESOURCE_POOL_H__

#include "OS/OS_Sync.h"

/**
 * @class CResourcePool
 * @brief Template class for managing a pool of resources.
 *
 * This class provides a thread-safe resource pool implementation using a recursive mutex for synchronization.
 * It allows allocation and deallocation of resources in a thread-safe manner.
 *
 * @tparam T The type of resources managed by the pool.
 */
template <class T>
class CResourcePool {
public:
    /**
     * @brief Constructor for CResourcePool.
     *
     * Initializes the resource pool.
     */
    CResourcePool() {}

    /**
     * @brief Destructor for CResourcePool.
     *
     * Frees all resources in the pool when the object is destroyed.
     */
    virtual ~CResourcePool() { freeAllResource(); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CResourcePool objects to ensure proper management of resources.
     */
    CResourcePool(const CResourcePool &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CResourcePool objects to ensure proper management of resources.
     */
    CResourcePool &operator=(const CResourcePool &) = delete;
#endif

public:
    /**
     * @brief Allocates a new resource.
     *
     * Allocates a new resource of type T and adds it to the resource pool.
     *
     * @return Pointer to the newly allocated resource.
     */
    T *mallocResource() {
        m_ResourceMutex.lock();
        T *pNewResource = new T;
        m_ResourcePool.push_back(pNewResource);
        m_ResourceMutex.unlock();
        return pNewResource;
    }

    /**
     * @brief Frees a specified resource.
     *
     * Frees the specified resource and removes it from the resource pool.
     *
     * @param pResourceAddress Reference to the pointer of the resource to be freed.
     * @return true if the resource was successfully freed, false if the resource was not found in the pool.
     */
    bool freeResource(T *&pResourceAddress) {
        m_ResourceMutex.lock();
        typename list<T *>::iterator it = m_ResourcePool.begin();
        while (it != m_ResourcePool.end()) {
            if (*it == pResourceAddress) {
                m_ResourcePool.erase(it);
                m_ResourceMutex.unlock();
                delete pResourceAddress;
                pResourceAddress = nullptr;
                return true;
            }
            it++;
        }
        m_ResourceMutex.unlock();
        return false;
    }

    /**
     * @brief Frees all resources in the pool.
     *
     * Frees all resources in the pool and clears the resource list.
     */
    void freeAllResource() {
        m_ResourceMutex.lock();
        while (m_ResourcePool.empty() == false) {
            T *pResourceAddress = m_ResourcePool.front();
            m_ResourcePool.pop_front();
            delete pResourceAddress;
            pResourceAddress = nullptr;
        }
        m_ResourceMutex.unlock();
    }

protected:
    /**
     * @var list<T *> m_ResourcePool
     * @brief The internal list storing the resources.
     */
    list<T *> m_ResourcePool;

    /**
     * @var OS::TRecursiveMutex m_ResourceMutex
     * @brief Recursive mutex for synchronizing access to the resource pool.
     */
    OS::TRecursiveMutex m_ResourceMutex;
};

#endif  // __TOOLSDK_RESOURCE_POOL_H__