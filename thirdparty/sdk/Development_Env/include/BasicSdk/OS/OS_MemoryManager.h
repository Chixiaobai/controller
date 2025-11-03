#ifndef __BASICSDK_OS_MEMORY_MANAGER_H__
#define __BASICSDK_OS_MEMORY_MANAGER_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Lock a memory region
 *
 * @param __address Pointer to the start of the memory region
 * @param __len Length of the memory region to lock
 * @return bool True if the memory region is locked successfully, false otherwise
 */
BASICSDK_API bool os_memory_manager_mlock(void *__address, unsigned int __len);

/**
 * @brief Unlock a memory region
 *
 * @param __address Pointer to the start of the memory region
 * @param __len Length of the memory region to unlock
 * @return bool True if the memory region is unlocked successfully, false otherwise
 */
BASICSDK_API bool os_memory_manager_munlock(void *__address, unsigned int __len);

/**
 * @brief Lock all current and future memory allocations
 *
 * @param __flags Flags specifying which memory to lock (RT_MCL_CURRENT, RT_MCL_FUTURE, or both)
 * @return bool True if the memory is locked successfully, false otherwise
 */
BASICSDK_API bool os_memory_manager_mlockall(int __flags);

/**
 * @brief Unlock all locked memory
 *
 * @return bool True if all locked memory is unlocked successfully, false otherwise
 */
BASICSDK_API bool os_memory_manager_munlockall();

/**
 * @brief Namespace for memory management functions and classes
 */
namespace OS {
class MemoryManager {
public:
    /**
     * @brief Flag to lock current memory mappings
     */
    static const int RT_MCL_CURRENT = 1;

    /**
     * @brief Flag to lock future memory mappings
     */
    static const int RT_MCL_FUTURE = 2;

    /**
     * @brief Lock a memory region
     *
     * @param __address Pointer to the start of the memory region
     * @param __len Length of the memory region to lock
     * @return bool True if the memory region is locked successfully, false otherwise
     */
    static bool mLock(void *__address, unsigned int __len) { return os_memory_manager_mlock(__address, __len); }

    /**
     * @brief Unlock a memory region
     *
     * @param __address Pointer to the start of the memory region
     * @param __len Length of the memory region to unlock
     * @return bool True if the memory region is unlocked successfully, false otherwise
     */
    static bool mUnlock(void *__address, unsigned int __len) { return os_memory_manager_munlock(__address, __len); }

    /**
     * @brief Lock all current and future memory allocations
     *
     * @param __flags Flags specifying which memory to lock (RT_MCL_CURRENT, RT_MCL_FUTURE, or both)
     * @return bool True if the memory is locked successfully, false otherwise
     */
    static bool mLockAll(int __flags) { return os_memory_manager_mlockall(__flags); }

    /**
     * @brief Unlock all locked memory
     *
     * @return bool True if all locked memory is unlocked successfully, false otherwise
     */
    static bool mUnlockAll(void) { return os_memory_manager_munlockall(); }
};
}  // namespace OS

#endif