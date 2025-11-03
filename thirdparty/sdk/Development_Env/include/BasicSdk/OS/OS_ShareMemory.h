#ifndef __BASICSDK_OS_SHARE_MEMORY_H__
#define __BASICSDK_OS_SHARE_MEMORY_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

struct sharememory_handle;

/**
 * @brief Create a shared memory segment
 *
 * @param strShmName Name of the shared memory segment
 * @param uintMemorySize Size of the shared memory segment
 * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
 * @return sharememory_handle* Pointer to the created shared memory handle
 */
BASICSDK_API sharememory_handle *os_sharememory_create(const char *strShmName, unsigned int uintMemorySize,
                                                       RT_MODE mode = USE_DEFAULT);

/**
 * @brief Open an existing shared memory segment
 *
 * @param strShmName Name of the shared memory segment
 * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
 * @return sharememory_handle* Pointer to the opened shared memory handle
 */
BASICSDK_API sharememory_handle *os_sharememory_open(const char *strShmName, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Destroy a shared memory segment
 *
 * @param handle Pointer to the shared memory handle to destroy
 */
BASICSDK_API void os_sharememory_destroy(sharememory_handle *&handle);

/**
 * @brief Close a shared memory segment
 *
 * @param handle Pointer to the shared memory handle to close
 * @param bNeedDestroy If true, destroy the shared memory segment (ignored on Windows)
 */
BASICSDK_API void os_sharememory_close(sharememory_handle *handle, bool bNeedDestroy = false);

/**
 * @brief Get the buffer of a shared memory segment
 *
 * @param handle Pointer to the shared memory handle
 * @return void* Pointer to the buffer of the shared memory segment
 */
BASICSDK_API void *os_sharememory_get_buffer(sharememory_handle *handle);

/**
 * @brief Namespace for shared memory-related functions and classes
 */
namespace OS {
class ShareMemory {
public:
    /**
     * @brief Constructor for ShareMemory to create a new shared memory segment
     *
     * @param strShmName Name of the shared memory segment
     * @param uintMemorySize Size of the shared memory segment
     * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
     */
    ShareMemory(const char *strShmName, unsigned int uintMemorySize, RT_MODE mode = USE_DEFAULT) {
        m_pShareMemoryHandle = os_sharememory_create(strShmName, uintMemorySize, mode);
    }

    /**
     * @brief Constructor for ShareMemory to open an existing shared memory segment
     *
     * @param strShmName Name of the shared memory segment
     * @param mode Real-time mode (USE_DEFAULT, USE_STD, USE_RT)
     */
    ShareMemory(const char *strShmName, RT_MODE mode = USE_DEFAULT) {
        m_pShareMemoryHandle = os_sharememory_open(strShmName, mode);
    }

    /**
     * @brief Destructor for ShareMemory
     *
     * Note: If you do not call close by yourself, the shared memory will not be unlinked.
     */
    virtual ~ShareMemory() { os_sharememory_destroy(m_pShareMemoryHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Delete copy constructor to prevent copying
     */
    ShareMemory(const ShareMemory &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying
     */
    ShareMemory &operator=(const ShareMemory &) = delete;
#endif

public:
    /**
     * @brief Close the shared memory segment
     *
     * @param bNeedDestroy If true, destroy the shared memory segment (ignored on Windows)
     */
    void close(bool bNeedDestroy = false) { os_sharememory_close(m_pShareMemoryHandle, bNeedDestroy); }

    /**
     * @brief Get the buffer of the shared memory segment
     *
     * @return void* Pointer to the buffer of the shared memory segment
     */
    void *getBuffer() { return os_sharememory_get_buffer(m_pShareMemoryHandle); }

protected:
    sharememory_handle *m_pShareMemoryHandle; /**< Pointer to the shared memory handle */
};
}  // namespace OS

#endif