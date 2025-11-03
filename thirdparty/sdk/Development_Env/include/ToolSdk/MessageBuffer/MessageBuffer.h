/**
 * @file MessageBuffer.h
 * @brief Header file for message buffer management.
 *
 * This file defines structures, functions, and a template class for managing message buffers.
 */

#ifndef __APPLE__

#ifndef __TOOLSDK_MESSAGE_BUFFER_H__
#define __TOOLSDK_MESSAGE_BUFFER_H__

#include "ToolSdkDef.h"
#include "OS/OSType.h"

/**
 * @def MAX_MESSAGE_URL_LEN
 * @brief Maximum length for message URLs.
 *
 * Defines the maximum length for message URLs used in message buffers.
 */
#define MAX_MESSAGE_URL_LEN 128

/**
 * @struct message_buffer_handle
 * @brief Opaque handle for managing message buffers.
 *
 * This structure represents an opaque handle used internally for managing message buffers.
 */
struct message_buffer_handle;

/**
 * @fn message_buffer_handle *message_buffer_create(const char *strMessageUrl, unsigned int uintHeadSize, unsigned int
 * uintMessageSize, unsigned int uintBufferSize, const bool bUsedInProcess)
 * @brief Creates a new message buffer.
 *
 * Creates a new message buffer with the specified parameters.
 *
 * @param strMessageUrl The URL for the message buffer.
 * @param uintHeadSize The size of the header in the message buffer.
 * @param uintMessageSize The size of each message element.
 * @param uintBufferSize The size of the buffer.
 * @param bUsedInProcess Indicates whether the buffer is used within a single process.
 * @return A pointer to the newly created message buffer handle, or nullptr if the operation failed.
 */
TOOLSDK_API message_buffer_handle *message_buffer_create(const char *strMessageUrl, unsigned int uintHeadSize,
                                                         unsigned int uintMessageSize, unsigned int uintBufferSize,
                                                         const bool bUsedInProcess);

/**
 * @fn message_buffer_handle *message_buffer_open(const char *strBufferName)
 * @brief Opens an existing message buffer.
 *
 * Opens an existing message buffer by its name.
 *
 * @param strBufferName The name of the message buffer to open.
 * @return A pointer to the opened message buffer handle, or nullptr if the operation failed.
 */
TOOLSDK_API message_buffer_handle *message_buffer_open(const char *strBufferName);

/**
 * @fn void message_buffer_destroy(message_buffer_handle *&handle)
 * @brief Destroys a message buffer.
 *
 * Destroys the specified message buffer and frees its resources.
 *
 * @param handle A reference to the message buffer handle to be destroyed.
 */
TOOLSDK_API void message_buffer_destroy(message_buffer_handle *&handle);

/**
 * @fn bool message_buffer_url(message_buffer_handle *handle, char *strMessageUrl, DWORD &dwLen)
 * @brief Retrieves the URL of a message buffer.
 *
 * Retrieves the URL of the specified message buffer and stores it in the provided buffer.
 *
 * @param handle The message buffer handle.
 * @param strMessageUrl Pointer to the buffer to store the URL.
 * @param dwLen Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool message_buffer_url(message_buffer_handle *handle, char *strMessageUrl, DWORD &dwLen);

/**
 * @fn bool message_buffer_name(message_buffer_handle *handle, char *strBufferName, DWORD &dwLen)
 * @brief Retrieves the name of a message buffer.
 *
 * Retrieves the name of the specified message buffer and stores it in the provided buffer.
 *
 * @param handle The message buffer handle.
 * @param strBufferName Pointer to the buffer to store the name.
 * @param dwLen Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool message_buffer_name(message_buffer_handle *handle, char *strBufferName, DWORD &dwLen);

/**
 * @fn bool message_buffer_element_size(message_buffer_handle *handle, unsigned int &uintMessageSize)
 * @brief Retrieves the size of each message element in a message buffer.
 *
 * Retrieves the size of each message element in the specified message buffer.
 *
 * @param handle The message buffer handle.
 * @param uintMessageSize Reference to the size of each message element.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool message_buffer_element_size(message_buffer_handle *handle, unsigned int &uintMessageSize);

/**
 * @fn bool message_buffer_size(message_buffer_handle *handle, unsigned int &uintBufferSize)
 * @brief Retrieves the size of a message buffer.
 *
 * Retrieves the size of the specified message buffer.
 *
 * @param handle The message buffer handle.
 * @param uintBufferSize Reference to the size of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool message_buffer_size(message_buffer_handle *handle, unsigned int &uintBufferSize);

/**
 * @fn bool message_buffer_category(message_buffer_handle *handle, bool &bUsedInProcess)
 * @brief Retrieves the category of a message buffer.
 *
 * Retrieves whether the specified message buffer is used within a single process.
 *
 * @param handle The message buffer handle.
 * @param bUsedInProcess Reference to a boolean indicating whether the buffer is used within a single process.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool message_buffer_category(message_buffer_handle *handle, bool &bUsedInProcess);

/**
 * @fn bool message_buffer_owner(message_buffer_handle *handle)
 * @brief Checks if the current process is the owner of a message buffer.
 *
 * Checks if the current process is the owner of the specified message buffer.
 *
 * @param handle The message buffer handle.
 * @return true if the current process is the owner, false otherwise.
 */
TOOLSDK_API bool message_buffer_owner(message_buffer_handle *handle);

/**
 * @fn void *message_buffer_head_address(message_buffer_handle *handle)
 * @brief Retrieves the address of the head of a message buffer.
 *
 * Retrieves the address of the head of the specified message buffer.
 *
 * @param handle The message buffer handle.
 * @return The address of the head of the message buffer.
 */
TOOLSDK_API void *message_buffer_head_address(message_buffer_handle *handle);

/**
 * @fn void *message_buffer_element_address(message_buffer_handle *handle, unsigned int uintIndex = 0)
 * @brief Retrieves the address of a message element in a message buffer.
 *
 * Retrieves the address of the specified message element in the message buffer.
 *
 * @param handle The message buffer handle.
 * @param uintIndex The index of the message element.
 * @return The address of the specified message element.
 */
TOOLSDK_API void *message_buffer_element_address(message_buffer_handle *handle, unsigned int uintIndex = 0);

/**
 * @class CMessageBuffer
 * @brief Template class for managing message buffers.
 *
 * This template class provides a convenient interface for creating, opening, and managing message buffers.
 *
 * @tparam T The type of the message elements.
 */
template <class T>
class CMessageBuffer {
public:
    /**
     * @brief Constructor for CMessageBuffer.
     *
     * Initializes the message buffer with a null handle.
     */
    CMessageBuffer() { m_pMessageBufferHandle = nullptr; }

    /**
     * @brief Destructor for CMessageBuffer.
     *
     * Cleans up the message buffer by destroying it if it is not null.
     */
    ~CMessageBuffer() {
        if (m_pMessageBufferHandle != nullptr) {
            message_buffer_destroy(m_pMessageBufferHandle);
        }
    }

    /**
     * @brief Creates a message buffer.
     *
     * Creates a new message buffer with the specified parameters.
     *
     * @param strMessageUrl The URL for the message buffer.
     * @param uintHeadSize The size of the header in the message buffer.
     * @param uintBufferSize The size of the buffer (default is 10).
     * @param bUsedInProcess Indicates whether the buffer is used within a single process (default is false).
     * @return true if the buffer was created successfully, false otherwise.
     */
    bool createBuffer(const char *strMessageUrl, unsigned int uintHeadSize, unsigned int uintBufferSize = 10,
                      const bool bUsedInProcess = false) {
        if (m_pMessageBufferHandle == nullptr) {
            m_pMessageBufferHandle =
                message_buffer_create(strMessageUrl, uintHeadSize, sizeof(T), uintBufferSize, bUsedInProcess);
            return m_pMessageBufferHandle != nullptr;
        } else {
            return false;
        }
    }

    /**
     * @brief Opens a message buffer.
     *
     * Opens an existing message buffer by its URL.
     *
     * @param strMessageUrl The URL of the message buffer to open.
     * @return true if the buffer was opened successfully, false otherwise.
     */
    bool openBuffer(const char *strMessageUrl) {
        if (m_pMessageBufferHandle == nullptr) {
            m_pMessageBufferHandle = message_buffer_open(strMessageUrl);
            return m_pMessageBufferHandle != nullptr;
        } else {
            return false;
        }
    }

    /**
     * @brief Retrieves the URL of the message buffer.
     *
     * Retrieves the URL of the message buffer and stores it in the provided string.
     *
     * @param strMessageUrl Reference to the string to store the URL.
     * @return true if the operation was successful, false otherwise.
     */
    bool getMessageUrl(string &strMessageUrl) {
        if (m_pMessageBufferHandle == nullptr) {
            return false;
        }
        DWORD dwSize = 0;
        if (message_buffer_url(m_pMessageBufferHandle, nullptr, dwSize) == true && dwSize > 0) {
            char *strValueA = new char[dwSize + 1];
            bzero(strValueA, (dwSize + 1) * sizeof(char));
            message_buffer_url(m_pMessageBufferHandle, strValueA, dwSize);
            strMessageUrl = strValueA;
            delete[] strValueA;
            strValueA = nullptr;
            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief Retrieves the name of the message buffer.
     *
     * Retrieves the name of the message buffer and stores it in the provided string.
     *
     * @param strBufferName Reference to the string to store the name.
     * @return true if the operation was successful, false otherwise.
     */
    bool getBufferName(string &strBufferName) {
        if (m_pMessageBufferHandle == nullptr) {
            return false;
        }
        DWORD dwSize = 0;
        if (message_buffer_name(m_pMessageBufferHandle, nullptr, dwSize) == true && dwSize > 0) {
            char *strValueA = new char[dwSize + 1];
            bzero(strValueA, (dwSize + 1) * sizeof(char));
            message_buffer_name(m_pMessageBufferHandle, strValueA, dwSize);
            strBufferName = strValueA;
            delete[] strValueA;
            strValueA = nullptr;
            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief Retrieves the size of each message element in the message buffer.
     *
     * Retrieves the size of each message element in the message buffer.
     *
     * @param uintMessageSize Reference to the size of each message element.
     * @return true if the operation was successful, false otherwise.
     */
    bool getMessageItemSize(unsigned int &uintMessageSize) {
        return message_buffer_element_size(m_pMessageBufferHandle, uintMessageSize);
    }

    /**
     * @brief Retrieves the size of the message buffer.
     *
     * Retrieves the size of the message buffer.
     *
     * @param uintBufferSize Reference to the size of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    bool getBufferSize(unsigned int &uintBufferSize) {
        return message_buffer_size(m_pMessageBufferHandle, uintBufferSize);
    }

    /**
     * @brief Checks if the message buffer can be used between processes.
     *
     * Checks if the message buffer can be used between different processes.
     *
     * @return true if the buffer can be used between processes, false otherwise.
     */
    bool canUsedBetweenProcess() {
        bool bUsedInProcessed;
        if (message_buffer_category(m_pMessageBufferHandle, bUsedInProcessed) == true) {
            return bUsedInProcessed == false;
        } else {
            return false;
        }
    }

    /**
     * @brief Checks if the current process is the owner of the message buffer.
     *
     * Checks if the current process is the owner of the message buffer.
     *
     * @return true if the current process is the owner, false otherwise.
     */
    bool isBufferOwner() { return message_buffer_owner(m_pMessageBufferHandle); }

    /**
     * @brief Retrieves the address of the head of the message buffer.
     *
     * Retrieves the address of the head of the message buffer.
     *
     * @return The address of the head of the message buffer.
     */
    void *getMessageBufferHeadAddress() { return message_buffer_head_address(m_pMessageBufferHandle); }

    /**
     * @brief Retrieves the address of a message element in the message buffer.
     *
     * Retrieves the address of the specified message element in the message buffer.
     *
     * @param uintIndex The index of the message element (default is 0).
     * @return The address of the specified message element.
     */
    T *getElementAddress(unsigned int uintIndex = 0) {
        return (T *)message_buffer_element_address(m_pMessageBufferHandle, uintIndex);
    }

protected:
    /**
     * @var message_buffer_handle *m_pMessageBufferHandle
     * @brief Opaque handle for managing the message buffer.
     *
     * This handle is used internally by the CMessageBuffer class to manage the message buffer.
     */
    message_buffer_handle *m_pMessageBufferHandle;
};

#endif  // __TOOLSDK_MESSAGE_BUFFER_H__

#endif  // __APPLE__