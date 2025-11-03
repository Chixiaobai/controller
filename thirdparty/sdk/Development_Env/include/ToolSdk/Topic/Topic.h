#ifndef __APPLE__

#ifndef __TOOLSDK_TOPIC_H__
#define __TOOLSDK_TOPIC_H__

#include "ToolSdkDef.h"
#include "OS/OSType.h"

/**
 * @file Topic.h
 * @brief Header file for topic-based communication classes and functions.
 *
 * This file contains the definitions for topic-based communication classes and functions, including error codes,
 * handles, and template classes for topic buffers, publishers, and subscribers.
 */

/**
 * @enum SUBSCRIBER_ERROR
 * @brief Error codes for subscriber operations.
 *
 * This enumeration defines error codes that can be returned by subscriber operations.
 */
enum SUBSCRIBER_ERROR {
    UNEXPECTED_PARAMETER_ERROR = -1, /**< Unexpected parameter error. */
    OPERATE_TIMEOUT_ERROR = -2,      /**< Operation timeout error. */
    RESOURCE_ACCESS_ERROR = -3       /**< Resource access error. */
};

/**
 * @struct topic_buffer_handle
 * @brief Handle for a topic buffer.
 *
 * This structure represents a handle to a topic buffer used for communication.
 */
struct topic_buffer_handle;

/**
 * @struct publisher_handle
 * @brief Handle for a publisher.
 *
 * This structure represents a handle to a publisher used for sending messages to a topic buffer.
 */
struct publisher_handle;

/**
 * @struct subscriber_handle
 * @brief Handle for a subscriber.
 *
 * This structure represents a handle to a subscriber used for receiving messages from a topic buffer.
 */
struct subscriber_handle;

/**
 * @brief Creates a topic buffer.
 *
 * Creates a new topic buffer with the specified URL, topic size, buffer size, and process usage flag.
 *
 * @param strTopicUrl The URL of the topic buffer.
 * @param uintTopicSize The size of each topic message.
 * @param uintBufferSize The number of messages the buffer can hold.
 * @param bUsedInProcess Flag indicating if the buffer is used within the same process.
 * @return A pointer to the created topic buffer handle, or nullptr if creation failed.
 */
TOOLSDK_API topic_buffer_handle *topic_buffer_create(const char *strTopicUrl, unsigned int uintTopicSize,
                                                     unsigned int uintBufferSize, const bool bUsedInProcess = false);

/**
 * @brief Destroys a topic buffer.
 *
 * Destroys the specified topic buffer and releases associated resources.
 *
 * @param handle Reference to the topic buffer handle to be destroyed.
 */
TOOLSDK_API void topic_buffer_destroy(topic_buffer_handle *&handle);

/**
 * @brief Gets the name of a topic buffer.
 *
 * Retrieves the name of the specified topic buffer.
 *
 * @param handle The topic buffer handle.
 * @param strBufferName Buffer to store the name of the topic buffer.
 * @param dwLen Length of the buffer to store the name.
 * @return true if the name was successfully retrieved, false otherwise.
 */
TOOLSDK_API bool topic_buffer_name(topic_buffer_handle *handle, char *strBufferName, DWORD &dwLen);

/**
 * @brief Creates a publisher.
 *
 * Creates a new publisher for the specified topic buffer name.
 *
 * @param strBufferName The name of the topic buffer.
 * @return A pointer to the created publisher handle, or nullptr if creation failed.
 */
TOOLSDK_API publisher_handle *publisher_create(const char *strBufferName);

/**
 * @brief Destroys a publisher.
 *
 * Destroys the specified publisher and releases associated resources.
 *
 * @param handle Reference to the publisher handle to be destroyed.
 */
TOOLSDK_API void publisher_destroy(publisher_handle *&handle);

/**
 * @brief Writes a message to a publisher.
 *
 * Writes a message to the specified publisher.
 *
 * @param handle The publisher handle.
 * @param pMessage Pointer to the message to be written.
 * @param uintMessageSize Size of the message.
 * @return true if the message was successfully written, false otherwise.
 */
TOOLSDK_API bool publisher_write(publisher_handle *handle, const void *pMessage, unsigned int uintMessageSize);

/**
 * @brief Creates a subscriber.
 *
 * Creates a new subscriber for the specified topic buffer name.
 *
 * @param strBufferName The name of the topic buffer.
 * @return A pointer to the created subscriber handle, or nullptr if creation failed.
 */
TOOLSDK_API subscriber_handle *subscriber_create(const char *strBufferName);

/**
 * @brief Destroys a subscriber.
 *
 * Destroys the specified subscriber and releases associated resources.
 *
 * @param handle Reference to the subscriber handle to be destroyed.
 */
TOOLSDK_API void subscriber_destroy(subscriber_handle *&handle);

/**
 * @brief Reads a message from a subscriber.
 *
 * Reads a message from the specified subscriber with a specified timeout and read interval.
 *
 * @param handle The subscriber handle.
 * @param pMessage Buffer to store the read message.
 * @param uintMessageSize Size of the message buffer.
 * @param timeout Timeout for reading the message (in seconds).
 * @param intErrorCode Pointer to an integer where the error code will be stored if an error occurs.
 * @param read_interval Interval between read attempts (in seconds).
 * @param mode Real-time mode for the read operation.
 * @return true if the message was successfully read, false otherwise.
 */
TOOLSDK_API bool subscriber_read(subscriber_handle *handle, void *pMessage, unsigned int uintMessageSize,
                                 double timeout = 0.0, int *intErrorCode = nullptr, double read_interval = 5e-5,
                                 RT_MODE mode = USE_DEFAULT);

/**
 * @class CTopicBuffer
 * @brief Template class for managing a topic buffer.
 *
 * This template class provides a convenient interface for managing a topic buffer, including creation and destruction.
 */
template <class T>
class CTopicBuffer {
public:
    /**
     * @brief Constructor for CTopicBuffer.
     *
     * Initializes a new CTopicBuffer with the specified topic URL, buffer size, and process usage flag.
     *
     * @param strTopicUrl The URL of the topic buffer.
     * @param uintBufferSize The number of messages the buffer can hold.
     * @param bUsedInProcess Flag indicating if the buffer is used within the same process.
     */
    CTopicBuffer(const char *strTopicUrl, unsigned int uintBufferSize, const bool bUsedInProcess = false) {
        m_pTopicBufferHandle = topic_buffer_create(strTopicUrl, sizeof(T), uintBufferSize, bUsedInProcess);
    }

    /**
     * @brief Destructor for CTopicBuffer.
     *
     * Destroys the CTopicBuffer and releases associated resources.
     */
    virtual ~CTopicBuffer() {
        if (m_pTopicBufferHandle != nullptr) {
            topic_buffer_destroy(m_pTopicBufferHandle);
        }
    }

public:
    /**
     * @brief Gets the name of the topic buffer.
     *
     * Retrieves the name of the topic buffer.
     *
     * @param strBufferName Reference to the string where the name of the topic buffer will be stored.
     * @return true if the name was successfully retrieved, false otherwise.
     */
    bool getBufferName(string &strBufferName) {
        if (m_pTopicBufferHandle == nullptr) {
            return false;
        }
        DWORD dwSize = 0;
        if (topic_buffer_name(m_pTopicBufferHandle, nullptr, dwSize) == true && dwSize > 0) {
            char *strValueA = new char[dwSize + 1];
            bzero(strValueA, (dwSize + 1) * sizeof(char));
            topic_buffer_name(m_pTopicBufferHandle, strValueA, dwSize);
            strBufferName = strValueA;
            delete[] strValueA;
            strValueA = nullptr;
            return true;
        } else {
            return false;
        }
    }

protected:
    /**
     * @var topic_buffer_handle *m_pTopicBufferHandle
     * @brief Handle to the topic buffer.
     */
    topic_buffer_handle *m_pTopicBufferHandle;
};

/**
 * @class CTopicPublisher
 * @brief Template class for managing a topic publisher.
 *
 * This template class provides a convenient interface for managing a topic publisher, including creation and
 * destruction.
 */
template <class T>
class CTopicPublisher {
public:
    /**
     * @brief Constructor for CTopicPublisher.
     *
     * Initializes a new CTopicPublisher for the specified topic buffer name.
     *
     * @param strBufferName The name of the topic buffer.
     */
    CTopicPublisher(const char *strBufferName) { m_pTopicPublisherHandle = publisher_create(strBufferName); }

    /**
     * @brief Destructor for CTopicPublisher.
     *
     * Destroys the CTopicPublisher and releases associated resources.
     */
    virtual ~CTopicPublisher() {
        if (m_pTopicPublisherHandle != nullptr) {
            publisher_destroy(m_pTopicPublisherHandle);
        }
    }

public:
    /**
     * @brief Writes a message to the publisher.
     *
     * Writes a message to the publisher.
     *
     * @param pMessage Pointer to the message to be written.
     * @return true if the message was successfully written, false otherwise.
     */
    bool write(const T *pMessage) { return publisher_write(m_pTopicPublisherHandle, pMessage, sizeof(T)); }

    /**
     * @brief Writes a message to the publisher.
     *
     * Writes a message to the publisher.
     *
     * @param msg The message to be written.
     * @return true if the message was successfully written, false otherwise.
     */
    bool write(const T &msg) { return write(&msg); }

protected:
    /**
     * @var publisher_handle *m_pTopicPublisherHandle
     * @brief Handle to the publisher.
     */
    publisher_handle *m_pTopicPublisherHandle;
};

/**
 * @class CTopicSubscriber
 * @brief Template class for managing a topic subscriber.
 *
 * This template class provides a convenient interface for managing a topic subscriber, including creation and
 * destruction.
 */
template <class T>
class CTopicSubscriber {
public:
    /**
     * @brief Constructor for CTopicSubscriber.
     *
     * Initializes a new CTopicSubscriber for the specified topic buffer name.
     *
     * @param strBufferName The name of the topic buffer.
     */
    CTopicSubscriber(const char *strBufferName) { m_pTopicSubscriberHandle = subscriber_create(strBufferName); }

    /**
     * @brief Destructor for CTopicSubscriber.
     *
     * Destroys the CTopicSubscriber and releases associated resources.
     */
    virtual ~CTopicSubscriber() {
        if (m_pTopicSubscriberHandle != nullptr) {
            subscriber_destroy(m_pTopicSubscriberHandle);
        }
    }

public:
    /**
     * @brief Reads a message from the subscriber.
     *
     * Reads a message from the subscriber with a specified timeout and read interval.
     *
     * @param pMessage Buffer to store the read message.
     * @param timeout Timeout for reading the message (in seconds).
     * @param read_interval Interval between read attempts (in seconds).
     * @param mode Real-time mode for the read operation.
     * @return true if the message was successfully read, false otherwise.
     */
    bool read(T *pMessage, double timeout, double read_interval = 5e-5, RT_MODE mode = USE_DEFAULT) {
        T msg;
        bool ret = subscriber_read(m_pTopicSubscriberHandle, &msg, sizeof(T), timeout, read_interval, mode);
        if (ret == true) {
            memcpy(pMessage, &msg, sizeof(T));
        }
        return ret;
    }

    /**
     * @brief Reads a message from the subscriber.
     *
     * Reads a message from the subscriber with a specified timeout and read interval.
     *
     * @param msg Reference to the message where the read data will be stored.
     * @param timeout Timeout for reading the message (in seconds).
     * @param read_interval Interval between read attempts (in seconds).
     * @param mode Real-time mode for the read operation.
     * @return true if the message was successfully read, false otherwise.
     */
    bool read(T &msg, double timeout, double read_interval = 5e-5, RT_MODE mode = USE_DEFAULT) {
        return read(&msg, timeout, read_interval, mode);
    }

protected:
    /**
     * @var subscriber_handle *m_pTopicSubscriberHandle
     * @brief Handle to the subscriber.
     */
    subscriber_handle *m_pTopicSubscriberHandle;
};

#endif

#endif