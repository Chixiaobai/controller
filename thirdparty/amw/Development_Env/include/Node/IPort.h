#pragma once

#include "Base/IBase.h"

/**
 * @interface IPortBase
 * @brief Base interface for all communication ports.
 *
 * This interface defines common initialization and metadata access methods
 * that are shared by both input and output port types.
 */
interface IPortBase : public IBase {
    /**
     * @brief Initializes the port with topic and message definition information.
     * @param topicName Name of the topic this port will publish or subscribe to.
     * @param messageDefName Name of the message definition (e.g., message type).
     * @param messageSize Size in bytes of each message.
     * @param buffers Number of message buffers to allocate. Default is 1.
     * @return true if initialization was successful; false otherwise.
     */
    virtual bool initPort(const char *topicName, const char *messageDefName, unsigned int messageSize,
                          unsigned int buffers = 1) = 0;

    /**
     * @brief Retrieves the topic name associated with this port.
     * @param strTopicName Buffer to store the topic name.
     * @param dwLen Length of buffer on input, actual length written on output.
     * @return true if successful; false otherwise.
     */
    virtual bool getTopicName(char *strTopicName, DWORD &dwLen) const = 0;

    /**
     * @brief Retrieves the message definition name associated with this port.
     * @param strMessageDefName Buffer to store the message definition name.
     * @param dwLen Length of buffer on input, actual length written on output.
     * @return true if successful; false otherwise.
     */
    virtual bool getMessageDefName(char *strMessageDefName, DWORD &dwLen) const = 0;
};

/**
 * @interface IOutputPort
 * @brief Interface for output (publishing) ports.
 *
 * This interface provides methods to write data to a topic and query statistics
 * about published messages.
 */
interface IOutputPort : public IBase {
    /**
     * @brief Gets the number of messages published through this port.
     * @return The total count of published messages.
     */
    virtual size_t getPublishCount() const = 0;

    /**
     * @brief Gets the estimated frequency of message publication.
     * @return Average publish rate in messages per second.
     */
    virtual double getPublishFrequency() const = 0;

    /**
     * @brief Writes a message to the topic.
     * @param data Pointer to the message data to be published.
     * @return true if the message was successfully queued for transmission; false otherwise.
     */
    virtual bool write(const void *data) = 0;
};

/**
 * @interface IInputPort
 * @brief Interface for input (subscribing) ports.
 *
 * This interface provides methods to read data from a topic, either blocking or non-blocking.
 */
interface IInputPort : public IBase {
    /**
     * @brief Reads a message from the topic.
     * @param data Buffer where the received message will be stored.
     * @param timeout Maximum time to wait in seconds. Zero means non-blocking.
     * @param intErrorCode Optional pointer to an integer to receive error codes.
     * @param mode Real-time mode behavior flag. Defaults to USE_DEFAULT.
     * @return true if a message was successfully read within the timeout period; false otherwise.
     */
    virtual bool read(void *data, double timeout = 0.0, int *intErrorCode = nullptr, RT_MODE mode = USE_DEFAULT) = 0;
};

ENABLE_RECOGNIZE_MACRO(IPortBase)
ENABLE_RECOGNIZE_MACRO(IOutputPort)
ENABLE_RECOGNIZE_MACRO(IInputPort)

class CPortBaseHelper {
public:
    static bool getTopicName(const IPortBase *pPortBase, string &strTopicName) {
        if (pPortBase != nullptr) {
            DWORD dwLen = 0;
            if (pPortBase->getTopicName(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pPortBase->getTopicName(pValue, dwLen);
                strTopicName = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
    static bool getMessageDefName(const IPortBase *pPortBase, string &strMessageDefName) {
        if (pPortBase != nullptr) {
            DWORD dwLen = 0;
            if (pPortBase->getMessageDefName(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pPortBase->getMessageDefName(pValue, dwLen);
                strMessageDefName = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
};
