#pragma once

#include "Base/IBase.h"

/**
 * @enum HARDWARE_SELECT_PROPERTY
 * @brief Properties used to check hardware readiness for I/O operations.
 */
enum HARDWARE_SELECT_PROPERTY {
    HARDWARE_READABLE = 1, /**< Check if hardware is ready for reading */
    HARDWARE_WRITABLE = 2, /**< Check if hardware is ready for writing */
};

/**
 * @interface IHardwareCore
 * @brief Core interface for interacting with hardware devices.
 *
 * This interface provides methods to open/close the hardware connection,
 * read/write data, exchange buffers, and query device properties.
 */
interface IHardwareCore : public IBase {
    /**
     * @brief Opens the hardware connection.
     *
     * @return true if the connection was successfully opened; false otherwise.
     */
    virtual bool open() = 0;

    /**
     * @brief Retrieves a property value by name.
     *
     * @param name Name of the property to retrieve.
     * @param value Buffer to receive the property value.
     * @param valueLen On input, specifies buffer size; on output, receives actual length.
     * @return true if the property was retrieved successfully; false otherwise.
     */
    virtual bool getProperty(const char *name, char *value, DWORD &valueLen) = 0;

    /**
     * @brief Writes raw data to the hardware.
     *
     * @param pBuffer Pointer to the data buffer.
     * @param iBufLen Number of bytes to write.
     * @return Number of bytes actually written.
     */
    virtual long write(const unsigned char *pBuffer, long iBufLen) = 0;

    /**
     * @brief Reads raw data from the hardware.
     *
     * @param pBuffer Buffer to receive the data.
     * @param iBufLen Maximum number of bytes to read.
     * @return Number of bytes actually read.
     */
    virtual long read(unsigned char *pBuffer, long iBufLen) = 0;

    /**
     * @brief Sends data and receives a response in one operation.
     *
     * @param inputBuf Data to send.
     * @param lInLen Number of bytes to send.
     * @param outputBuf Buffer to receive the response.
     * @param lOutBufLen Maximum number of bytes to receive.
     * @return true if the exchange completed successfully; false otherwise.
     */
    virtual bool exchange(const unsigned char *inputBuf, long lInLen, unsigned char *outputBuf, long lOutBufLen) = 0;

    /**
     * @brief Clears all internal data caches.
     */
    virtual void clearAllCache() = 0;

    /**
     * @brief Closes the hardware connection.
     */
    virtual void close() = 0;

    /**
     * @brief Checks whether the hardware connection is currently open.
     *
     * @return true if open; false otherwise.
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Waits for the hardware to become ready for I/O operations.
     *
     * @param iProperty Type of readiness to check (readable or writable).
     * @param dwMilliSeconds Timeout in milliseconds.
     * @return true if the hardware is ready within the timeout; false otherwise.
     */
    virtual bool select(HARDWARE_SELECT_PROPERTY iProperty, long dwMilliSeconds) = 0;

    /**
     * @brief Retrieves the current hardware configuration as a string.
     *
     * @param strHardwareConfiguration Buffer to receive the configuration string.
     * @param dwLen On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getHardwareConfiguration(char *strHardwareConfiguration, DWORD &dwLen) = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareCore)

class CHardwareCoreHelper {
public:
    static bool getProperty(IHardwareCore *hardwareCore, string name, string &value) {
        if (hardwareCore != nullptr) {
            DWORD dwLen = 0;
            if (hardwareCore->getProperty(name.c_str(), nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                hardwareCore->getProperty(name.c_str(), pValue, dwLen);
                value = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
    static bool getHardwareConfiguration(IHardwareCore *hardwareCore, string &strHardwareConfiguration) {
        if (hardwareCore != nullptr) {
            DWORD dwLen = 0;
            if (hardwareCore->getHardwareConfiguration(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                hardwareCore->getHardwareConfiguration(pValue, dwLen);
                strHardwareConfiguration = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
};
