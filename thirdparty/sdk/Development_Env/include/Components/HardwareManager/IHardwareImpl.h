#pragma once

#include "Base/IBase.h"

/**
 * @interface IHardwareImpl
 * @brief Interface defining low-level hardware implementation operations.
 *
 * This interface provides methods for implementing core hardware functionalities including
 * opening/closing connections, reading/writing data, setting properties, and managing link states.
 */
interface IHardwareImpl : public IBase {
    /**
     * @brief Implements the opening of a hardware connection.
     *
     * @return true if the connection was successfully opened; false otherwise.
     */
    virtual bool openImpl() = 0;

    /**
     * @brief Implements writing raw data to the hardware.
     *
     * @param buf Pointer to the data buffer.
     * @param len Number of bytes to write.
     * @return Number of bytes actually written.
     */
    virtual int writeImpl(const unsigned char *buf, int len) = 0;

    /**
     * @brief Implements reading raw data from the hardware.
     *
     * @param buf Buffer to receive the data.
     * @param len Maximum number of bytes to read.
     * @return Number of bytes actually read.
     */
    virtual int readImpl(unsigned char *buf, int len) = 0;

    /**
     * @brief Implements closing the hardware connection.
     */
    virtual void closeImpl() = 0;

    /**
     * @brief Implements setting a hardware property.
     *
     * @param name Name of the property to set.
     * @param value New value for the property.
     * @return true if the property was successfully set; false otherwise.
     */
    virtual bool setPropertyImpl(const char *name, const char *value) = 0;

    /**
     * @brief Waits until the hardware is readable within the specified timeout.
     *
     * @param dwMillionTimeout Timeout in milliseconds.
     * @return true if hardware becomes readable within the timeout; false otherwise.
     */
    virtual bool selectReadableImpl(int dwMillionTimeout) = 0;

    /**
     * @brief Waits until the hardware is writable within the specified timeout.
     *
     * @param dwMillionTimeout Timeout in milliseconds.
     * @return true if hardware becomes writable within the timeout; false otherwise.
     */
    virtual bool selectWritableImpl(int dwMillionTimeout) = 0;

    /**
     * @brief Clears internal buffers and resets state.
     */
    virtual void clearImpl() = 0;

    /**
     * @brief Checks whether the hardware link is broken.
     *
     * @return true if the link is broken; false otherwise.
     */
    virtual bool isLinkBrokenImpl() = 0;

    /**
     * @brief Notifies that the hardware link has been broken.
     */
    virtual void brokenLinkImpl() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareImpl)
