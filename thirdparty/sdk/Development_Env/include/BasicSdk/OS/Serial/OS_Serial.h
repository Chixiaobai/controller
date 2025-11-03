#pragma once

#include "OSType_Serial.h"

/**
 * @brief Define serial communication bit flags
 */
enum SERIAL_BIT { SERIAL_TX_BIT = 0x01, SERIAL_RX_BIT = 0x02 };

/**
 * @brief Open a serial port
 *
 * This function opens a serial port based on the operating system.
 *
 * @param strSerialName Name of the serial port
 * @param uBaudRate Baud rate
 * @param byteParity Parity bit
 * @param byteStopbits Stop bits
 * @param iBaudIndex Baud rate index (used only on non-Windows systems, default is B38400)
 * @return OS_SERIAL_HANDLE Handle to the serial port
 */
#ifdef _WIN32
BASICSDK_API OS_SERIAL_HANDLE os_serial_open(const char *strSerialName, UINT uBaudRate, BYTE byteParity,
                                             BYTE byteStopbits);
#else
BASICSDK_API OS_SERIAL_HANDLE os_serial_open(const char *strSerialName, UINT uBaudRate, BYTE byteParity,
                                             BYTE byteStopbits, int iBaudIndex = B38400);
#endif

/**
 * @brief Close a serial port
 *
 * @param hHandle Handle to the serial port
 */
BASICSDK_API void os_serial_close(OS_SERIAL_HANDLE &hHandle);

/**
 * @brief Purge a serial port
 *
 * @param hHandle Handle to the serial port
 * @param iFlag Purge flag
 */
BASICSDK_API void os_serial_purge(OS_SERIAL_HANDLE hHandle, int iFlag);

/**
 * @brief Select a serial port
 *
 * @param hHandle Handle to the serial port
 * @param dwMillionTimeout Timeout in microseconds
 * @return bool True if successful, false otherwise
 */
BASICSDK_API bool os_serial_select(OS_SERIAL_HANDLE hHandle, int dwMillionTimeout);

/**
 * @brief Write data to a serial port
 *
 * @param hHandle Handle to the serial port
 * @param buf Buffer containing data to write
 * @param len Length of the data to write
 * @param dwTimeoutMillisecond Maximum timeout in millisecond, should between [0, 1000] (optional)
 * @return int Number of bytes actually written
 */
BASICSDK_API int os_serial_write(OS_SERIAL_HANDLE hHandle, const unsigned char *buf, int len, int dwTimeoutMillisecond = 100);

/**
 * @brief Read data from a serial port
 *
 * @param hHandle Handle to the serial port
 * @param buf Buffer to store the read data
 * @param len Length of the buffer
 * @param m_pHead Pointer to the head data (optional)
 * @param m_uHeadLen Length of the head data (optional)
 * @param dwMaxByteInterval Maximum byte interval time (optional)
 * @param dwTimeoutMillisecond Maximum timeout in millisecond, should between [0, 1000] (optional)
 * @return int Number of bytes actually read
 */
BASICSDK_API int os_serial_read(OS_SERIAL_HANDLE hHandle, unsigned char *buf, int len, void *m_pHead = nullptr,
                                UINT m_uHeadLen = 0, int dwMaxByteInterval = 5, int dwTimeoutMillisecond = 100);

/**
 * @brief Namespace for serial communication functions and classes
 */
namespace OS {
class Serial {
public:
    /**
     * @brief Open a serial port (Windows-specific)
     *
     * @param strSerialName Name of the serial port
     * @param uBaudRate Baud rate
     * @param byteParity Parity bit
     * @param byteStopbits Stop bits
     * @return OS_SERIAL_HANDLE Handle to the serial port
     */
#ifdef _WIN32
    static OS_SERIAL_HANDLE open(const char *strSerialName, UINT uBaudRate, BYTE byteParity, BYTE byteStopbits) {
        return os_serial_open(strSerialName, uBaudRate, byteParity, byteStopbits);
    }
#else
    /**
     * @brief Open a serial port (non-Windows-specific)
     *
     * @param strSerialName Name of the serial port
     * @param uBaudRate Baud rate
     * @param byteParity Parity bit
     * @param byteStopbits Stop bits
     * @param iBaudIndex Baud rate index (default is B38400)
     * @return OS_SERIAL_HANDLE Handle to the serial port
     */
    static OS_SERIAL_HANDLE open(const char *strSerialName, UINT uBaudRate, BYTE byteParity, BYTE byteStopbits,
                                 int iBaudIndex = B38400) {
        return os_serial_open(strSerialName, uBaudRate, byteParity, byteStopbits, iBaudIndex);
    }
#endif

    /**
     * @brief Close a serial port
     *
     * @param hHandle Handle to the serial port
     */
    static void close(OS_SERIAL_HANDLE &hHandle) { return os_serial_close(hHandle); }

    /**
     * @brief Purge a serial port
     *
     * @param hHandle Handle to the serial port
     * @param iFlag Purge flag
     */
    static void purge(OS_SERIAL_HANDLE hHandle, int iFlag) { return os_serial_purge(hHandle, iFlag); }

    /**
     * @brief Select a serial port
     *
     * @param hHandle Handle to the serial port
     * @param dwMillionTimeout Timeout in microseconds
     * @return bool True if successful, false otherwise
     */
    static bool select(OS_SERIAL_HANDLE hHandle, int dwMillionTimeout) {
        return os_serial_select(hHandle, dwMillionTimeout);
    }

    /**
     * @brief Write data to a serial port
     *
     * @param hHandle Handle to the serial port
     * @param buf Buffer containing data to write
     * @param len Length of the data to write
     * @return int Number of bytes actually written
     */
    static int write(OS_SERIAL_HANDLE hHandle, const unsigned char *buf, int len) {
        return os_serial_write(hHandle, buf, len);
    }

    /**
     * @brief Read data from a serial port
     *
     * @param hHandle Handle to the serial port
     * @param buf Buffer to store the read data
     * @param len Length of the buffer
     * @param m_pHead Pointer to the head data (optional)
     * @param m_uHeadLen Length of the head data (optional)
     * @param dwMaxByteInterval Maximum byte interval time (optional)
     * @return int Number of bytes actually read
     */
    static int read(OS_SERIAL_HANDLE hHandle, unsigned char *buf, int len, void *m_pHead = nullptr, UINT m_uHeadLen = 0,
                    int dwMaxByteInterval = 5) {
        return os_serial_read(hHandle, buf, len, m_pHead, m_uHeadLen, dwMaxByteInterval);
    }
};
}  // namespace OS