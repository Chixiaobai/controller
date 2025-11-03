#ifndef __TOOLSDK_CRC_H__
#define __TOOLSDK_CRC_H__

/**
 * @file Crc.h
 * @brief Header file for CRC (Cyclic Redundancy Check) functionality.
 *
 * This file defines structures and functions for performing CRC calculations of various lengths (8, 16, 32, 64 bits).
 * It includes predefined parameters for common CRC algorithms and provides helper functions for calculating CRC values.
 */

#include "stdafx.h"
#include "ToolSdkDef.h"
#include "Crypto/Crc.h"

/**
 * @def CRC_TYPE_DEFINITION(name, type, typePrefix)
 * @brief Macro to define a parameter structure for a specific CRC type.
 *
 * This macro creates a structure that holds the necessary parameters for a CRC calculation:
 * - Polynomial: The polynomial used in the CRC algorithm.
 * - Initial Value: The initial value of the CRC register.
 * - XOR Output: The value to XOR with the final CRC result.
 * - Reflect Input: Whether to reflect the input data bits.
 * - Reflect Output: Whether to reflect the output CRC bits.
 *
 * @param name The name of the CRC type (e.g., CRC8, CRC16).
 * @param type The data type for the CRC values (e.g., unsigned char, unsigned short).
 * @param typePrefix A prefix for naming the members of the structure.
 */
#define CRC_TYPE_DEFINITION(name, type, typePrefix) \
    typedef struct _##name##_PARAMETER {            \
        type typePrefix##Polynomial;                \
        type typePrefix##CrcValue;                  \
        type typePrefix##XorOut;                    \
        bool bRefIn;                                \
        bool bRefOut;                               \
    } name##_PARAMETER

// Define parameter structures for different CRC types
CRC_TYPE_DEFINITION(CRC8, unsigned char, byte);
CRC_TYPE_DEFINITION(CRC16, unsigned short, ui);
CRC_TYPE_DEFINITION(CRC32, unsigned int, uint);
CRC_TYPE_DEFINITION(CRC64, unsigned long long, ull);

/**
 * @defgroup CRC8_Parameters CRC8 Parameters
 * @brief Predefined parameters for 8-bit CRC calculations.
 * @{
 */
const CRC8_PARAMETER CRC8_MAXIM = {0x31, 0x00, 0x00, true, true};      // x8 + x5 + x4 + 1
const CRC8_PARAMETER CRC8_REGULAR = {0x07, 0x00, 0x00, false, false};  // x8 + x5 + x4 + 1
const CRC8_PARAMETER CRC8_ITU = {0x07, 0x00, 0x55, false, false};      // x8 + x2 + x1 + 1
const CRC8_PARAMETER CRC8_ROHC = {0x07, 0xFF, 0x00, true, true};       // x8 + x2 + x1 + 1
/** @} */

/**
 * @defgroup CRC16_Parameters CRC16 Parameters
 * @brief Predefined parameters for 16-bit CRC calculations.
 * @{
 */
const CRC16_PARAMETER CRC16_IBM = {0x8005, 0x0000, 0x0000, true, true};            // x16 + x15 + x2 + 1
const CRC16_PARAMETER CRC16_MAXIM = {0x8005, 0x0000, 0xFFFF, true, true};          // x16 + x15 + x2 + 1
const CRC16_PARAMETER CRC16_USB = {0x8005, 0xFFFF, 0xFFFF, true, true};            // x16 + x15 + x2 + 1
const CRC16_PARAMETER CRC16_MODBUS = {0x8005, 0xFFFF, 0x0000, true, true};         // x16 + x15 + x2 + 1
const CRC16_PARAMETER CRC16_CCITT_KERMIT = {0x1021, 0x0000, 0x0000, true, true};   // x16 + x12 + x5 + 1
const CRC16_PARAMETER CRC16_CCITT_FALSE = {0x1021, 0xFFFF, 0x0000, false, false};  // x16 + x12 + x5 + 1
const CRC16_PARAMETER CRC16_X25 = {0x1021, 0xFFFF, 0xFFFF, true, true};            // x16 + x12 + x5 + 1
const CRC16_PARAMETER CRC16_XMODEM = {0x1021, 0x0000, 0x0000, false, false};       // x16 + x12 + x5 + 1
const CRC16_PARAMETER CRC16_DNP = {0x3D65, 0x0000, 0xFFFF, true, true};            // x16 + x15 + x2 + 1
/** @} */

/**
 * @defgroup CRC32_Parameters CRC32 Parameters
 * @brief Predefined parameters for 32-bit CRC calculations.
 * @{
 */
const CRC32_PARAMETER CRC32_BZIP2 = {
    0x04C11DB7ul, 0xFFFFFFFF, 0xFFFFFFFF, true,
    true};  // x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
const CRC32_PARAMETER CRC32_CKSUM = {
    0x04C11DB7ul, 0x00000000, 0xFFFFFFFF, false,
    false};  // x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
const CRC32_PARAMETER CRC32_IEEE802_3 = {
    0x04C11DB7ul, 0xFFFFFFFF, 0xFFFFFFFF, true,
    true};  // x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
const CRC32_PARAMETER CRC32_MPEG_2 = {
    0x04C11DB7ul, 0xFFFFFFFF, 0x00000000, false,
    false};  // x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
/** @} */

/**
 * @defgroup CRC64_Parameters CRC64 Parameters
 * @brief Predefined parameters for 64-bit CRC calculations.
 * @{
 */
const CRC64_PARAMETER CRC64_GOISO = {0x000000000000001Bull, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF, true,
                                     true};  // x64 + x4 + x3 + x + 1
const CRC64_PARAMETER CRC64_ECMA_182 = {
    0x42F0E1EBA9EA3693ull, 0x0000000000000000, 0x0000000000000000, false,
    false};  // x64 + x62 + x57 + x55 + x54 + x53 + ... + x13 + x12 + x10 + x9 + x7 + x4 + x + 1
const CRC64_PARAMETER CRC64_WE = {
    0x42F0E1EBA9EA3693ull, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF, false,
    false};  // x64 + x62 + x57 + x55 + x54 + x53 + ... + x13 + x12 + x10 + x9 + x7 + x4 + x + 1
const CRC64_PARAMETER CRC64_XZ = {
    0x42F0E1EBA9EA3693ull, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF, true,
    true};  // x64 + x62 + x57 + x55 + x54 + x53 + ... + x13 + x12 + x10 + x9 + x7 + x4 + x + 1
/** @} */

/**
 * @struct crc_context_handle
 * @brief Opaque handle for managing CRC contexts.
 *
 * This structure represents an opaque handle used internally for managing CRC contexts.
 */
struct crc_context_handle;

/**
 * @fn crc_context_handle *crc_context_create()
 * @brief Creates a new CRC context handle.
 *
 * Allocates and initializes a new CRC context handle for use in subsequent CRC calculations.
 *
 * @return A pointer to the newly created CRC context handle.
 */
TOOLSDK_API crc_context_handle *crc_context_create();

/**
 * @fn void crc_context_destroy(crc_context_handle *&handle)
 * @brief Destroys a CRC context handle.
 *
 * Frees the resources associated with the given CRC context handle.
 *
 * @param handle A reference to the CRC context handle to be destroyed.
 */
TOOLSDK_API void crc_context_destroy(crc_context_handle *&handle);

/**
 * @fn unsigned char crc8_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer, unsigned int
 * uintContentSize, const CRC8_PARAMETER &crcParameter)
 * @brief Calculates an 8-bit CRC value for a buffer.
 *
 * Computes the 8-bit CRC value for the given content buffer using the specified CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strContentBuffer Pointer to the buffer containing the data.
 * @param uintContentSize The size of the content buffer in bytes.
 * @param crcParameter The CRC8 parameters to use.
 * @return The computed 8-bit CRC value.
 */
TOOLSDK_API unsigned char crc8_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer,
                                         unsigned int uintContentSize, const CRC8_PARAMETER &crcParameter);

/**
 * @fn bool crc8_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned char *byteCrc, const
 * CRC8_PARAMETER &crcParameter)
 * @brief Calculates an 8-bit CRC value for a file.
 *
 * Computes the 8-bit CRC value for the contents of the specified file using the given CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strFilePath The path to the file.
 * @param byteCrc Pointer to store the computed CRC value.
 * @param crcParameter The CRC8 parameters to use.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool crc8_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned char *byteCrc,
                                     const CRC8_PARAMETER &crcParameter);

/**
 * @fn unsigned short crc16_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer, unsigned int
 * uintContentSize, const CRC16_PARAMETER &crcParameter)
 * @brief Calculates a 16-bit CRC value for a buffer.
 *
 * Computes the 16-bit CRC value for the given content buffer using the specified CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strContentBuffer Pointer to the buffer containing the data.
 * @param uintContentSize The size of the content buffer in bytes.
 * @param crcParameter The CRC16 parameters to use.
 * @return The computed 16-bit CRC value.
 */
TOOLSDK_API unsigned short crc16_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer,
                                           unsigned int uintContentSize, const CRC16_PARAMETER &crcParameter);

/**
 * @fn bool crc16_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned short *uiCrc, const
 * CRC16_PARAMETER &crcParameter)
 * @brief Calculates a 16-bit CRC value for a file.
 *
 * Computes the 16-bit CRC value for the contents of the specified file using the given CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strFilePath The path to the file.
 * @param uiCrc Pointer to store the computed CRC value.
 * @param crcParameter The CRC16 parameters to use.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool crc16_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned short *uiCrc,
                                      const CRC16_PARAMETER &crcParameter);

/**
 * @fn unsigned int crc32_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer, unsigned int
 * uintContentSize, const CRC32_PARAMETER &crcParameter)
 * @brief Calculates a 32-bit CRC value for a buffer.
 *
 * Computes the 32-bit CRC value for the given content buffer using the specified CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strContentBuffer Pointer to the buffer containing the data.
 * @param uintContentSize The size of the content buffer in bytes.
 * @param crcParameter The CRC32 parameters to use.
 * @return The computed 32-bit CRC value.
 */
TOOLSDK_API unsigned int crc32_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer,
                                         unsigned int uintContentSize, const CRC32_PARAMETER &crcParameter);

/**
 * @fn bool crc32_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned int *uintCrc, const
 * CRC32_PARAMETER &crcParameter)
 * @brief Calculates a 32-bit CRC value for a file.
 *
 * Computes the 32-bit CRC value for the contents of the specified file using the given CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strFilePath The path to the file.
 * @param uintCrc Pointer to store the computed CRC value.
 * @param crcParameter The CRC32 parameters to use.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool crc32_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned int *uintCrc,
                                      const CRC32_PARAMETER &crcParameter);

/**
 * @fn unsigned long long crc64_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer, unsigned
 * int uintContentSize, const CRC64_PARAMETER &crcParameter)
 * @brief Calculates a 64-bit CRC value for a buffer.
 *
 * Computes the 64-bit CRC value for the given content buffer using the specified CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strContentBuffer Pointer to the buffer containing the data.
 * @param uintContentSize The size of the content buffer in bytes.
 * @param crcParameter The CRC64 parameters to use.
 * @return The computed 64-bit CRC value.
 */
TOOLSDK_API unsigned long long crc64_calculate(crc_context_handle *handle, const unsigned char *strContentBuffer,
                                               unsigned int uintContentSize, const CRC64_PARAMETER &crcParameter);

/**
 * @fn bool crc64_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned long long *ullCrc, const
 * CRC64_PARAMETER &crcParameter)
 * @brief Calculates a 64-bit CRC value for a file.
 *
 * Computes the 64-bit CRC value for the contents of the specified file using the given CRC parameters.
 *
 * @param handle The CRC context handle.
 * @param strFilePath The path to the file.
 * @param ullCrc Pointer to store the computed CRC value.
 * @param crcParameter The CRC64 parameters to use.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool crc64_calculate_file(crc_context_handle *handle, const char *strFilePath, unsigned long long *ullCrc,
                                      const CRC64_PARAMETER &crcParameter);
/**
 * @class CCrcHelper
 * @brief Helper class for CRC calculations.
 *
 * This class provides a convenient interface for performing CRC calculations of various lengths (8, 16, 32, 64 bits).
 * It manages the CRC context handle internally and offers methods to calculate CRC values for both buffers and files.
 */
class CCrcHelper {
public:
    /**
     * @brief Constructor for CCrcHelper.
     *
     * Initializes the CRC helper by creating a CRC context handle.
     */
    CCrcHelper() { m_hCrcContextHandle = crc_context_create(); }

    /**
     * @brief Destructor for CCrcHelper.
     *
     * Cleans up the CRC helper by destroying the CRC context handle.
     */
    virtual ~CCrcHelper() { crc_context_destroy(m_hCrcContextHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CCrcHelper objects to ensure proper management of the CRC context handle.
     */
    CCrcHelper(const CCrcHelper &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CCrcHelper objects to ensure proper management of the CRC context handle.
     */
    CCrcHelper &operator=(const CCrcHelper &) = delete;
#endif

public:
    /**
     * @brief Calculates an 8-bit CRC value for a buffer.
     *
     * Computes the 8-bit CRC value for the given content buffer using the specified CRC parameters.
     *
     * @param strContentBuffer Pointer to the buffer containing the data.
     * @param uintContentSize The size of the content buffer in bytes.
     * @param crcParameter The CRC8 parameters to use (default is CRC8_MAXIM).
     * @return The computed 8-bit CRC value.
     */
    unsigned char calculate_crc8(const unsigned char *strContentBuffer, unsigned int uintContentSize,
                                 const CRC8_PARAMETER &crcParameter = CRC8_MAXIM) {
        return crc8_calculate(m_hCrcContextHandle, strContentBuffer, uintContentSize, crcParameter);
    }

    /**
     * @brief Calculates an 8-bit CRC value for a file.
     *
     * Computes the 8-bit CRC value for the contents of the specified file using the given CRC parameters.
     *
     * @param strFilePath The path to the file.
     * @param byteCrc Pointer to store the computed CRC value.
     * @param crcParameter The CRC8 parameters to use (default is CRC8_MAXIM).
     * @return true if the operation was successful, false otherwise.
     */
    bool calculate_file_crc8(const char *strFilePath, unsigned char *byteCrc,
                             const CRC8_PARAMETER &crcParameter = CRC8_MAXIM) {
        return crc8_calculate_file(m_hCrcContextHandle, strFilePath, byteCrc, crcParameter);
    }

    /**
     * @brief Calculates a 16-bit CRC value for a buffer.
     *
     * Computes the 16-bit CRC value for the given content buffer using the specified CRC parameters.
     *
     * @param strContentBuffer Pointer to the buffer containing the data.
     * @param uintContentSize The size of the content buffer in bytes.
     * @param crcParameter The CRC16 parameters to use (default is CRC16_IBM).
     * @return The computed 16-bit CRC value.
     */
    unsigned short calculate_crc16(const unsigned char *strContentBuffer, unsigned int uintContentSize,
                                   const CRC16_PARAMETER &crcParameter = CRC16_IBM) {
        return crc16_calculate(m_hCrcContextHandle, strContentBuffer, uintContentSize, crcParameter);
    }

    /**
     * @brief Calculates a 16-bit CRC value for a file.
     *
     * Computes the 16-bit CRC value for the contents of the specified file using the given CRC parameters.
     *
     * @param strFilePath The path to the file.
     * @param uiCrc Pointer to store the computed CRC value.
     * @param crcParameter The CRC16 parameters to use (default is CRC16_IBM).
     * @return true if the operation was successful, false otherwise.
     */
    bool calculate_file_crc16(const char *strFilePath, unsigned short *uiCrc,
                              const CRC16_PARAMETER &crcParameter = CRC16_IBM) {
        return crc16_calculate_file(m_hCrcContextHandle, strFilePath, uiCrc, crcParameter);
    }

    /**
     * @brief Calculates a 32-bit CRC value for a buffer.
     *
     * Computes the 32-bit CRC value for the given content buffer using the specified CRC parameters.
     *
     * @param strContentBuffer Pointer to the buffer containing the data.
     * @param uintContentSize The size of the content buffer in bytes.
     * @param crcParameter The CRC32 parameters to use (default is CRC32_IEEE802_3).
     * @return The computed 32-bit CRC value.
     */
    unsigned int calculate_crc32(const unsigned char *strContentBuffer, unsigned int uintContentSize,
                                 const CRC32_PARAMETER &crcParameter = CRC32_IEEE802_3) {
        return crc32_calculate(m_hCrcContextHandle, strContentBuffer, uintContentSize, crcParameter);
    }

    /**
     * @brief Calculates a 32-bit CRC value for a file.
     *
     * Computes the 32-bit CRC value for the contents of the specified file using the given CRC parameters.
     *
     * @param strFilePath The path to the file.
     * @param uintCrc Pointer to store the computed CRC value.
     * @param crcParameter The CRC32 parameters to use (default is CRC32_IEEE802_3).
     * @return true if the operation was successful, false otherwise.
     */
    bool calculate_file_crc32(const char *strFilePath, unsigned int *uintCrc,
                              const CRC32_PARAMETER &crcParameter = CRC32_IEEE802_3) {
        return crc32_calculate_file(m_hCrcContextHandle, strFilePath, uintCrc, crcParameter);
    }

    /**
     * @brief Calculates a 64-bit CRC value for a buffer.
     *
     * Computes the 64-bit CRC value for the given content buffer using the specified CRC parameters.
     *
     * @param strContentBuffer Pointer to the buffer containing the data.
     * @param uintContentSize The size of the content buffer in bytes.
     * @param crcParameter The CRC64 parameters to use (default is CRC64_WE).
     * @return The computed 64-bit CRC value.
     */
    unsigned long long calculate_crc64(const unsigned char *strContentBuffer, unsigned int uintContentSize,
                                       const CRC64_PARAMETER &crcParameter = CRC64_WE) {
        return crc64_calculate(m_hCrcContextHandle, strContentBuffer, uintContentSize, crcParameter);
    }

    /**
     * @brief Calculates a 64-bit CRC value for a file.
     *
     * Computes the 64-bit CRC value for the contents of the specified file using the given CRC parameters.
     *
     * @param strFilePath The path to the file.
     * @param ullCrc Pointer to store the computed CRC value.
     * @param crcParameter The CRC64 parameters to use (default is CRC64_WE).
     * @return true if the operation was successful, false otherwise.
     */
    bool calculate_file_crc64(const char *strFilePath, unsigned long long *ullCrc,
                              const CRC64_PARAMETER &crcParameter = CRC64_WE) {
        return crc64_calculate_file(m_hCrcContextHandle, strFilePath, ullCrc, crcParameter);
    }

protected:
    /**
     * @var crc_context_handle *m_hCrcContextHandle
     * @brief Opaque handle for managing the CRC context.
     *
     * This handle is used internally by the CCrcHelper class to manage the CRC context.
     */
    crc_context_handle *m_hCrcContextHandle;
};

#endif
