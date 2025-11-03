#ifndef __TOOLSDK_MD5_H__
#define __TOOLSDK_MD5_H__

/**
 * @file Md5.h
 * @brief Header file for MD5 (Message-Digest Algorithm 5) functionality.
 *
 * This file defines structures and functions for performing MD5 hash calculations.
 * It includes helper functions for calculating MD5 hashes of buffers and files.
 */

#include "ToolSdkDef.h"
#include "StringAssistant/StringAssistant.h"

/**
 * @def MD5_DIGEST_SIZE
 * @brief The size of the MD5 digest in bytes.
 */
#define MD5_DIGEST_SIZE 16

/**
 * @struct md5_context_handle
 * @brief Opaque handle for managing MD5 contexts.
 *
 * This structure represents an opaque handle used internally for managing MD5 contexts.
 */
struct md5_context_handle;

/**
 * @fn md5_context_handle *md5_context_create()
 * @brief Creates a new MD5 context handle.
 *
 * Allocates and initializes a new MD5 context handle for use in subsequent MD5 calculations.
 *
 * @return A pointer to the newly created MD5 context handle.
 */
TOOLSDK_API md5_context_handle *md5_context_create();

/**
 * @fn void md5_context_destroy(md5_context_handle *&handle)
 * @brief Destroys an MD5 context handle.
 *
 * Frees the resources associated with the given MD5 context handle.
 *
 * @param handle A reference to the MD5 context handle to be destroyed.
 */
TOOLSDK_API void md5_context_destroy(md5_context_handle *&handle);

/**
 * @fn void md5_calculate(md5_context_handle *handle, const unsigned char *strContentBuffer, unsigned int
 * uintContentSize, unsigned char strMd5Result[MD5_DIGEST_SIZE])
 * @brief Calculates the MD5 hash for a buffer.
 *
 * Computes the MD5 hash for the given content buffer and stores the result in the provided buffer.
 *
 * @param handle The MD5 context handle.
 * @param strContentBuffer Pointer to the buffer containing the data.
 * @param uintContentSize The size of the content buffer in bytes.
 * @param strMd5Result Pointer to a buffer of size MD5_DIGEST_SIZE to store the computed MD5 hash.
 */
TOOLSDK_API void md5_calculate(md5_context_handle *handle, const unsigned char *strContentBuffer,
                               unsigned int uintContentSize, unsigned char strMd5Result[MD5_DIGEST_SIZE]);

/**
 * @fn bool md5_calculate_file(md5_context_handle *handle, const char *strFilePath, unsigned char
 * strMd5Result[MD5_DIGEST_SIZE])
 * @brief Calculates the MD5 hash for a file.
 *
 * Computes the MD5 hash for the contents of the specified file and stores the result in the provided buffer.
 *
 * @param handle The MD5 context handle.
 * @param strFilePath The path to the file.
 * @param strMd5Result Pointer to a buffer of size MD5_DIGEST_SIZE to store the computed MD5 hash.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool md5_calculate_file(md5_context_handle *handle, const char *strFilePath,
                                    unsigned char strMd5Result[MD5_DIGEST_SIZE]);

/**
 * @class CMd5Helper
 * @brief Helper class for MD5 hash calculations.
 *
 * This class provides a convenient interface for performing MD5 hash calculations of buffers and files.
 * It manages the MD5 context handle internally and offers methods to calculate MD5 hashes.
 */
class CMd5Helper {
public:
    /**
     * @brief Constructor for CMd5Helper.
     *
     * Initializes the MD5 helper by creating an MD5 context handle.
     */
    CMd5Helper() { m_hMd5ContextHandle = md5_context_create(); }

    /**
     * @brief Destructor for CMd5Helper.
     *
     * Cleans up the MD5 helper by destroying the MD5 context handle.
     */
    virtual ~CMd5Helper() { md5_context_destroy(m_hMd5ContextHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CMd5Helper objects to ensure proper management of the MD5 context handle.
     */
    CMd5Helper(const CMd5Helper &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CMd5Helper objects to ensure proper management of the MD5 context handle.
     */
    CMd5Helper &operator=(const CMd5Helper &) = delete;
#endif

public:
    /**
     * @brief Calculates the MD5 hash for a buffer.
     *
     * Computes the MD5 hash for the given content buffer and returns it as a hexadecimal string.
     *
     * @param strContentBuffer Pointer to the buffer containing the data.
     * @param uintContentSize The size of the content buffer in bytes.
     * @return The computed MD5 hash as a hexadecimal string.
     */
    string calculate(const unsigned char *strContentBuffer, unsigned int uintContentSize) {
        unsigned char strMd5Result[MD5_DIGEST_SIZE];
        md5_calculate(m_hMd5ContextHandle, strContentBuffer, uintContentSize, strMd5Result);
        string strMd5;
        for (int i = 0; i < MD5_DIGEST_SIZE; i++) strMd5 += CStringAssistant::format("%02X", strMd5Result[i]);
        return strMd5;
    }

    /**
     * @brief Calculates the MD5 hash for a file.
     *
     * Computes the MD5 hash for the contents of the specified file and stores it in the provided string.
     *
     * @param strFilePath The path to the file.
     * @param strMd5 Reference to a string to store the computed MD5 hash as a hexadecimal string.
     * @return true if the operation was successful, false otherwise.
     */
    bool calculate_file(const char *strFilePath, string &strMd5) {
        unsigned char strMd5Result[MD5_DIGEST_SIZE];
        strMd5.clear();
        if (md5_calculate_file(m_hMd5ContextHandle, strFilePath, strMd5Result) == true) {
            for (int i = 0; i < MD5_DIGEST_SIZE; i++) strMd5 += CStringAssistant::format("%02X", strMd5Result[i]);
            return true;
        } else
            return false;
    }

protected:
    /**
     * @var md5_context_handle *m_hMd5ContextHandle
     * @brief Opaque handle for managing the MD5 context.
     *
     * This handle is used internally by the CMd5Helper class to manage the MD5 context.
     */
    md5_context_handle *m_hMd5ContextHandle;
};

#endif
