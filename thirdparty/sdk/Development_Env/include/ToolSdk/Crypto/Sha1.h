/**
 * @file Sha1.h
 * @brief Header file for SHA-1 (Secure Hash Algorithm 1) functionality.
 *
 * This file defines structures and functions for performing SHA-1 hash calculations.
 * It includes helper functions for calculating SHA-1 hashes of buffers and files.
 */

#ifndef __TOOLSDK_SHA1_H__
#define __TOOLSDK_SHA1_H__

#include "ToolSdkDef.h"
#include "StringAssistant/StringAssistant.h"

/**
 * @def SHA1_DIGEST_SIZE
 * @brief The size of the SHA-1 digest in bytes.
 */
#define SHA1_DIGEST_SIZE 20

/**
 * @struct sha1_context_handle
 * @brief Opaque handle for managing SHA-1 contexts.
 *
 * This structure represents an opaque handle used internally for managing SHA-1 contexts.
 */
struct sha1_context_handle;

/**
 * @fn sha1_context_handle *sha1_context_create()
 * @brief Creates a new SHA-1 context handle.
 *
 * Allocates and initializes a new SHA-1 context handle for use in subsequent SHA-1 calculations.
 *
 * @return A pointer to the newly created SHA-1 context handle.
 */
TOOLSDK_API sha1_context_handle *sha1_context_create();

/**
 * @fn void sha1_context_destroy(sha1_context_handle *&handle)
 * @brief Destroys a SHA-1 context handle.
 *
 * Frees the resources associated with the given SHA-1 context handle.
 *
 * @param handle A reference to the SHA-1 context handle to be destroyed.
 */
TOOLSDK_API void sha1_context_destroy(sha1_context_handle *&handle);

/**
 * @fn void sha1_calculate(sha1_context_handle *handle, const unsigned char *strContentBuffer, unsigned int
 * uintContentSize, unsigned char strSha1Result[SHA1_DIGEST_SIZE])
 * @brief Calculates the SHA-1 hash for a buffer.
 *
 * Computes the SHA-1 hash for the given content buffer and stores the result in the provided buffer.
 *
 * @param handle The SHA-1 context handle.
 * @param strContentBuffer Pointer to the buffer containing the data.
 * @param uintContentSize The size of the content buffer in bytes.
 * @param strSha1Result Pointer to a buffer of size SHA1_DIGEST_SIZE to store the computed SHA-1 hash.
 */
TOOLSDK_API void sha1_calculate(sha1_context_handle *handle, const unsigned char *strContentBuffer,
                                unsigned int uintContentSize, unsigned char strSha1Result[SHA1_DIGEST_SIZE]);

/**
 * @fn bool sha1_calculate_file(sha1_context_handle *handle, const char *strFilePath, unsigned char
 * strSha1Result[SHA1_DIGEST_SIZE])
 * @brief Calculates the SHA-1 hash for a file.
 *
 * Computes the SHA-1 hash for the contents of the specified file and stores the result in the provided buffer.
 *
 * @param handle The SHA-1 context handle.
 * @param strFilePath The path to the file.
 * @param strSha1Result Pointer to a buffer of size SHA1_DIGEST_SIZE to store the computed SHA-1 hash.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool sha1_calculate_file(sha1_context_handle *handle, const char *strFilePath,
                                     unsigned char strSha1Result[SHA1_DIGEST_SIZE]);

/**
 * @class CSha1Helper
 * @brief Helper class for SHA-1 hash calculations.
 *
 * This class provides a convenient interface for performing SHA-1 hash calculations of buffers and files.
 * It manages the SHA-1 context handle internally and offers methods to calculate SHA-1 hashes.
 */
class CSha1Helper {
public:
    /**
     * @brief Constructor for CSha1Helper.
     *
     * Initializes the SHA-1 helper by creating a SHA-1 context handle.
     */
    CSha1Helper() { m_hSha1ContextHandle = sha1_context_create(); }

    /**
     * @brief Destructor for CSha1Helper.
     *
     * Cleans up the SHA-1 helper by destroying the SHA-1 context handle.
     */
    virtual ~CSha1Helper() { sha1_context_destroy(m_hSha1ContextHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CSha1Helper objects to ensure proper management of the SHA-1 context handle.
     */
    CSha1Helper(const CSha1Helper &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CSha1Helper objects to ensure proper management of the SHA-1 context handle.
     */
    CSha1Helper &operator=(const CSha1Helper &) = delete;
#endif

public:
    /**
     * @brief Calculates the SHA-1 hash for a buffer.
     *
     * Computes the SHA-1 hash for the given content buffer and returns it as a hexadecimal string.
     *
     * @param strContentBuffer Pointer to the buffer containing the data.
     * @param uintContentSize The size of the content buffer in bytes.
     * @return The computed SHA-1 hash as a hexadecimal string.
     */
    string calculate(const unsigned char *strContentBuffer, unsigned int uintContentSize) {
        unsigned char strSha1Result[SHA1_DIGEST_SIZE];
        sha1_calculate(m_hSha1ContextHandle, strContentBuffer, uintContentSize, strSha1Result);
        string strSha1;
        for (int i = 0; i < SHA1_DIGEST_SIZE; i++) strSha1 += CStringAssistant::format("%02X", strSha1Result[i]);
        return strSha1;
    }

    /**
     * @brief Calculates the SHA-1 hash for a file.
     *
     * Computes the SHA-1 hash for the contents of the specified file and stores it in the provided string.
     *
     * @param strFilePath The path to the file.
     * @param strSha1 Reference to a string to store the computed SHA-1 hash as a hexadecimal string.
     * @return true if the operation was successful, false otherwise.
     */
    bool calculate_file(const char *strFilePath, string &strSha1) {
        unsigned char strSha1Result[SHA1_DIGEST_SIZE];
        strSha1.clear();
        if (sha1_calculate_file(m_hSha1ContextHandle, strFilePath, strSha1Result) == true) {
            for (int i = 0; i < SHA1_DIGEST_SIZE; i++) strSha1 += CStringAssistant::format("%02X", strSha1Result[i]);
            return true;
        } else
            return false;
    }

protected:
    /**
     * @var sha1_context_handle *m_hSha1ContextHandle
     * @brief Opaque handle for managing the SHA-1 context.
     *
     * This handle is used internally by the CSha1Helper class to manage the SHA-1 context.
     */
    sha1_context_handle *m_hSha1ContextHandle;
};

#endif  // __TOOLSDK_SHA1_H__