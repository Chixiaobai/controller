#ifndef __TOOLSDK_RC4_H__
#define __TOOLSDK_RC4_H__

#include "ToolSdkDef.h"

/**
 * @struct rc4_context_handle
 * @brief Opaque handle for managing RC4 contexts.
 *
 * This structure represents an opaque handle used internally for managing RC4 contexts.
 */
struct rc4_context_handle;

/**
 * @fn rc4_context_handle *rc4_context_create(unsigned int len, const unsigned char *key = nullptr)
 * @brief Creates a new RC4 context handle.
 *
 * Allocates and initializes a new RC4 context handle with the specified key length and key.
 *
 * @param len The length of the key in bytes.
 * @param key Pointer to the key buffer (optional, default is nullptr).
 * @return A pointer to the newly created RC4 context handle.
 */
TOOLSDK_API rc4_context_handle *rc4_context_create(unsigned int len, const unsigned char *key = nullptr);

/**
 * @fn void rc4_context_destroy(rc4_context_handle *&handle)
 * @brief Destroys an RC4 context handle.
 *
 * Frees the resources associated with the given RC4 context handle.
 *
 * @param handle A reference to the RC4 context handle to be destroyed.
 */
TOOLSDK_API void rc4_context_destroy(rc4_context_handle *&handle);

/**
 * @fn bool rc4_get_key(rc4_context_handle *handle, unsigned int len, unsigned char *key)
 * @brief Retrieves the key from an RC4 context handle.
 *
 * Copies the key from the RC4 context handle to the provided buffer.
 *
 * @param handle The RC4 context handle.
 * @param len The length of the key buffer.
 * @param key Pointer to the buffer to store the key.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rc4_get_key(rc4_context_handle *handle, unsigned int len, unsigned char *key);

/**
 * @fn bool rc4_set_key(rc4_context_handle *handle, unsigned int len, const unsigned char *key = nullptr)
 * @brief Sets the key for an RC4 context handle.
 *
 * Sets the key for the RC4 context handle using the provided key buffer.
 *
 * @param handle The RC4 context handle.
 * @param len The length of the key buffer.
 * @param key Pointer to the key buffer (optional, default is nullptr).
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rc4_set_key(rc4_context_handle *handle, unsigned int len, const unsigned char *key = nullptr);

/**
 * @fn bool rc4_encryption(rc4_context_handle *handle, const unsigned char *strPlainBuffer, const unsigned int
 * uintBufferSize, unsigned char *strCipherBuffer)
 * @brief Encrypts a buffer using the RC4 algorithm.
 *
 * Encrypts the provided plain buffer using the RC4 algorithm and stores the result in the cipher buffer.
 *
 * @param handle The RC4 context handle.
 * @param strPlainBuffer Pointer to the buffer containing the plain data.
 * @param uintBufferSize The size of the plain buffer in bytes.
 * @param strCipherBuffer Pointer to the buffer to store the encrypted data.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rc4_encryption(rc4_context_handle *handle, const unsigned char *strPlainBuffer,
                                const unsigned int uintBufferSize, unsigned char *strCipherBuffer);

/**
 * @fn bool rc4_decryption(rc4_context_handle *handle, const unsigned char *strCipherBuffer, const unsigned int
 * uintBufferSize, unsigned char *strPlainBuffer)
 * @brief Decrypts a buffer using the RC4 algorithm.
 *
 * Decrypts the provided cipher buffer using the RC4 algorithm and stores the result in the plain buffer.
 *
 * @param handle The RC4 context handle.
 * @param strCipherBuffer Pointer to the buffer containing the encrypted data.
 * @param uintBufferSize The size of the cipher buffer in bytes.
 * @param strPlainBuffer Pointer to the buffer to store the decrypted data.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rc4_decryption(rc4_context_handle *handle, const unsigned char *strCipherBuffer,
                                const unsigned int uintBufferSize, unsigned char *strPlainBuffer);

/**
 * @fn bool rc4_encryptio_file(rc4_context_handle *handle, const char *strPlainFilePath, const char *strCipherFilePath)
 * @brief Encrypts a file using the RC4 algorithm.
 *
 * Encrypts the contents of the specified plain file using the RC4 algorithm and saves the result to the cipher file.
 *
 * @param handle The RC4 context handle.
 * @param strPlainFilePath The path to the plain file.
 * @param strCipherFilePath The path to the cipher file.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rc4_encryptio_file(rc4_context_handle *handle, const char *strPlainFilePath,
                                    const char *strCipherFilePath);

/**
 * @fn bool rc4_decryption_file(rc4_context_handle *handle, const char *strCipherFilePath, const char *strPlainFilePath)
 * @brief Decrypts a file using the RC4 algorithm.
 *
 * Decrypts the contents of the specified cipher file using the RC4 algorithm and saves the result to the plain file.
 *
 * @param handle The RC4 context handle.
 * @param strCipherFilePath The path to the cipher file.
 * @param strPlainFilePath The path to the plain file.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rc4_decryption_file(rc4_context_handle *handle, const char *strCipherFilePath,
                                     const char *strPlainFilePath);

/**
 * @class CRc4Helper
 * @brief Helper class for RC4 encryption and decryption.
 *
 * This class provides a convenient interface for performing RC4 encryption and decryption operations.
 * It manages the RC4 context handle internally and offers methods to encrypt and decrypt buffers and files.
 */
class CRc4Helper {
public:
    /**
     * @brief Constructor for CRc4Helper.
     *
     * Initializes the RC4 helper by creating an RC4 context handle with the specified key length and key.
     *
     * @param len The length of the key in bytes (default is 128).
     * @param key Pointer to the key buffer (optional, default is nullptr).
     */
    CRc4Helper(int len = 128, const unsigned char *key = nullptr) {
        m_hRc4ContextHandle = rc4_context_create(len, key);
    }

    /**
     * @brief Destructor for CRc4Helper.
     *
     * Cleans up the RC4 helper by destroying the RC4 context handle.
     */
    virtual ~CRc4Helper() { rc4_context_destroy(m_hRc4ContextHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CRc4Helper objects to ensure proper management of the RC4 context handle.
     */
    CRc4Helper(const CRc4Helper &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CRc4Helper objects to ensure proper management of the RC4 context handle.
     */
    CRc4Helper &operator=(const CRc4Helper &) = delete;
#endif

public:
    /**
     * @brief Retrieves the key from the RC4 context handle.
     *
     * Copies the key from the RC4 context handle to the provided buffer.
     *
     * @param len The length of the key buffer.
     * @param key Pointer to the buffer to store the key.
     * @return true if the operation was successful, false otherwise.
     */
    bool get_key(unsigned int len, unsigned char *key) { return rc4_get_key(m_hRc4ContextHandle, len, key); }

    /**
     * @brief Sets the key for the RC4 context handle.
     *
     * Sets the key for the RC4 context handle using the provided key buffer.
     *
     * @param len The length of the key buffer.
     * @param key Pointer to the key buffer (optional, default is nullptr).
     * @return true if the operation was successful, false otherwise.
     */
    bool set_key(unsigned int len, const unsigned char *key = nullptr) {
        return rc4_set_key(m_hRc4ContextHandle, len, key);
    }

    /**
     * @brief Encrypts a buffer using the RC4 algorithm.
     *
     * Encrypts the provided plain buffer using the RC4 algorithm and stores the result in the cipher buffer.
     *
     * @param strPlainBuffer Pointer to the buffer containing the plain data.
     * @param uintBufferSize The size of the plain buffer in bytes.
     * @param strCipherBuffer Pointer to the buffer to store the encrypted data.
     * @return true if the operation was successful, false otherwise.
     */
    bool encryption(const unsigned char *strPlainBuffer, const unsigned int uintBufferSize,
                    unsigned char *strCipherBuffer) {
        return rc4_encryption(m_hRc4ContextHandle, strPlainBuffer, uintBufferSize, strCipherBuffer);
    }

    /**
     * @brief Decrypts a buffer using the RC4 algorithm.
     *
     * Decrypts the provided cipher buffer using the RC4 algorithm and stores the result in the plain buffer.
     *
     * @param strCipherBuffer Pointer to the buffer containing the encrypted data.
     * @param uintBufferSize The size of the cipher buffer in bytes.
     * @param strPlainBuffer Pointer to the buffer to store the decrypted data.
     * @return true if the operation was successful, false otherwise.
     */
    bool decryption(const unsigned char *strCipherBuffer, const unsigned int uintBufferSize,
                    unsigned char *strPlainBuffer) {
        return rc4_decryption(m_hRc4ContextHandle, strCipherBuffer, uintBufferSize, strPlainBuffer);
    }

    /**
     * @brief Encrypts a file using the RC4 algorithm.
     *
     * Encrypts the contents of the specified plain file using the RC4 algorithm and saves the result to the cipher
     * file.
     *
     * @param strPlainFilePath The path to the plain file.
     * @param strCipherFilePath The path to the cipher file.
     * @return true if the operation was successful, false otherwise.
     */
    bool encryption_file(const char *strPlainFilePath, const char *strCipherFilePath) {
        return rc4_encryptio_file(m_hRc4ContextHandle, strPlainFilePath, strCipherFilePath);
    }

    /**
     * @brief Decrypts a file using the RC4 algorithm.
     *
     * Decrypts the contents of the specified cipher file using the RC4 algorithm and saves the result to the plain
     * file.
     *
     * @param strCipherFilePath The path to the cipher file.
     * @param strPlainFilePath The path to the plain file.
     * @return true if the operation was successful, false otherwise.
     */
    bool decryption_file(const char *strCipherFilePath, const char *strPlainFilePath) {
        return rc4_decryption_file(m_hRc4ContextHandle, strCipherFilePath, strPlainFilePath);
    }

protected:
    /**
     * @var rc4_context_handle *m_hRc4ContextHandle
     * @brief Opaque handle for managing the RC4 context.
     *
     * This handle is used internally by the CRc4Helper class to manage the RC4 context.
     */
    rc4_context_handle *m_hRc4ContextHandle;
};

#endif
