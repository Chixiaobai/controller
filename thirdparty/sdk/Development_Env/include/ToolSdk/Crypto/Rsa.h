#ifndef __TOOLSDK_RSA_H__
#define __TOOLSDK_RSA_H__

#include "ToolSdkDef.h"

/**
 * @struct rsa_context_handle
 * @brief Opaque handle for managing RSA contexts.
 *
 * This structure represents an opaque handle used internally for managing RSA contexts.
 */
struct rsa_context_handle;

/**
 * @struct rsa_stream_handle
 * @brief Opaque handle for managing RSA stream data.
 *
 * This structure represents an opaque handle used internally for managing RSA stream data, such as encrypted or
 * decrypted content.
 */
struct rsa_stream_handle;

/**
 * @fn rsa_context_handle *rsa_context_create()
 * @brief Creates a new RSA context handle.
 *
 * Allocates and initializes a new RSA context handle for use in subsequent RSA operations.
 *
 * @return A pointer to the newly created RSA context handle.
 */
TOOLSDK_API rsa_context_handle *rsa_context_create();

/**
 * @fn void rsa_context_destroy(rsa_context_handle *&handle)
 * @brief Destroys an RSA context handle.
 *
 * Frees the resources associated with the given RSA context handle.
 *
 * @param handle A reference to the RSA context handle to be destroyed.
 */
TOOLSDK_API void rsa_context_destroy(rsa_context_handle *&handle);

/**
 * @fn bool rsa_generate_keyfile(rsa_context_handle *handle, const char *pubkeypemfile, const char *prikeypemfile, int
 * nbits)
 * @brief Generates an RSA key pair and saves them to files.
 *
 * Generates a new RSA key pair (public and private keys) and saves them to the specified files in PEM format.
 *
 * @param handle The RSA context handle.
 * @param pubkeypemfile The path to the file where the public key will be saved.
 * @param prikeypemfile The path to the file where the private key will be saved.
 * @param nbits The length of the key in bits (e.g., 1024, 2048).
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rsa_generate_keyfile(rsa_context_handle *handle, const char *pubkeypemfile, const char *prikeypemfile,
                                      int nbits);

/**
 * @fn bool rsa_generate_keystring(rsa_context_handle *handle, char *pubkeypemstring, char *prikeypemstring, int nbits)
 * @brief Generates an RSA key pair and returns them as strings.
 *
 * Generates a new RSA key pair (public and private keys) and returns them as strings in PEM format.
 *
 * @param handle The RSA context handle.
 * @param pubkeypemstring Pointer to a buffer to store the public key in PEM format.
 * @param prikeypemstring Pointer to a buffer to store the private key in PEM format.
 * @param nbits The length of the key in bits (e.g., 1024, 2048).
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rsa_generate_keystring(rsa_context_handle *handle, char *pubkeypemstring, char *prikeypemstring,
                                        int nbits);

/**
 * @fn rsa_stream_handle *rsa_encrypt_by_pubkey_string(rsa_context_handle *handle, const char *pubkeypemstring, const
 * char *plaintext, const unsigned int plaintextsize)
 * @brief Encrypts a plaintext using a public key provided as a string.
 *
 * Encrypts the provided plaintext using the RSA public key and returns the encrypted data as an RSA stream handle.
 *
 * @param handle The RSA context handle.
 * @param pubkeypemstring The public key in PEM format.
 * @param plaintext Pointer to the buffer containing the plaintext data.
 * @param plaintextsize The size of the plaintext buffer in bytes.
 * @return A pointer to the RSA stream handle containing the encrypted data.
 */
TOOLSDK_API rsa_stream_handle *rsa_encrypt_by_pubkey_string(rsa_context_handle *handle, const char *pubkeypemstring,
                                                            const char *plaintext, const unsigned int plaintextsize);

/**
 * @fn rsa_stream_handle *rsa_decrypt_by_prikey_string(rsa_context_handle *handle, const char *prikeypemstring, const
 * char *ciphertext, const unsigned int ciphertextsize)
 * @brief Decrypts a ciphertext using a private key provided as a string.
 *
 * Decrypts the provided ciphertext using the RSA private key and returns the decrypted data as an RSA stream handle.
 *
 * @param handle The RSA context handle.
 * @param prikeypemstring The private key in PEM format.
 * @param ciphertext Pointer to the buffer containing the ciphertext data.
 * @param ciphertextsize The size of the ciphertext buffer in bytes.
 * @return A pointer to the RSA stream handle containing the decrypted data.
 */
TOOLSDK_API rsa_stream_handle *rsa_decrypt_by_prikey_string(rsa_context_handle *handle, const char *prikeypemstring,
                                                            const char *ciphertext, const unsigned int ciphertextsize);

/**
 * @fn rsa_stream_handle *rsa_encrypt_by_pubkey_file(rsa_context_handle *handle, const char *pubkeypemfile, const char
 * *plaintext, const unsigned int plaintextsize)
 * @brief Encrypts a plaintext using a public key provided in a file.
 *
 * Encrypts the provided plaintext using the RSA public key from the specified file and returns the encrypted data as an
 * RSA stream handle.
 *
 * @param handle The RSA context handle.
 * @param pubkeypemfile The path to the file containing the public key in PEM format.
 * @param plaintext Pointer to the buffer containing the plaintext data.
 * @param plaintextsize The size of the plaintext buffer in bytes.
 * @return A pointer to the RSA stream handle containing the encrypted data.
 */
TOOLSDK_API rsa_stream_handle *rsa_encrypt_by_pubkey_file(rsa_context_handle *handle, const char *pubkeypemfile,
                                                          const char *plaintext, const unsigned int plaintextsize);

/**
 * @fn rsa_stream_handle *rsa_decrypt_by_prikey_file(rsa_context_handle *handle, const char *prikeypemfile, const char
 * *ciphertext, const unsigned int ciphertextsize)
 * @brief Decrypts a ciphertext using a private key provided in a file.
 *
 * Decrypts the provided ciphertext using the RSA private key from the specified file and returns the decrypted data as
 * an RSA stream handle.
 *
 * @param handle The RSA context handle.
 * @param prikeypemfile The path to the file containing the private key in PEM format.
 * @param ciphertext Pointer to the buffer containing the ciphertext data.
 * @param ciphertextsize The size of the ciphertext buffer in bytes.
 * @return A pointer to the RSA stream handle containing the decrypted data.
 */
TOOLSDK_API rsa_stream_handle *rsa_decrypt_by_prikey_file(rsa_context_handle *handle, const char *prikeypemfile,
                                                          const char *ciphertext, const unsigned int ciphertextsize);

/**
 * @fn unsigned int rsa_get_stream_size(rsa_stream_handle *handle)
 * @brief Retrieves the size of the data in an RSA stream handle.
 *
 * Returns the size of the data stored in the RSA stream handle.
 *
 * @param handle The RSA stream handle.
 * @return The size of the data in bytes.
 */
TOOLSDK_API unsigned int rsa_get_stream_size(rsa_stream_handle *handle);

/**
 * @fn bool rsa_get_stream_content(rsa_stream_handle *handle, char *content, const unsigned int content_size)
 * @brief Retrieves the content of an RSA stream handle.
 *
 * Copies the content of the RSA stream handle to the provided buffer.
 *
 * @param handle The RSA stream handle.
 * @param content Pointer to the buffer to store the content.
 * @param content_size The size of the buffer in bytes.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool rsa_get_stream_content(rsa_stream_handle *handle, char *content, const unsigned int content_size);

/**
 * @fn void rsa_destroy_stream_handle(rsa_stream_handle *&handle)
 * @brief Destroys an RSA stream handle.
 *
 * Frees the resources associated with the given RSA stream handle.
 *
 * @param handle A reference to the RSA stream handle to be destroyed.
 */
TOOLSDK_API void rsa_destroy_stream_handle(rsa_stream_handle *&handle);

/**
 * @class CRsaStream
 * @brief Class for managing RSA stream data.
 *
 * This class provides a convenient interface for handling RSA stream data, such as encrypted or decrypted content.
 * It manages the RSA stream handle internally and offers methods to retrieve the size and content of the stream.
 */
class CRsaStream {
public:
    /**
     * @brief Constructor for CRsaStream.
     *
     * Initializes the RSA stream by taking ownership of the provided RSA stream handle.
     *
     * @param hRsaStreamHandle The RSA stream handle.
     */
    CRsaStream(rsa_stream_handle *hRsaStreamHandle) { m_hRsaStreamHandle = hRsaStreamHandle; }

    /**
     * @brief Destructor for CRsaStream.
     *
     * Cleans up the RSA stream by destroying the RSA stream handle.
     */
    virtual ~CRsaStream() { rsa_destroy_stream_handle(m_hRsaStreamHandle); }

public:
    /**
     * @brief Retrieves the size of the data in the RSA stream.
     *
     * Returns the size of the data stored in the RSA stream.
     *
     * @return The size of the data in bytes.
     */
    unsigned int getStreamSize() { return rsa_get_stream_size(m_hRsaStreamHandle); }

    /**
     * @brief Retrieves the content of the RSA stream.
     *
     * Copies the content of the RSA stream to the provided buffer.
     *
     * @param content Pointer to the buffer to store the content.
     * @param content_size The size of the buffer in bytes.
     * @return true if the operation was successful, false otherwise.
     */
    bool getStreamContent(char *content, const unsigned int content_size) {
        return rsa_get_stream_content(m_hRsaStreamHandle, content, content_size);
    }

protected:
    /**
     * @var rsa_stream_handle *m_hRsaStreamHandle
     * @brief Opaque handle for managing the RSA stream.
     *
     * This handle is used internally by the CRsaStream class to manage the RSA stream data.
     */
    rsa_stream_handle *m_hRsaStreamHandle;
};

/**
 * @class CRsaHelper
 * @brief Helper class for RSA encryption and decryption.
 *
 * This class provides a convenient interface for performing RSA encryption and decryption operations.
 * It manages the RSA context handle internally and offers methods to generate key pairs and encrypt/decrypt data.
 */
class CRsaHelper {
public:
    /**
     * @brief Constructor for CRsaHelper.
     *
     * Initializes the RSA helper by creating an RSA context handle.
     */
    CRsaHelper() { m_hRsaContextHandle = rsa_context_create(); }

    /**
     * @brief Destructor for CRsaHelper.
     *
     * Cleans up the RSA helper by destroying the RSA context handle.
     */
    virtual ~CRsaHelper() { rsa_context_destroy(m_hRsaContextHandle); }

public:
#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CRsaHelper objects to ensure proper management of the RSA context handle.
     */
    CRsaHelper(const CRsaHelper &) = delete;

    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CRsaHelper objects to ensure proper management of the RSA context handle.
     */
    CRsaHelper &operator=(const CRsaHelper &) = delete;
#endif

    /**
     * @brief Generates an RSA key pair and saves them to files.
     *
     * Generates a new RSA key pair (public and private keys) and saves them to the specified files in PEM format.
     *
     * @param pubkeypemfile The path to the file where the public key will be saved.
     * @param prikeypemfile The path to the file where the private key will be saved.
     * @param nbits The length of the key in bits (default is 1024, generally recommended to be 1024 or higher).
     * @return true if the operation was successful, false otherwise.
     */
    bool generateKeyPairFile(const char *pubkeypemfile, const char *prikeypemfile, int nbits = 1024) {
        return rsa_generate_keyfile(m_hRsaContextHandle, pubkeypemfile, prikeypemfile, nbits);
    }

    /**
     * @brief Generates an RSA key pair and returns them as strings.
     *
     * Generates a new RSA key pair (public and private keys) and returns them as strings in PEM format.
     *
     * @param pubkeypemstring Pointer to a buffer to store the public key in PEM format.
     * @param prikeypemstring Pointer to a buffer to store the private key in PEM format.
     * @param nbits The length of the key in bits (default is 1024, generally recommended to be 1024 or higher).
     * @return true if the operation was successful, false otherwise.
     */
    bool generateKeyPairString(char *pubkeypemstring, char *prikeypemstring, int nbits = 1024) {
        return rsa_generate_keystring(m_hRsaContextHandle, pubkeypemstring, prikeypemstring, nbits);
    }

    /**
     * @brief Encrypts a plaintext using a public key provided as a string.
     *
     * Encrypts the provided plaintext using the RSA public key and returns the encrypted data as an RSA stream.
     *
     * @param pubkeypemstring The public key in PEM format.
     * @param plaintext Pointer to the buffer containing the plaintext data.
     * @param plaintextsize The size of the plaintext buffer in bytes.
     * @return A unique pointer to the CRsaStream object containing the encrypted data.
     */
    std::unique_ptr<CRsaStream> encryptByPubkeyString(const char *pubkeypemstring, const char *plaintext,
                                                      const unsigned int plaintextsize) {
        rsa_stream_handle *streamObj =
            rsa_encrypt_by_pubkey_string(m_hRsaContextHandle, pubkeypemstring, plaintext, plaintextsize);
        return std::unique_ptr<CRsaStream>(new CRsaStream(streamObj));
    }

    /**
     * @brief Decrypts a ciphertext using a private key provided as a string.
     *
     * Decrypts the provided ciphertext using the RSA private key and returns the decrypted data as an RSA stream.
     *
     * @param prikeypemstring The private key in PEM format.
     * @param ciphertext Pointer to the buffer containing the ciphertext data.
     * @param ciphertextsize The size of the ciphertext buffer in bytes.
     * @return A unique pointer to the CRsaStream object containing the decrypted data.
     */
    std::unique_ptr<CRsaStream> decryptByPrikeyString(const char *prikeypemstring, const char *ciphertext,
                                                      const unsigned int ciphertextsize) {
        rsa_stream_handle *streamObj =
            rsa_decrypt_by_prikey_string(m_hRsaContextHandle, prikeypemstring, ciphertext, ciphertextsize);
        return std::unique_ptr<CRsaStream>(new CRsaStream(streamObj));
    }

    /**
     * @brief Encrypts a plaintext using a public key provided in a file.
     *
     * Encrypts the provided plaintext using the RSA public key from the specified file and returns the encrypted data
     * as an RSA stream.
     *
     * @param pubkeypemfile The path to the file containing the public key in PEM format.
     * @param plaintext Pointer to the buffer containing the plaintext data.
     * @param plaintextsize The size of the plaintext buffer in bytes.
     * @return A unique pointer to the CRsaStream object containing the encrypted data.
     */
    std::unique_ptr<CRsaStream> encryptByPubkeyFile(const char *pubkeypemfile, const char *plaintext,
                                                    const unsigned int plaintextsize) {
        rsa_stream_handle *streamObj =
            rsa_encrypt_by_pubkey_file(m_hRsaContextHandle, pubkeypemfile, plaintext, plaintextsize);
        return std::unique_ptr<CRsaStream>(new CRsaStream(streamObj));
    }

    /**
     * @brief Decrypts a ciphertext using a private key provided in a file.
     *
     * Decrypts the provided ciphertext using the RSA private key from the specified file and returns the decrypted data
     * as an RSA stream.
     *
     * @param prikeypemfile The path to the file containing the private key in PEM format.
     * @param ciphertext Pointer to the buffer containing the ciphertext data.
     * @param ciphertextsize The size of the ciphertext buffer in bytes.
     * @return A unique pointer to the CRsaStream object containing the decrypted data.
     */
    std::unique_ptr<CRsaStream> decryptByPrikeyFile(const char *prikeypemfile, const char *ciphertext,
                                                    const unsigned int ciphertextsize) {
        rsa_stream_handle *streamObj =
            rsa_decrypt_by_prikey_file(m_hRsaContextHandle, prikeypemfile, ciphertext, ciphertextsize);
        return std::unique_ptr<CRsaStream>(new CRsaStream(streamObj));
    }

protected:
    /**
     * @var rsa_context_handle *m_hRsaContextHandle
     * @brief Opaque handle for managing the RSA context.
     *
     * This handle is used internally by the CRsaHelper class to manage the RSA context.
     */
    rsa_context_handle *m_hRsaContextHandle;
};

#endif
