#ifndef __TOOLSDK_BOUNDED_STREAM_H__
#define __TOOLSDK_BOUNDED_STREAM_H__

#include "ToolSdkDef.h"
#include "BoundedStream/AbstractStream.h"

// Forward declaration for binary stream handle
struct binary_stream_handle;

/**
 * @brief Creates a binary stream handle.
 *
 * Allocates and initializes a binary stream with a specified bounded size and block size.
 *
 * @param uBoundedSize The maximum size of the stream.
 * @param uBlockSize The size of each block in the stream.
 * @return A pointer to the created binary stream handle.
 */
TOOLSDK_API binary_stream_handle *binary_stream_create(size_t uBoundedSize, size_t uBlockSize);

/**
 * @brief Destroys a binary stream handle.
 *
 * Frees the resources associated with the binary stream handle.
 *
 * @param handle Reference to the binary stream handle to be destroyed.
 */
TOOLSDK_API void binary_stream_destroy(binary_stream_handle *&handle);

/**
 * @brief Gets the length of a binary stream.
 *
 * Returns the current length of the binary stream.
 *
 * @param handle The binary stream handle.
 * @return The length of the binary stream.
 */
TOOLSDK_API size_t binary_stream_get_length(binary_stream_handle *handle);

/**
 * @brief Reads data from a binary stream.
 *
 * Reads a specified number of bytes from the binary stream into a buffer.
 *
 * @param handle The binary stream handle.
 * @param pBuf Pointer to the buffer where the data will be stored.
 * @param uReadLen Number of bytes to read from the stream.
 * @return The result of the read operation.
 */
TOOLSDK_API int binary_stream_read(binary_stream_handle *handle, void *pBuf, size_t uReadLen);

/**
 * @brief Writes data to a binary stream.
 *
 * Writes a specified number of bytes from a buffer into the binary stream.
 *
 * @param handle The binary stream handle.
 * @param pBuf Pointer to the buffer containing the data to be written.
 * @param uWriteLen Number of bytes to write to the stream.
 * @return The result of the write operation.
 */
TOOLSDK_API int binary_stream_write(binary_stream_handle *handle, const void *pBuf, const size_t uWriteLen);

/**
 * @brief Checks if there is enough readable space in a binary stream.
 *
 * Determines if the binary stream has sufficient space to read the specified number of bytes.
 *
 * @param handle The binary stream handle.
 * @param uRequireSpace The number of bytes required for reading.
 * @return true if there is enough space to read the required number of bytes, false otherwise.
 */
TOOLSDK_API bool binary_stream_enough_readable(binary_stream_handle *handle, size_t uRequireSpace);

/**
 * @brief Checks if there is enough writable space in a binary stream.
 *
 * Determines if the binary stream has sufficient space to write the specified number of bytes.
 *
 * @param handle The binary stream handle.
 * @param uRequireSpace The number of bytes required for writing.
 * @return true if there is enough space to write the required number of bytes, false otherwise.
 */
TOOLSDK_API bool binary_stream_enough_writable(binary_stream_handle *handle, size_t uRequireSpace);

// Forward declaration for cache stream handle
struct cache_stream_handle;

/**
 * @brief Creates a cache stream handle.
 *
 * Allocates and initializes a cache stream with an optional specified cache size.
 *
 * @param uCacheSize The initial size of the cache (default is 0).
 * @return A pointer to the created cache stream handle.
 */
TOOLSDK_API cache_stream_handle *cache_stream_create(size_t uCacheSize = 0);

/**
 * @brief Destroys a cache stream handle.
 *
 * Frees the resources associated with the cache stream handle.
 *
 * @param handle Reference to the cache stream handle to be destroyed.
 */
TOOLSDK_API void cache_stream_destroy(cache_stream_handle *&handle);

/**
 * @brief Attaches a buffer to a cache stream.
 *
 * Attaches an external buffer to the cache stream for data storage.
 *
 * @param handle The cache stream handle.
 * @param pBuf Pointer to the buffer to be attached.
 * @param uCacheSize The size of the buffer.
 * @return true if the buffer was successfully attached, false otherwise.
 */
TOOLSDK_API bool cache_stream_attach(cache_stream_handle *handle, char *pBuf, size_t uCacheSize);

/**
 * @brief Detaches a buffer from a cache stream.
 *
 * Detaches the buffer from the cache stream and returns it.
 *
 * @param handle The cache stream handle.
 * @param uCacheSize Reference to the size of the detached buffer.
 * @return Pointer to the detached buffer.
 */
TOOLSDK_API char *cache_stream_detach(cache_stream_handle *handle, size_t &uCacheSize);

/**
 * @brief Gets the length of a cache stream.
 *
 * Returns the current length of the cache stream.
 *
 * @param handle The cache stream handle.
 * @return The length of the cache stream.
 */
TOOLSDK_API size_t cache_stream_get_length(cache_stream_handle *handle);

/**
 * @brief Reads data from a cache stream.
 *
 * Reads a specified number of bytes from the cache stream into a buffer.
 *
 * @param handle The cache stream handle.
 * @param pBuf Pointer to the buffer where the data will be stored.
 * @param uReadLen Number of bytes to read from the stream.
 * @return The result of the read operation.
 */
TOOLSDK_API int cache_stream_read(cache_stream_handle *handle, void *pBuf, size_t uReadLen);

/**
 * @brief Writes data to a cache stream.
 *
 * Writes a specified number of bytes from a buffer into the cache stream.
 *
 * @param handle The cache stream handle.
 * @param pBuf Pointer to the buffer containing the data to be written.
 * @param uWriteLen Number of bytes to write to the stream.
 * @return The result of the write operation.
 */
TOOLSDK_API int cache_stream_write(cache_stream_handle *handle, const void *pBuf, const size_t uWriteLen);

/**
 * @brief Checks if there is enough readable space in a cache stream.
 *
 * Determines if the cache stream has sufficient space to read the specified number of bytes.
 *
 * @param handle The cache stream handle.
 * @param uRequireSpace The number of bytes required for reading.
 * @return true if there is enough space to read the required number of bytes, false otherwise.
 */
TOOLSDK_API bool cache_stream_enough_readable(cache_stream_handle *handle, size_t uRequireSpace);

/**
 * @brief Checks if there is enough writable space in a cache stream.
 *
 * Determines if the cache stream has sufficient space to write the specified number of bytes.
 *
 * @param handle The cache stream handle.
 * @param uRequireSpace The number of bytes required for writing.
 * @return true if there is enough space to write the required number of bytes, false otherwise.
 */
TOOLSDK_API bool cache_stream_enough_writable(cache_stream_handle *handle, size_t uRequireSpace);

/**
 * @class CNullStream
 * @brief A null stream implementation.
 *
 * This class represents a null stream that does not store any data. It is used for testing
 * and scenarios where data storage is not required.
 */
class CNullStream : public CAbstractStream {
public:
    /**
     * @brief Constructor for CNullStream.
     *
     * Initializes the null stream with a length of 0.
     */
    CNullStream() { m_uStreamLength = 0; }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Destructor for CNullStream.
     *
     * Cleans up the null stream.
     */
    virtual ~CNullStream() = default;

    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CNullStream objects.
     */
    CNullStream(const CNullStream &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CNullStream objects.
     */
    CNullStream &operator=(const CNullStream &) = delete;
#endif

public:
    /**
     * @brief Reads an object from the stream.
     *
     * This method does not actually read any data but simulates a read operation by adjusting
     * the stream length. If there is not enough space, it throws a CStreamException.
     *
     * @param pBuf Pointer to the buffer where the data will be stored (unused).
     * @param uReadLen Number of bytes to read from the stream.
     */
    virtual void readObject(void *pBuf, size_t uReadLen) {
        UNREFERENCED_PARAMETER(pBuf);
        if (m_uStreamLength >= uReadLen) {
            m_uStreamLength -= uReadLen;
        } else
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
    }

    /**
     * @brief Writes an object to the stream.
     *
     * This method does not actually write any data but simulates a write operation by adjusting
     * the stream length.
     *
     * @param pBuf Pointer to the buffer containing the data to be written (unused).
     * @param uReadLen Number of bytes to write to the stream.
     */
    virtual void writeObject(const void *pBuf, const size_t uReadLen) {
        UNREFERENCED_PARAMETER(pBuf);
        m_uStreamLength += uReadLen;
    }

    /**
     * @brief Gets the length of the stream.
     *
     * Returns the current length of the null stream.
     *
     * @return The length of the stream.
     */
    virtual size_t getLength() { return m_uStreamLength; }

protected:
    /**
     * @brief Checks if there is enough readable space in the stream.
     *
     * Determines if the stream has sufficient space to read the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for reading.
     * @return true if there is enough space to read the required number of bytes, false otherwise.
     */
    virtual bool enoughReadable(size_t uRequireSpace) {
        if (m_uStreamLength >= uRequireSpace)
            return true;
        else
            return false;
    }

    /**
     * @brief Checks if there is enough writable space in the stream.
     *
     * Determines if the stream has sufficient space to write the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for writing (unused).
     * @return true if there is enough space to write the required number of bytes, false otherwise.
     */
    virtual bool enoughWritable(size_t uRequireSpace) {
        UNREFERENCED_PARAMETER(uRequireSpace);
        return true;
    }

protected:
    /// The length of the stream.
    size_t m_uStreamLength;
};

/// Default block size for bounded streams.
#define DEFAULT_BLOCK_SIZE 512

/**
 * @class CBoundedStream
 * @brief A bounded stream implementation.
 *
 * This class represents a bounded stream that has a fixed maximum size and block size.
 * It uses a binary stream handle to manage the stream data.
 */
class CBoundedStream : public CAbstractStream {
public:
    /**
     * @brief Constructor for CBoundedStream.
     *
     * Initializes the bounded stream with a specified bounded size and block size.
     *
     * @param uBoundedSize The maximum size of the stream (default is 0).
     * @param uBlockSize The size of each block in the stream (default is DEFAULT_BLOCK_SIZE).
     */
    CBoundedStream(size_t uBoundedSize = 0, size_t uBlockSize = DEFAULT_BLOCK_SIZE) {
        m_hBinaryStreamHandle = binary_stream_create(uBoundedSize, uBlockSize);
    }

    /**
     * @brief Destructor for CBoundedStream.
     *
     * Cleans up the bounded stream by destroying the binary stream handle.
     */
    virtual ~CBoundedStream() { binary_stream_destroy(m_hBinaryStreamHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CBoundedStream objects.
     */
    CBoundedStream(const CBoundedStream &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CBoundedStream objects.
     */
    CBoundedStream &operator=(const CBoundedStream &) = delete;
#endif

public:
    /**
     * @brief Reads an object from the stream.
     *
     * Reads a specified number of bytes from the binary stream into a buffer.
     * Throws a CStreamException if the read operation fails.
     *
     * @param pBuf Pointer to the buffer where the data will be stored.
     * @param uReadLen Number of bytes to read from the stream.
     */
    virtual void readObject(void *pBuf, size_t uReadLen) {
        int ret = binary_stream_read(m_hBinaryStreamHandle, pBuf, uReadLen);
        if (ret == CStreamException::INNER_CRITICAL_ERROR || ret == CStreamException::NOT_ENOUGH_SPACE)
            throw new CStreamException(ret);
    }

    /**
     * @brief Writes an object to the stream.
     *
     * Writes a specified number of bytes from a buffer into the binary stream.
     * Throws a CStreamException if the write operation fails.
     *
     * @param pBuf Pointer to the buffer containing the data to be written.
     * @param uWriteLen Number of bytes to write to the stream.
     */
    virtual void writeObject(const void *pBuf, const size_t uWriteLen) {
        int ret = binary_stream_write(m_hBinaryStreamHandle, pBuf, uWriteLen);
        if (ret == CStreamException::INNER_CRITICAL_ERROR || ret == CStreamException::NOT_ENOUGH_SPACE)
            throw new CStreamException(ret);
    }

    /**
     * @brief Gets the length of the stream.
     *
     * Returns the current length of the bounded stream.
     *
     * @return The length of the stream.
     */
    virtual size_t getLength() { return binary_stream_get_length(m_hBinaryStreamHandle); }

    /**
     * @brief Checks if there is enough readable space in the stream.
     *
     * Determines if the stream has sufficient space to read the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for reading.
     * @return true if there is enough space to read the required number of bytes, false otherwise.
     */
    virtual bool enoughReadable(size_t uRequireSpace) {
        return binary_stream_enough_readable(m_hBinaryStreamHandle, uRequireSpace);
    }

    /**
     * @brief Checks if there is enough writable space in the stream.
     *
     * Determines if the stream has sufficient space to write the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for writing.
     * @return true if there is enough space to write the required number of bytes, false otherwise.
     */
    virtual bool enoughWritable(size_t uRequireSpace) {
        return binary_stream_enough_writable(m_hBinaryStreamHandle, uRequireSpace);
    }

private:
    /// The binary stream handle.
    binary_stream_handle *m_hBinaryStreamHandle;
};

/**
 * @class CCacheStream
 * @brief A cache stream implementation.
 *
 * This class represents a cache stream that can attach and detach external buffers.
 * It uses a cache stream handle to manage the stream data.
 */
class CCacheStream : public CAbstractStream {
public:
    /**
     * @brief Constructor for CCacheStream.
     *
     * Initializes the cache stream with an optional specified cache size.
     *
     * @param uCacheSize The initial size of the cache (default is 0).
     */
    CCacheStream(size_t uCacheSize = 0) { m_hCacheStreamHandle = cache_stream_create(uCacheSize); }

    /**
     * @brief Destructor for CCacheStream.
     *
     * Cleans up the cache stream by destroying the cache stream handle.
     */
    virtual ~CCacheStream() { cache_stream_destroy(m_hCacheStreamHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CCacheStream objects.
     */
    CCacheStream(const CCacheStream &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CCacheStream objects.
     */
    CCacheStream &operator=(const CCacheStream &) = delete;
#endif

public:
    /**
     * @brief Attaches a buffer to the stream.
     *
     * Attaches an external buffer to the cache stream for data storage.
     *
     * @param pBuf Pointer to the buffer to be attached.
     * @param uCacheSize The size of the buffer.
     * @return true if the buffer was successfully attached, false otherwise.
     */
    bool attach(char *pBuf, size_t uCacheSize) { return cache_stream_attach(m_hCacheStreamHandle, pBuf, uCacheSize); }

    /**
     * @brief Detaches a buffer from the stream.
     *
     * Detaches the buffer from the cache stream and returns it.
     *
     * @param uCacheSize Reference to the size of the detached buffer.
     * @return Pointer to the detached buffer.
     */
    char *detach(size_t &uCacheSize) { return cache_stream_detach(m_hCacheStreamHandle, uCacheSize); }

public:
    /**
     * @brief Reads an object from the stream.
     *
     * Reads a specified number of bytes from the cache stream into a buffer.
     * Throws a CStreamException if the read operation fails.
     *
     * @param pBuf Pointer to the buffer where the data will be stored.
     * @param uReadLen Number of bytes to read from the stream.
     */
    virtual void readObject(void *pBuf, size_t uReadLen) {
        int ret = cache_stream_read(m_hCacheStreamHandle, pBuf, uReadLen);
        if (ret == CStreamException::INNER_CRITICAL_ERROR || ret == CStreamException::NOT_ENOUGH_SPACE)
            throw new CStreamException(ret);
    }

    /**
     * @brief Writes an object to the stream.
     *
     * Writes a specified number of bytes from a buffer into the cache stream.
     * Throws a CStreamException if the write operation fails.
     *
     * @param pBuf Pointer to the buffer containing the data to be written.
     * @param uWriteLen Number of bytes to write to the stream.
     */
    virtual void writeObject(const void *pBuf, const size_t uWriteLen) {
        int ret = cache_stream_write(m_hCacheStreamHandle, pBuf, uWriteLen);
        if (ret == CStreamException::INNER_CRITICAL_ERROR || ret == CStreamException::NOT_ENOUGH_SPACE)
            throw new CStreamException(ret);
    }

    /**
     * @brief Gets the length of the stream.
     *
     * Returns the current length of the cache stream.
     *
     * @return The length of the stream.
     */
    virtual size_t getLength() { return cache_stream_get_length(m_hCacheStreamHandle); }

    /**
     * @brief Checks if there is enough readable space in the stream.
     *
     * Determines if the stream has sufficient space to read the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for reading.
     * @return true if there is enough space to read the required number of bytes, false otherwise.
     */
    virtual bool enoughReadable(size_t uRequireSpace) {
        return cache_stream_enough_readable(m_hCacheStreamHandle, uRequireSpace);
    }

    /**
     * @brief Checks if there is enough writable space in the stream.
     *
     * Determines if the stream has sufficient space to write the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for writing.
     * @return true if there is enough space to write the required number of bytes, false otherwise.
     */
    virtual bool enoughWritable(size_t uRequireSpace) {
        return cache_stream_enough_writable(m_hCacheStreamHandle, uRequireSpace);
    }

private:
    /// The cache stream handle.
    cache_stream_handle *m_hCacheStreamHandle;
};

#endif
