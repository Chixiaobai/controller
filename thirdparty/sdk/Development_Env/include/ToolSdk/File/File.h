/**
 * @file File.h
 * @brief Header file for file handling functionalities.
 *
 * This file defines structures, functions, and a class for managing file operations.
 */

#ifndef __TOOLSDK_FILE_H__
#define __TOOLSDK_FILE_H__

#include "ToolSdkDef.h"
#include "OS/OS_FileSystem.h"

/**
 * @def CARRIAGE_PART
 * @brief Macro for the carriage return part.
 *
 * Represents the carriage return part used in file operations.
 */
#define CARRIAGE_PART "\n"

/**
 * @struct file_object_handle
 * @brief Opaque handle for managing file objects.
 *
 * This structure represents an opaque handle used internally for managing file objects.
 */
struct file_object_handle;

/**
 * @fn file_object_handle *file_object_create()
 * @brief Creates a new file object handle.
 *
 * Allocates and initializes a new file object handle for use in subsequent file operations.
 *
 * @return A pointer to the newly created file object handle.
 */
TOOLSDK_API file_object_handle *file_object_create();

/**
 * @fn file_object_handle *file_object_attach(OS_FILE_HANDLE iFd)
 * @brief Attaches an existing file descriptor to a file object handle.
 *
 * Attaches an existing file descriptor to a new file object handle for use in subsequent file operations.
 *
 * @param iFd The file descriptor to attach.
 * @return A pointer to the newly created file object handle.
 */
TOOLSDK_API file_object_handle *file_object_attach(OS_FILE_HANDLE iFd);

/**
 * @fn void file_object_destroy(file_object_handle *&handle)
 * @brief Destroys a file object handle.
 *
 * Frees the resources associated with the given file object handle.
 *
 * @param handle A reference to the file object handle to be destroyed.
 */
TOOLSDK_API void file_object_destroy(file_object_handle *&handle);

/**
 * @fn bool file_object_open(file_object_handle *handle, const char *strFilePath, int iOpenFlags)
 * @brief Opens a file using the specified flags.
 *
 * Opens the specified file with the given flags and associates it with the file object handle.
 *
 * @param handle The file object handle.
 * @param strFilePath The path to the file to open.
 * @param iOpenFlags The flags specifying the mode in which to open the file.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool file_object_open(file_object_handle *handle, const char *strFilePath, int iOpenFlags);

/**
 * @fn void file_object_close(file_object_handle *handle)
 * @brief Closes a file.
 *
 * Closes the file associated with the file object handle.
 *
 * @param handle The file object handle.
 */
TOOLSDK_API void file_object_close(file_object_handle *handle);

/**
 * @fn long file_object_get_length(file_object_handle *handle)
 * @brief Retrieves the length of the file.
 *
 * Retrieves the length of the file associated with the file object handle.
 *
 * @param handle The file object handle.
 * @return The length of the file in bytes.
 */
TOOLSDK_API long file_object_get_length(file_object_handle *handle);

/**
 * @fn int file_object_read(file_object_handle *handle, void *lpBuf, unsigned int uCount)
 * @brief Reads data from a file.
 *
 * Reads up to uCount bytes from the file associated with the file object handle into the provided buffer.
 *
 * @param handle The file object handle.
 * @param lpBuf Pointer to the buffer to store the read data.
 * @param uCount The number of bytes to read.
 * @return The number of bytes read, or -1 if an error occurred.
 */
TOOLSDK_API int file_object_read(file_object_handle *handle, void *lpBuf, unsigned int uCount);

/**
 * @fn int file_object_write(file_object_handle *handle, void *lpBuf, unsigned int uCount)
 * @brief Writes data to a file.
 *
 * Writes uCount bytes from the provided buffer to the file associated with the file object handle.
 *
 * @param handle The file object handle.
 * @param lpBuf Pointer to the buffer containing the data to write.
 * @param uCount The number of bytes to write.
 * @return The number of bytes written, or -1 if an error occurred.
 */
TOOLSDK_API int file_object_write(file_object_handle *handle, void *lpBuf, unsigned int uCount);

/**
 * @fn long file_object_seek(file_object_handle *handle, long lOff, unsigned int nFrom)
 * @brief Seeks to a position in a file.
 *
 * Seeks to the specified position in the file associated with the file object handle.
 *
 * @param handle The file object handle.
 * @param lOff The offset to seek to.
 * @param nFrom The reference point for the seek operation.
 * @return The new position in the file, or -1 if an error occurred.
 */
TOOLSDK_API long file_object_seek(file_object_handle *handle, long lOff, unsigned int nFrom);

/**
 * @fn void file_object_sync(file_object_handle *handle)
 * @brief Synchronizes a file.
 *
 * Synchronizes the file associated with the file object handle, ensuring all buffered data is written to disk.
 *
 * @param handle The file object handle.
 */
TOOLSDK_API void file_object_sync(file_object_handle *handle);

/**
 * @class CFile
 * @brief Class for managing file operations.
 *
 * This class provides a convenient interface for performing file operations such as opening, reading, writing, and
 * seeking.
 */
class CFile {
public:
    /**
     * @enum OpenFlags
     * @brief Flags for opening files.
     *
     * Enumerates the flags that can be used to specify the mode in which to open a file.
     */
    enum OpenFlags {
        modeRead = (int)0x01,       /**< Open for reading. */
        modeWrite = (int)0x02,      /**< Open for writing. */
        modeReadWrite = (int)0x04,  /**< Open for reading and writing. */
        modeCreate = (int)0x08,     /**< Create the file if it does not exist. */
        modeNoTruncate = (int)0x10, /**< Do not truncate the file if it exists. */
        modeSync = (int)0x20        /**< Synchronous mode, write operations wait for data to be written to disk. */
    };

    /**
     * @enum SeekMode
     * @brief Modes for seeking in a file.
     *
     * Enumerates the modes that can be used to specify the reference point for seek operations.
     */
    enum SeekMode {
        begin = (int)0x01,   /**< Seek from the beginning of the file. */
        current = (int)0x02, /**< Seek from the current position in the file. */
        end = (int)0x03      /**< Seek from the end of the file. */
    };

public:
    /**
     * @brief Constructor for CFile.
     *
     * Initializes the file object by creating a new file object handle.
     */
    CFile() { m_pFileObjectHandle = file_object_create(); }

    /**
     * @brief Constructor for CFile with an existing file descriptor.
     *
     * Initializes the file object by attaching an existing file descriptor to a new file object handle.
     *
     * @param iFd The file descriptor to attach.
     */
    CFile(OS_FILE_HANDLE iFd) { m_pFileObjectHandle = file_object_attach(iFd); }

    /**
     * @brief Destructor for CFile.
     *
     * Cleans up the file object by destroying the file object handle.
     */
    virtual ~CFile() { file_object_destroy(m_pFileObjectHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CFile objects to ensure proper management of the file object handle.
     */
    CFile(const CFile &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CFile objects to ensure proper management of the file object handle.
     */
    CFile &operator=(const CFile &) = delete;
#endif

public:
    /**
     * @brief Seeks to the beginning of the file.
     *
     * Seeks to the beginning of the file associated with the file object handle.
     */
    void SeekToBegin() { file_object_seek(m_pFileObjectHandle, 0, CFile::begin); }

    /**
     * @brief Seeks to the end of the file.
     *
     * Seeks to the end of the file associated with the file object handle and returns the new position.
     *
     * @return The new position in the file.
     */
    unsigned long SeekToEnd() { return file_object_seek(m_pFileObjectHandle, 0, CFile::end); }

public:
    /**
     * @brief Opens a file using the specified flags.
     *
     * Opens the specified file with the given flags and associates it with the file object handle.
     *
     * @param strFilePath The path to the file to open.
     * @param iOpenFlags The flags specifying the mode in which to open the file.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool open(const char *strFilePath, int iOpenFlags) {
        return file_object_open(m_pFileObjectHandle, strFilePath, iOpenFlags);
    }

    /**
     * @brief Closes a file.
     *
     * Closes the file associated with the file object handle.
     */
    virtual void close() { file_object_close(m_pFileObjectHandle); }

    /**
     * @brief Retrieves the length of the file.
     *
     * Retrieves the length of the file associated with the file object handle.
     *
     * @return The length of the file in bytes.
     */
    virtual long getLength() const { return file_object_get_length(m_pFileObjectHandle); }

    /**
     * @brief Reads data from a file.
     *
     * Reads up to uCount bytes from the file associated with the file object handle into the provided buffer.
     *
     * @param lpBuf Pointer to the buffer to store the read data.
     * @param uCount The number of bytes to read.
     * @return The number of bytes read, or -1 if an error occurred.
     */
    virtual int read(void *lpBuf, unsigned int uCount) {
#ifdef _WIN32
        return file_object_read(m_pFileObjectHandle, lpBuf, uCount);
#else
        int ret = 0;
        while (true) {
            ret = file_object_read(m_pFileObjectHandle, lpBuf, uCount);
            if (ret == -1 && errno == EINTR)
                continue;
            else
                break;
        }
        return ret;
#endif
    }

    /**
     * @brief Writes data to a file.
     *
     * Writes uCount bytes from the provided buffer to the file associated with the file object handle.
     *
     * @param lpBuf Pointer to the buffer containing the data to write.
     * @param uCount The number of bytes to write.
     * @return The number of bytes written, or -1 if an error occurred.
     */
    virtual int write(void *lpBuf, unsigned int uCount) {
#ifdef _WIN32
        return file_object_write(m_pFileObjectHandle, lpBuf, uCount);
#else
        int ret = 0;
        while (true) {
            ret = file_object_write(m_pFileObjectHandle, lpBuf, uCount);
            if (ret == -1 && errno == EINTR)
                continue;
            else
                break;
        }
        return ret;
#endif
    }

    /**
     * @brief Seeks to a position in a file.
     *
     * Seeks to the specified position in the file associated with the file object handle.
     *
     * @param lOff The offset to seek to.
     * @param nFrom The reference point for the seek operation.
     * @return The new position in the file, or -1 if an error occurred.
     */
    virtual long seek(long lOff, unsigned int nFrom) { return file_object_seek(m_pFileObjectHandle, lOff, nFrom); }

    /**
     * @brief Synchronizes a file.
     *
     * Synchronizes the file associated with the file object handle, ensuring all buffered data is written to disk.
     */
    virtual void sync() { file_object_sync(m_pFileObjectHandle); }

public:
    /**
     * @brief Retrieves the base name of a file.
     *
     * Retrieves the base name (name without path and extension) of the specified file path.
     *
     * @param absoluteFilePath The absolute path to the file.
     * @return The base name of the file as a string.
     */
    static string baseName(const char *absoluteFilePath) {
        string strValue;
        DWORD dwLen = 0;
        if (OS::FileSystem::baseName(absoluteFilePath, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            OS::FileSystem::baseName(absoluteFilePath, pValue, dwLen);
            strValue = pValue;
            delete[] pValue;
            pValue = nullptr;
            return strValue;
        } else
            return "";
    }

    /**
     * @brief Retrieves the directory name of a file.
     *
     * Retrieves the directory name (path without file name) of the specified file path.
     *
     * @param absoluteFilePath The absolute path to the file.
     * @return The directory name of the file as a string.
     */
    static string dirName(const char *absoluteFilePath) {
        string strValue;
        DWORD dwLen = 0;
        if (OS::FileSystem::dirName(absoluteFilePath, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            OS::FileSystem::dirName(absoluteFilePath, pValue, dwLen);
            strValue = pValue;
            delete[] pValue;
            pValue = nullptr;
            return strValue;
        } else
            return "";
    }

    /**
     * @brief Retrieves the file suffix (extension) of a file.
     *
     * Retrieves the file suffix (extension) of the specified file path.
     *
     * @param absoluteFilePath The absolute path to the file.
     * @return The file suffix of the file as a string.
     */
    static string fileSuffix(const char *absoluteFilePath) {
        string strValue;
        DWORD dwLen = 0;
        if (OS::FileSystem::fileSuffix(absoluteFilePath, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            OS::FileSystem::fileSuffix(absoluteFilePath, pValue, dwLen);
            strValue = pValue;
            delete[] pValue;
            pValue = nullptr;
            return strValue;
        } else
            return "";
    }

    /**
     * @brief Synchronizes a file by opening and closing it.
     *
     * Synchronizes the specified file by opening it in read mode and then calling the sync method.
     *
     * @param strFilePath The path to the file to synchronize.
     */
    static void syncFile(const char *strFilePath) {
        CFile syncFile;
        syncFile.open(strFilePath, CFile::modeRead);
        syncFile.sync();
        syncFile.close();
    }

protected:
    /**
     * @var file_object_handle *m_pFileObjectHandle
     * @brief Opaque handle for managing the file object.
     *
     * This handle is used internally by the CFile class to manage the file object.
     */
    file_object_handle *m_pFileObjectHandle;
};

#endif  // __TOOLSDK_FILE_H__