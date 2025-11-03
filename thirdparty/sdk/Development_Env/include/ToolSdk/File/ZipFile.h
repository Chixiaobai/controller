#ifndef __TOOLSDK_ZIPFILE_H__
#define __TOOLSDK_ZIPFILE_H__

#include "ToolSdkDef.h"
#include "NameEnum/INameEnum.h"

/**
 * @file ZipFile.h
 * @brief Header file for zip file operations.
 *
 * This file contains the definitions for interfaces, enumerations, structures, and classes related to zip file
 * operations, including zipping and unzipping files and folders.
 */

// Note: The current version does not support symbolic links.

/**
 * @interface IZipProcessCallback
 * @brief Interface for zip process callbacks.
 *
 * This interface defines a callback method that is called during the zip file processing to report progress.
 */
interface IZipProcessCallback : public IBase {
    /**
     * @brief Callback method for zip file processing.
     *
     * This method is called to report the progress of the zip file processing.
     *
     * @param uintZipFileIndex The index of the zip file being processed.
     * @param strZipFilePath The path of the zip file being processed.
     * @param dblPercent The percentage of completion (0.0 to 100.0).
     */
    virtual void onZipFileProcess(unsigned int uintZipFileIndex, const char *strZipFilePath,
                                  const double dblPercent) = 0;
};

/**
 * @interface IUnzipProcessCallback
 * @brief Interface for unzip process callbacks.
 *
 * This interface defines a callback method that is called during the unzip file processing to report progress.
 */
interface IUnzipProcessCallback : public IBase {
    /**
     * @brief Callback method for unzip file processing.
     *
     * This method is called to report the progress of the unzip file processing.
     *
     * @param uintZipFileIndex The index of the zip file being processed.
     * @param strZipFilePath The path of the zip file being processed.
     * @param dblPercent The percentage of completion (0.0 to 100.0).
     */
    virtual void onUnzipFileProcess(unsigned int uintZipFileIndex, const char *strZipFilePath,
                                    const double dblPercent) = 0;
};

/**
 * @enum COMPRESSED_LEVEL
 * @brief Compression levels for zip files.
 *
 * This enumeration defines different compression levels that can be used when creating zip files.
 */
enum COMPRESSED_LEVEL {
    NOT_COMPRESSED_LEVEL = 0,     /**< No compression. */
    FASTEST_COMPRESSED_LEVEL = 1, /**< Fastest compression level. */
    DEFAULT_COMPRESSED_LEVEL = 8, /**< Default compression level. */
    LOWEST_COMPRESSED_LEVEL = 9   /**< Lowest compression level (best compression). */
};

/**
 * @struct zip_file_object_handle
 * @brief Handle for a zip file object.
 *
 * This structure represents a handle to a zip file object used for zipping files and folders.
 */
struct zip_file_object_handle;

/**
 * @brief Creates a zip file object handle.
 *
 * Creates a new zip file object handle.
 *
 * @return A pointer to the created zip file object handle, or nullptr if creation failed.
 */
TOOLSDK_API zip_file_object_handle *zip_file_object_create();

/**
 * @brief Destroys a zip file object handle.
 *
 * Destroys the specified zip file object handle and releases associated resources.
 *
 * @param handle Reference to the zip file object handle to be destroyed.
 */
TOOLSDK_API void zip_file_object_destroy(zip_file_object_handle *&handle);

/**
 * @brief Opens a zip file object.
 *
 * Opens a zip file object with the specified file path, password, and compression level.
 *
 * @param handle The zip file object handle.
 * @param strFilePath The path of the zip file to open.
 * @param strPassword The password for the zip file (optional).
 * @param level The compression level for the zip file.
 * @return true if the zip file was successfully opened, false otherwise.
 */
TOOLSDK_API bool zip_file_object_open(zip_file_object_handle *handle, const char *strFilePath, const char *strPassword,
                                      COMPRESSED_LEVEL level);

/**
 * @brief Closes a zip file object.
 *
 * Closes the specified zip file object.
 *
 * @param handle The zip file object handle.
 */
TOOLSDK_API void zip_file_object_close(zip_file_object_handle *handle);

/**
 * @brief Appends a file to a zip file object.
 *
 * Appends a file to the specified zip file object.
 *
 * @param handle The zip file object handle.
 * @param strSourceFilePath The path of the source file to append.
 * @param strDstZipFolderPath The destination folder path within the zip file (optional).
 * @param zipProcessCallback The callback for reporting zip process progress (optional).
 * @return true if the file was successfully appended, false otherwise.
 */
TOOLSDK_API bool zip_file_object_append_file(zip_file_object_handle *handle, const char *strSourceFilePath,
                                             const char *strDstZipFolderPath, IZipProcessCallback *zipProcessCallback);

/**
 * @brief Appends a folder to a zip file object.
 *
 * Appends a folder to the specified zip file object.
 *
 * @param handle The zip file object handle.
 * @param strSourceFolderPath The path of the source folder to append.
 * @param strDstZipFolderPath The destination folder path within the zip file (optional).
 * @param zipProcessCallback The callback for reporting zip process progress (optional).
 * @return true if the folder was successfully appended, false otherwise.
 */
TOOLSDK_API bool zip_file_object_append_folder(zip_file_object_handle *handle, const char *strSourceFolderPath,
                                               const char *strDstZipFolderPath,
                                               IZipProcessCallback *zipProcessCallback);

/**
 * @struct unzip_file_object_handle
 * @brief Handle for an unzip file object.
 *
 * This structure represents a handle to an unzip file object used for unzipping files and folders.
 */
struct unzip_file_object_handle;

/**
 * @brief Creates an unzip file object handle.
 *
 * Creates a new unzip file object handle.
 *
 * @return A pointer to the created unzip file object handle, or nullptr if creation failed.
 */
TOOLSDK_API unzip_file_object_handle *unzip_file_object_create();

/**
 * @brief Destroys an unzip file object handle.
 *
 * Destroys the specified unzip file object handle and releases associated resources.
 *
 * @param handle Reference to the unzip file object handle to be destroyed.
 */
TOOLSDK_API void unzip_file_object_destroy(unzip_file_object_handle *&handle);

/**
 * @brief Opens an unzip file object.
 *
 * Opens an unzip file object with the specified file path and password.
 *
 * @param handle The unzip file object handle.
 * @param strFilePath The path of the zip file to open.
 * @param strPassword The password for the zip file (optional).
 * @return true if the zip file was successfully opened, false otherwise.
 */
TOOLSDK_API bool unzip_file_object_open(unzip_file_object_handle *handle, const char *strFilePath,
                                        const char *strPassword);

/**
 * @brief Closes an unzip file object.
 *
 * Closes the specified unzip file object.
 *
 * @param handle The unzip file object handle.
 */
TOOLSDK_API void unzip_file_object_close(unzip_file_object_handle *handle);

/**
 * @brief Lists the contents of a zip file object.
 *
 * Lists the contents of the specified zip file object.
 *
 * @param handle The unzip file object handle.
 * @param strSourceFilePath The path of the source file or folder within the zip file to list (optional).
 * @param bRecursion Flag indicating whether to list contents recursively.
 * @return A pointer to an INameEnum object containing the listed contents, or nullptr if listing failed.
 */
TOOLSDK_API INameEnum *unzip_file_object_ls(unzip_file_object_handle *handle, const char *strSourceFilePath,
                                            bool bRecursion);

/**
 * @brief Extracts a file from a zip file object.
 *
 * Extracts a file from the specified zip file object.
 *
 * @param handle The unzip file object handle.
 * @param strSourceFilePath The path of the source file within the zip file to extract.
 * @param strDstZipFolderPath The destination folder path for the extracted file.
 * @param unzipProcessCallback The callback for reporting unzip process progress (optional).
 * @return true if the file was successfully extracted, false otherwise.
 */
TOOLSDK_API bool unzip_file_object_extract(unzip_file_object_handle *handle, const char *strSourceFilePath,
                                           const char *strDstZipFolderPath,
                                           IUnzipProcessCallback *unzipProcessCallback);

/**
 * @brief Extracts all files from a zip file object.
 *
 * Extracts all files from the specified zip file object.
 *
 * @param handle The unzip file object handle.
 * @param strDstZipFolderPath The destination folder path for the extracted files.
 * @param unzipProcessCallback The callback for reporting unzip process progress (optional).
 * @return true if all files were successfully extracted, false otherwise.
 */
TOOLSDK_API bool unzip_file_object_extract_all(unzip_file_object_handle *handle, const char *strDstZipFolderPath,
                                               IUnzipProcessCallback *unzipProcessCallback);

/**
 * @class CZipFile
 * @brief Class for managing zip file operations.
 *
 * This class provides a convenient interface for managing zip file operations, including creation, opening, closing,
 * appending files and folders, and more.
 */
class CZipFile {
public:
    /**
     * @brief Constructor for CZipFile.
     *
     * Initializes a new CZipFile object.
     */
    CZipFile() { m_pZipFileObjectHandle = zip_file_object_create(); }

    /**
     * @brief Destructor for CZipFile.
     *
     * Destroys the CZipFile object and releases associated resources.
     */
    virtual ~CZipFile() { zip_file_object_destroy(m_pZipFileObjectHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CZipFile objects to ensure proper management of resources.
     */
    CZipFile(const CZipFile &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CZipFile objects to ensure proper management of resources.
     */
    CZipFile &operator=(const CZipFile &) = delete;
#endif

public:
    /**
     * @brief Opens a zip file.
     *
     * Opens a zip file with the specified file path, password, and compression level.
     *
     * @param strFilePath The path of the zip file to open.
     * @param strPassword The password for the zip file (optional).
     * @param level The compression level for the zip file.
     * @return true if the zip file was successfully opened, false otherwise.
     */
    virtual bool open(const char *strFilePath, const char *strPassword = nullptr,
                      COMPRESSED_LEVEL level = DEFAULT_COMPRESSED_LEVEL) {
        return zip_file_object_open(m_pZipFileObjectHandle, strFilePath, strPassword, level);
    }

    /**
     * @brief Closes the zip file.
     *
     * Closes the zip file.
     */
    virtual void close() { zip_file_object_close(m_pZipFileObjectHandle); }

    /**
     * @brief Appends a file to the zip file.
     *
     * Appends a file to the zip file.
     *
     * @param strSourceFilePath The path of the source file to append.
     * @param strDstZipFolderPath The destination folder path within the zip file (optional).
     * @param zipProcessCallback The callback for reporting zip process progress (optional).
     * @return true if the file was successfully appended, false otherwise.
     */
    virtual bool append_file(const char *strSourceFilePath, const char *strDstZipFolderPath = "",
                             IZipProcessCallback *zipProcessCallback = nullptr) {
        return zip_file_object_append_file(m_pZipFileObjectHandle, strSourceFilePath, strDstZipFolderPath,
                                           zipProcessCallback);
    }

    /**
     * @brief Appends a folder to the zip file.
     *
     * Appends a folder to the zip file.
     *
     * @param strSourceFolderPath The path of the source folder to append.
     * @param strDstZipFolderPath The destination folder path within the zip file (optional).
     * @param zipProcessCallback The callback for reporting zip process progress (optional).
     * @return true if the folder was successfully appended, false otherwise.
     */
    virtual bool append_folder(const char *strSourceFolderPath, const char *strDstZipFolderPath = "",
                               IZipProcessCallback *zipProcessCallback = nullptr) {
        return zip_file_object_append_folder(m_pZipFileObjectHandle, strSourceFolderPath, strDstZipFolderPath,
                                             zipProcessCallback);
    }

protected:
    /**
     * @var zip_file_object_handle *m_pZipFileObjectHandle
     * @brief Handle to the zip file object.
     */
    zip_file_object_handle *m_pZipFileObjectHandle;
};

/**
 * @class CUnzipFile
 * @brief Class for managing unzip file operations.
 *
 * This class provides a convenient interface for managing unzip file operations, including opening, closing, listing
 * contents, and extracting files and folders.
 */
class CUnzipFile {
public:
    /**
     * @brief Constructor for CUnzipFile.
     *
     * Initializes a new CUnzipFile object.
     */
    CUnzipFile() { m_pUnzipFileObjectHandle = unzip_file_object_create(); }

    /**
     * @brief Destructor for CUnzipFile.
     *
     * Destroys the CUnzipFile object and releases associated resources.
     */
    ~CUnzipFile() { unzip_file_object_destroy(m_pUnzipFileObjectHandle); }

public:
    /**
     * @brief Opens an unzip file.
     *
     * Opens an unzip file with the specified file path and password.
     *
     * @param strFilePath The path of the zip file to open.
     * @param strPassword The password for the zip file (optional).
     * @return true if the zip file was successfully opened, false otherwise.
     */
    virtual bool open(const char *strFilePath, const char *strPassword = nullptr) {
        return unzip_file_object_open(m_pUnzipFileObjectHandle, strFilePath, strPassword);
    }

    /**
     * @brief Closes the unzip file.
     *
     * Closes the unzip file.
     */
    virtual void close() { unzip_file_object_close(m_pUnzipFileObjectHandle); }

    /**
     * @brief Lists the contents of the unzip file.
     *
     * Lists the contents of the unzip file.
     *
     * @param strDstZipFolderPath The path of the source file or folder within the zip file to list (optional).
     * @param bRecursion Flag indicating whether to list contents recursively.
     * @return A pointer to an INameEnum object containing the listed contents, or nullptr if listing failed.
     */
    virtual INameEnum *ls(const char *strDstZipFolderPath = "", bool bRecursion = false) {
        return unzip_file_object_ls(m_pUnzipFileObjectHandle, strDstZipFolderPath, bRecursion);
    }

    /**
     * @brief Extracts a file from the unzip file.
     *
     * Extracts a file from the unzip file.
     *
     * @param strSourceFilePath The path of the source file within the zip file to extract.
     * @param strDstZipFolderPath The destination folder path for the extracted file.
     * @param unzipProcessCallback The callback for reporting unzip process progress (optional).
     * @return true if the file was successfully extracted, false otherwise.
     */
    virtual bool extract(const char *strSourceFilePath, const char *strDstZipFolderPath,
                         IUnzipProcessCallback *unzipProcessCallback = nullptr) {
        return unzip_file_object_extract(m_pUnzipFileObjectHandle, strSourceFilePath, strDstZipFolderPath,
                                         unzipProcessCallback);
    }

    /**
     * @brief Extracts all files from the unzip file.
     *
     * Extracts all files from the unzip file.
     *
     * @param strDstZipFolderPath The destination folder path for the extracted files.
     * @param unzipProcessCallback The callback for reporting unzip process progress (optional).
     * @return true if all files were successfully extracted, false otherwise.
     */
    virtual bool extractAll(const char *strDstZipFolderPath, IUnzipProcessCallback *unzipProcessCallback = nullptr) {
        return unzip_file_object_extract_all(m_pUnzipFileObjectHandle, strDstZipFolderPath, unzipProcessCallback);
    }

protected:
    /**
     * @var unzip_file_object_handle *m_pUnzipFileObjectHandle
     * @brief Handle to the unzip file object.
     */
    unzip_file_object_handle *m_pUnzipFileObjectHandle;
};

#endif