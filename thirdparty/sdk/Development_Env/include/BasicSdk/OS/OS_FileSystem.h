#ifndef __BASICSDK_OS_FILE_SYSTEM_H__
#define __BASICSDK_OS_FILE_SYSTEM_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"
#include "Base/IBase.h"

/**
 * @brief Define infinite depth for directory traversal
 */
#define INFINITE_DEEP -1

/**
 * @brief Interface for callback functions during directory traversal
 */
interface IWalkThroughCallback : public IBase {
public:
    /**
     * @brief Callback function called when a file is encountered
     *
     * @param filePath Path to the file
     * @param rootPath Root directory path
     */
    virtual void onFile(const char *filePath, const char *rootPath) = 0;

    /**
     * @brief Callback function called when a directory is encountered
     *
     * @param dirPath Path to the directory
     * @param rootPath Root directory path
     */
    virtual void onDirectory(const char *dirPath, const char *rootPath) = 0;
};

/**
 * @brief Enable recognition macro for IWalkThroughCallback interface
 */
ENABLE_RECOGNIZE_MACRO(IWalkThroughCallback)

/**
 * @brief Walk through a directory and its subdirectories
 *
 * @param dirPath Path to the directory to traverse
 * @param rootDir Root directory path
 * @param pCallback Pointer to the callback interface
 */
BASICSDK_API void os_filesystem_walk_through(const char *dirPath, const char *rootDir, IWalkThroughCallback *pCallback);

/**
 * @brief Walk through a directory with a specified maximum depth
 *
 * @param dirPath Path to the directory to traverse
 * @param rootDir Root directory path
 * @param uintMaxDeep Maximum depth of traversal (use INFINITE_DEEP for infinite depth)
 * @param pCallback Pointer to the callback interface
 */
BASICSDK_API void os_filesystem_walk_through_ex(const char *dirPath, const char *rootDir, unsigned int uintMaxDeep,
                                                IWalkThroughCallback *pCallback);

/**
 * @brief Get the base name of a file from an absolute file path
 *
 * @param absoluteFilePath Absolute path to the file
 * @param baseName Buffer to store the base name
 * @param len Length of the buffer
 * @return bool True if successful, false otherwise
 */
BASICSDK_API bool os_filesystem_base_name(const char *absoluteFilePath, char *baseName, DWORD &len);

/**
 * @brief Get the directory name from an absolute file path
 *
 * @param absoluteFilePath Absolute path to the file
 * @param dirName Buffer to store the directory name
 * @param len Length of the buffer
 * @return bool True if successful, false otherwise
 */
BASICSDK_API bool os_filesystem_dir_name(const char *absoluteFilePath, char *dirName, DWORD &len);

/**
 * @brief Get the file suffix from an absolute file path
 *
 * @param absoluteFilePath Absolute path to the file
 * @param fileSuffix Buffer to store the file suffix
 * @param len Length of the buffer
 * @return bool True if successful, false otherwise
 */
BASICSDK_API bool os_filesystem_file_suffix(const char *absoluteFilePath, char *fileSuffix, DWORD &len);

/**
 * @brief Synchronize the file system
 */
BASICSDK_API void os_filesystem_sync();

/**
 * @brief Namespace for file system-related functions and classes
 */
namespace OS {
class FileSystem {
public:
    /**
     * @brief Walk through a directory and its subdirectories
     *
     * @param dirPath Path to the directory to traverse
     * @param rootDir Root directory path
     * @param pCallback Pointer to the callback interface
     */
    static void walkThrough(const char *dirPath, const char *rootDir, IWalkThroughCallback *pCallback) {
        return os_filesystem_walk_through(dirPath, rootDir, pCallback);
    }

    /**
     * @brief Walk through a directory with a specified maximum depth
     *
     * @param dirPath Path to the directory to traverse
     * @param rootDir Root directory path
     * @param uintMaxDeep Maximum depth of traversal (use INFINITE_DEEP for infinite depth)
     * @param pCallback Pointer to the callback interface
     */
    static void walkThroughEx(const char *dirPath, const char *rootDir, unsigned int uintMaxDeep,
                              IWalkThroughCallback *pCallback) {
        return os_filesystem_walk_through_ex(dirPath, rootDir, uintMaxDeep, pCallback);
    }

    /**
     * @brief Synchronize the file system
     */
    static void sync() { return os_filesystem_sync(); }

    /**
     * @brief Get the base name of a file from an absolute file path
     *
     * @param absoluteFilePath Absolute path to the file
     * @param baseName Buffer to store the base name
     * @param len Length of the buffer
     * @return bool True if successful, false otherwise
     */
    static bool baseName(const char *absoluteFilePath, char *baseName, DWORD &len) {
        return os_filesystem_base_name(absoluteFilePath, baseName, len);
    }

    /**
     * @brief Get the directory name from an absolute file path
     *
     * @param absoluteFilePath Absolute path to the file
     * @param dirName Buffer to store the directory name
     * @param len Length of the buffer
     * @return bool True if successful, false otherwise
     */
    static bool dirName(const char *absoluteFilePath, char *dirName, DWORD &len) {
        return os_filesystem_dir_name(absoluteFilePath, dirName, len);
    }

    /**
     * @brief Get the file suffix from an absolute file path
     *
     * @param absoluteFilePath Absolute path to the file
     * @param fileSuffix Buffer to store the file suffix
     * @param len Length of the buffer
     * @return bool True if successful, false otherwise
     */
    static bool fileSuffix(const char *absoluteFilePath, char *fileSuffix, DWORD &len) {
        return os_filesystem_file_suffix(absoluteFilePath, fileSuffix, len);
    }
};
}  // namespace OS

#endif