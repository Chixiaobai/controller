#ifndef __BASICSDK_OS_FILE_H__
#define __BASICSDK_OS_FILE_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Check if a path is relative
 *
 * @param strPath Path to check
 * @return bool True if the path is relative, false otherwise
 */
BASICSDK_API bool os_file_is_relative_path(const char *strPath);

/**
 * @brief Check if a file exists
 *
 * @param strFilePath Path to the file
 * @return bool True if the file exists, false otherwise
 */
BASICSDK_API bool os_file_exist(const char *strFilePath);

/**
 * @brief Create a file
 *
 * @param strFilePath Path to the file to create
 * @return bool True if the file is created successfully, false otherwise
 */
BASICSDK_API bool os_file_create(const char *strFilePath);

/**
 * @brief Remove a file
 *
 * @param strFilePath Path to the file to remove
 * @return bool True if the file is removed successfully, false otherwise
 */
BASICSDK_API bool os_file_remove(const char *strFilePath);

/**
 * @brief Get the file time
 *
 * @param strFilePath Path to the file
 * @param tv Pointer to an OS_FILE_TIME structure to store the file time
 * @return bool True if the file time is retrieved successfully, false otherwise
 */
BASICSDK_API bool os_file_gettime(const char *strFilePath, OS_FILE_TIME *tv);

/**
 * @brief Set the file time
 *
 * @param strFilePath Path to the file
 * @param tv Pointer to an OS_FILE_TIME structure containing the new file time
 * @return bool True if the file time is set successfully, false otherwise
 */
BASICSDK_API bool os_file_settime(const char *strFilePath, OS_FILE_TIME *tv);

/**
 * @brief Move a file
 *
 * @param strSrcFilePath Path to the source file
 * @param strDstFilePath Path to the destination file
 * @return bool True if the file is moved successfully, false otherwise
 */
BASICSDK_API bool os_file_move(const char *strSrcFilePath, const char *strDstFilePath);

/**
 * @brief Copy a file
 *
 * @param strSrcFilePath Path to the source file
 * @param strDstFilePath Path to the destination file
 * @param bFailedIfExist If true, the function fails if the destination file already exists
 * @return bool True if the file is copied successfully, false otherwise
 */
BASICSDK_API bool os_file_copy(const char *strSrcFilePath, const char *strDstFilePath, bool bFailedIfExist);

/**
 * @brief Namespace for file-related functions and classes
 */
namespace OS {
class File {
public:
    /**
     * @brief Check if a path is relative
     *
     * @param strPath Path to check
     * @return bool True if the path is relative, false otherwise
     */
    static bool isRelativePath(const string &strPath) { return os_file_is_relative_path(strPath.c_str()); }

    /**
     * @brief Check if a file exists
     *
     * @param strFilePath Path to the file
     * @return bool True if the file exists, false otherwise
     */
    static bool exist(const string &strFilePath) { return os_file_exist(strFilePath.c_str()); }

    /**
     * @brief Create a file
     *
     * @param strFilePath Path to the file to create
     * @return bool True if the file is created successfully, false otherwise
     */
    static bool create(const string &strFilePath) { return os_file_create(strFilePath.c_str()); }

    /**
     * @brief Remove a file
     *
     * @param strFilePath Path to the file to remove
     * @return bool True if the file is removed successfully, false otherwise
     */
    static bool remove(const string &strFilePath) { return os_file_remove(strFilePath.c_str()); }

    /**
     * @brief Get the file time
     *
     * @param strFilePath Path to the file
     * @param tv Pointer to an OS_FILE_TIME structure to store the file time
     * @return bool True if the file time is retrieved successfully, false otherwise
     */
    static bool gettime(const string &strFilePath, OS_FILE_TIME *tv) {
        return os_file_gettime(strFilePath.c_str(), tv);
    }

    /**
     * @brief Set the file time
     *
     * @param strFilePath Path to the file
     * @param tv Pointer to an OS_FILE_TIME structure containing the new file time
     *0 @return bool True if the file time is set successfully, false otherwise
     */
    static bool settime(const string &strFilePath, OS_FILE_TIME *tv) {
        return os_file_settime(strFilePath.c_str(), tv);
    }

    /**
     * @brief Move a file
     *
     * @param strSrcFilePath Path to the source file
     * @param strDstFilePath Path to the destination file
     * @return bool True if the file is moved successfully, false otherwise
     */
    static bool move(const string &strSrcFilePath, const string &strDstFilePath) {
        return os_file_move(strSrcFilePath.c_str(), strDstFilePath.c_str());
    }

    /**
     * @brief Copy a file
     *
     * @param strSrcFilePath Path to the source file
     * @param strDstFilePath Path to the destination file
     * @param bFailedIfExist If true, the function fails if the destination file already exists
     * @return bool True if the file is copied successfully, false otherwise
     */
    static bool copy(const string &strSrcFilePath, const string &strDstFilePath, bool bFailedIfExist) {
        return os_file_copy(strSrcFilePath.c_str(), strDstFilePath.c_str(), bFailedIfExist);
    }
};
}  // namespace OS

#endif