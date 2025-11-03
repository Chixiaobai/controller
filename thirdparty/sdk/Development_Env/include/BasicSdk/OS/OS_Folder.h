#ifndef __BASICSDK_OS_FOLDER_H__
#define __BASICSDK_OS_FOLDER_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Check if a folder exists
 *
 * @param strFolderPath Path to the folder
 * @return bool True if the folder exists, false otherwise
 */
BASICSDK_API bool os_folder_exist(const char *strFolderPath);

/**
 * @brief Create a folder
 *
 * @param strFolderPath Path to the folder to create
 * @return bool True if the folder is created successfully, false otherwise
 */
BASICSDK_API bool os_folder_create(const char *strFolderPath);

/**
 * @brief Remove a folder
 *
 * @param strFolderPath Path to the folder to remove
 * @param bIncludeAllFiles If true, remove all files and subfolders; if false, only remove the folder if it is empty
 * @return bool True if the folder is removed successfully, false otherwise
 */
BASICSDK_API bool os_folder_remove(const char *strFolderPath, bool bIncludeAllFiles);

/**
 * @brief Move a folder
 *
 * @param strSrcFolderPath Path to the source folder
 * @param strDstFolderPath Path to the destination folder
 * @return bool True if the folder is moved successfully, false otherwise
 */
BASICSDK_API bool os_folder_move(const char *strSrcFolderPath, const char *strDstFolderPath);

/**
 * @brief Copy a folder
 *
 * @param strSrcFolderPath Path to the source folder
 * @param strDstFolderPath Path to the destination folder
 * @param bFailedIfExist If true, the function fails if the destination folder already exists
 * @return bool True if the folder is copied successfully, false otherwise
 */
BASICSDK_API bool os_folder_copy(const char *strSrcFolderPath, const char *strDstFolderPath, bool bFailedIfExist);

/**
 * @brief Namespace for folder-related functions and classes
 */
namespace OS {
class Folder {
public:
    /**
     * @brief Check if a folder exists
     *
     * @param strFolderPath Path to the folder
     * @return bool True if the folder exists, false otherwise
     */
    static bool exist(const string &strFolderPath) { return os_folder_exist(strFolderPath.c_str()); }

    /**
     * @brief Create a folder
     *
     * @param strFolderPath Path to the folder to create
     * @return bool True if the folder is created successfully, false otherwise
     */
    static bool create(const string &strFolderPath) { return os_folder_create(strFolderPath.c_str()); }

    /**
     * @brief Remove a folder
     *
     * @param strFolderPath Path to the folder to remove
     * @param bIncludeAllFiles If true, remove all files and subfolders; if false, only remove the folder if it is empty
     * @return bool True if the folder is removed successfully, false otherwise
     */
    static bool remove(const string &strFolderPath, bool bIncludeAllFiles) {
        return os_folder_remove(strFolderPath.c_str(), bIncludeAllFiles);
    }

    /**
     * @brief Move a folder
     *
     * @param strSrcFolderPath Path to the source folder
     * @param strDstFolderPath Path to the destination folder
     * @return bool True if the folder is moved successfully, false otherwise
     */
    static bool move(const string &strSrcFolderPath, const string &strDstFolderPath) {
        return os_folder_move(strSrcFolderPath.c_str(), strDstFolderPath.c_str());
    }

    /**
     * @brief Copy a folder
     *
     * @param strSrcFolderPath Path to the source folder
     * @param strDstFolderPath Path to the destination folder
     * @param bFailedIfExist If true, the function fails if the destination folder already exists
     * @return bool True if the folder is copied successfully, false otherwise
     */
    static bool copy(const string &strSrcFolderPath, const string &strDstFolderPath, bool bFailedIfExist) {
        return os_folder_copy(strSrcFolderPath.c_str(), strDstFolderPath.c_str(), bFailedIfExist);
    }
};
}  // namespace OS

#endif