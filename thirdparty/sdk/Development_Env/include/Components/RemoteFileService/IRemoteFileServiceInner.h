#pragma once

#define UNIQUE_REMOTEFILESERVICE_NAME "RemoteFileService"

struct safearray_handle;
interface IRemoteFile;
/**
 * @interface IRemoteFileServiceInner
 * @brief Interface for managing files and folders on a remote system.
 *
 * This interface provides comprehensive methods to create, remove, move, copy,
 * and query the existence of files and folders on a remote system. It also supports
 * time manipulation, path operations, and file listing capabilities.
 */
interface IRemoteFileServiceInner : public IBase {
    /**
     * @brief Creates a new file on the remote system.
     *
     * @param strFilePath Path where the file should be created.
     * @return true if the file was successfully created; false otherwise.
     */
    virtual bool createRemoteFile(const char* strFilePath) = 0;

    /**
     * @brief Removes a file from the remote system.
     *
     * @param strFilePath Path of the file to remove.
     * @return true if the file was successfully removed; false otherwise.
     */
    virtual bool removeRemoteFile(const char* strFilePath) = 0;

    /**
     * @brief Checks whether a file exists on the remote system.
     *
     * @param strFilePath Path of the file to check.
     * @return true if the file exists; false otherwise.
     */
    virtual bool existRemoteFile(const char* strFilePath) = 0;

    /**
     * @brief Moves a file within the remote system (not between hosts).
     *
     * @param strSrcFilePath Source file path.
     * @param strDstFilePath Destination file path.
     * @return true if the file was successfully moved; false otherwise.
     */
    virtual bool moveRemoteFile(const char* strSrcFilePath, const char* strDstFilePath) = 0;

    /**
     * @brief Copies a file within the remote system (not between hosts).
     *
     * @param strSrcFilePath Source file path.
     * @param strDstFilePath Destination file path.
     * @param bFailedIfExist If true, operation fails if destination already exists.
     * @return true if the file was successfully copied; false otherwise.
     */
    virtual bool copyRemoteFile(const char* strSrcFilePath, const char* strDstFilePath, bool bFailedIfExist) = 0;

    /**
     * @brief Retrieves an interface to access a remote file.
     *
     * @param strFilePath Path of the remote file.
     * @return Pointer to the IRemoteFile interface if successful; nullptr otherwise.
     */
    virtual IRemoteFile* getRemoteFile(const char* strFilePath) = 0;

    /**
     * @brief Creates a folder on the remote system.
     *
     * @param strFolderPath Path where the folder should be created.
     * @return true if the folder was successfully created; false otherwise.
     */
    virtual bool createRemoteFolder(const char* strFolderPath) = 0;

    /**
     * @brief Removes a folder from the remote system.
     *
     * @param strFolderPath Path of the folder to remove.
     * @param bIncludeAllFiles If true, removes all contents inside the folder recursively.
     * @return true if the folder was successfully removed; false otherwise.
     */
    virtual bool removeRemoteFolder(const char* strFolderPath, bool bIncludeAllFiles) = 0;

    /**
     * @brief Checks whether a folder exists on the remote system.
     *
     * @param strFolderPath Path of the folder to check.
     * @return true if the folder exists; false otherwise.
     */
    virtual bool existRemoteFolder(const char* strFolderPath) = 0;

    /**
     * @brief Moves a folder within the remote system (not between hosts).
     *
     * @param strSrcFolderPath Source folder path.
     * @param strDstFolderPath Destination folder path.
     * @return true if the folder was successfully moved; false otherwise.
     */
    virtual bool moveRemoteFolder(const char* strSrcFolderPath, const char* strDstFolderPath) = 0;

    /**
     * @brief Copies a folder within the remote system (not between hosts).
     *
     * @param strSrcFolderPath Source folder path.
     * @param strDstFolderPath Destination folder path.
     * @param bFailedIfExist If true, operation fails if destination already exists.
     * @return true if the folder was successfully copied; false otherwise.
     */
    virtual bool copyRemoteFolder(const char* strSrcFolderPath, const char* strDstFolderPath, bool bFailedIfExist) = 0;

    /**
     * @brief Gets the application-specific folder path on the remote system.
     *
     * @param strApplicationDir Output parameter that receives the application folder path.
     */
    virtual void getApplicationFolder(safearray_handle * *strApplicationDir) = 0;

    /**
     * @brief Lists files and folders in the specified remote directory.
     *
     * @param strParentFolderPath Path of the parent directory to list.
     * @param strFileList Output parameter that receives the list of files.
     * @param strFolderList Output parameter that receives the list of folders.
     */
    virtual void remoteLs(const char* strParentFolderPath, safearray_handle** strFileList,
                          safearray_handle** strFolderList) = 0;

    /**
     * @brief Splices a relative path with a workspace to form an absolute path.
     *
     * @param strRelativePath Relative path to splice.
     * @param strWorkspace Base workspace path.
     * @param strAbsolutePath Output parameter that receives the resulting absolute path.
     * @return true if the operation succeeded; false otherwise.
     */
    virtual bool splicePath(const char* strRelativePath, const char* strWorkspace,
                            safearray_handle** strAbsolutePath) = 0;

    /**
     * @brief Retrieves the path separator character used by the remote system.
     *
     * @return The path separator character (e.g., '/' or '\').
     */
    virtual char getPathSplit() = 0;

    /**
     * @brief Forces all pending changes to be written to disk on the remote system.
     *
     * @return true if the sync operation succeeded; false otherwise.
     */
    virtual bool sync() = 0;

    /**
     * @brief Sets both access and modification times for a file on the remote system.
     *
     * @param strFilePath Path of the file.
     * @param intYear Year for the new timestamp.
     * @param intMonth Month (1-12) for the new timestamp.
     * @param intDay Day of month (1-31) for the new timestamp.
     * @param intHour Hour (0-23) for the new timestamp.
     * @param intMinute Minute (0-59) for the new timestamp.
     * @param intSecond Second (0-59) for the new timestamp.
     * @return true if the timestamps were successfully set; false otherwise.
     */
    virtual bool setFileTime(const char* strFilePath, int intYear, int intMonth, int intDay, int intHour, int intMinute,
                             int intSecond) = 0;

    /**
     * @brief Sets the access time for a file on the remote system.
     *
     * @param strFilePath Path of the file.
     * @param intYear Year for the new timestamp.
     * @param intMonth Month (1-12) for the new timestamp.
     * @param intDay Day of month (1-31) for the new timestamp.
     * @param intHour Hour (0-23) for the new timestamp.
     * @param intMinute Minute (0-59) for the new timestamp.
     * @param intSecond Second (0-59) for the new timestamp.
     * @return true if the timestamp was successfully set; false otherwise.
     */
    virtual bool setFileAccessTime(const char* strFilePath, int intYear, int intMonth, int intDay, int intHour,
                                   int intMinute, int intSecond) = 0;

    /**
     * @brief Sets the modification time for a file on the remote system.
     *
     * @param strFilePath Path of the file.
     * @param intYear Year for the new timestamp.
     * @param intMonth Month (1-12) for the new timestamp.
     * @param intDay Day of month (1-31) for the new timestamp.
     * @param intHour Hour (0-23) for the new timestamp.
     * @param intMinute Minute (0-59) for the new timestamp.
     * @param intSecond Second (0-59) for the new timestamp.
     * @return true if the timestamp was successfully set; false otherwise.
     */
    virtual bool setFileModificationTime(const char* strFilePath, int intYear, int intMonth, int intDay, int intHour,
                                         int intMinute, int intSecond) = 0;

    /**
     * @brief Retrieves the access time of a file on the remote system.
     *
     * @param strFilePath Path of the file.
     * @param intYear Output year value.
     * @param intMonth Output month value (1-12).
     * @param intDay Output day of month (1-31).
     * @param intHour Output hour value (0-23).
     * @param intMinute Output minute value (0-59).
     * @param intSecond Output second value (0-59).
     * @return true if the timestamp was successfully retrieved; false otherwise.
     */
    virtual bool getFileAccessTime(const char* strFilePath, int& intYear, int& intMonth, int& intDay, int& intHour,
                                   int& intMinute, int& intSecond) = 0;

    /**
     * @brief Retrieves the modification time of a file on the remote system.
     *
     * @param strFilePath Path of the file.
     * @param intYear Output year value.
     * @param intMonth Output month value (1-12).
     * @param intDay Output day of month (1-31).
     * @param intHour Output hour value (0-23).
     * @param intMinute Output minute value (0-59).
     * @param intSecond Output second value (0-59).
     * @return true if the timestamp was successfully retrieved; false otherwise.
     */
    virtual bool getFileModificationTime(const char* strFilePath, int& intYear, int& intMonth, int& intDay,
                                         int& intHour, int& intMinute, int& intSecond) = 0;
};

class CRemoteFileServiceHelper {};

ENABLE_RECOGNIZE_MACRO(IRemoteFileServiceInner)
