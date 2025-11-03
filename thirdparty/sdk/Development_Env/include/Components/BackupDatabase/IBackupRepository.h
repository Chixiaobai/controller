#pragma once

#include "IBackupElement.h"
#include "NameEnum/INameEnum.h"

/**
 * @enum STORAGE_STRATEGY
 * @brief Storage strategies supported by backup repositories.
 */
enum STORAGE_STRATEGY {
    ROLLING_STORAGE = 0, /**< Rolling storage strategy (oldest backups are replaced first) */
};

/**
 * @typedef BACKUP_FILTER_FUNC
 * @brief Function pointer type for filtering backup elements.
 *
 * This function type is used as a callback during repository enumeration to allow
 * custom filtering logic based on the provided parameter and backup element properties.
 *
 * @param lpParam User-defined parameter passed to the filter function.
 * @param pBackupElement Pointer to the IBackupElement being evaluated.
 * @return true if the backup element should be included; false otherwise.
 */
typedef bool (*BACKUP_FILTER_FUNC)(void *lpParam, IBackupElement *pBackupElement);

/**
 * @interface IBackupRepositoryCallback
 * @brief Callback interface for handling repository events.
 *
 * This interface defines methods that are called when significant operations occur
 * on the backup repository such as backup, restore, rename, and destroy.
 */
interface IBackupRepositoryCallback : public IBase {
    /**
     * @brief Called after a successful backup operation.
     *
     * @param pBackupElement Pointer to the IBackupElement that was backed up.
     * @return true if the callback was handled successfully; false otherwise.
     */
    virtual bool doBackupCallback(IBackupElement * pBackupElement) = 0;

    /**
     * @brief Called after a successful restore operation.
     *
     * @param pBackupElement Pointer to the IBackupElement that was restored.
     * @return true if the callback was handled successfully; false otherwise.
     */
    virtual bool doRestoreCallback(IBackupElement * pBackupElement) = 0;

    /**
     * @brief Called after a successful rename operation.
     *
     * @param pBackupElement Pointer to the IBackupElement that was renamed.
     * @param strOldBackupName The previous name of the backup.
     * @param strNewBackupName The new name assigned to the backup.
     * @return true if the callback was handled successfully; false otherwise.
     */
    virtual bool doRenameCallback(IBackupElement * pBackupElement, const char *strOldBackupName,
                                  const char *strNewBackupName) = 0;

    /**
     * @brief Called after a successful destruction of a backup element.
     *
     * @param pBackupElement Pointer to the IBackupElement that was destroyed.
     * @return true if the callback was handled successfully; false otherwise.
     */
    virtual bool doDestroyCallback(IBackupElement * pBackupElement) = 0;
};

/**
 * @interface IBackupRepository
 * @brief Interface representing a backup repository.
 *
 * This interface provides methods to manage a collection of backup elements including creation,
 * deletion, renaming, querying metadata, and managing repository settings.
 */
interface IBackupRepository : public IBase {
    /**
     * @brief Sets the workspace directory for this repository.
     *
     * @param strWorkspace Path to the directory where backups will be stored.
     * @return true if the workspace was successfully set; false otherwise.
     */
    virtual bool setBackupRepositoryWorkspace(const char *strWorkspace) = 0;

    /**
     * @brief Retrieves the name of this repository.
     *
     * @param strRepositoryName Buffer to receive the repository name.
     * @param dwLen On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getRepositoryName(char *strRepositoryName, DWORD &dwLen) = 0;

    /**
     * @brief Gets the storage strategy used by this repository.
     *
     * @return The STORAGE_STRATEGY value indicating the current strategy.
     */
    virtual const STORAGE_STRATEGY getRepositoryStorageStrategy() = 0;

    /**
     * @brief Retrieves the maximum capacity of the repository in bytes.
     *
     * @return Repository capacity in bytes.
     */
    virtual unsigned long getRepositoryCapacity() = 0;

    /**
     * @brief Sets the maximum capacity of the repository in bytes.
     *
     * @param ulCapacity New capacity value in bytes.
     * @return true if successful; false otherwise.
     */
    virtual bool setRepositoryCapacity(unsigned long ulCapacity) = 0;

    /**
     * @brief Loads existing backups from persistent storage into memory.
     */
    virtual void loadBackupRepositories() = 0;

    /**
     * @brief Saves current in-memory backups to persistent storage.
     */
    virtual void saveBackupRepositories() = 0;

    /**
     * @brief Creates a new backup element within this repository.
     *
     * @param strBackupName Name for the new backup.
     * @param type Type of backup to create (default: FILE_BACKUP).
     * @param backupTime Optional timestamp for the backup (default: now).
     * @return true if the backup element was created successfully; false otherwise.
     */
    virtual bool createBackupElement(const char *strBackupName, BACKUP_TYPE type = FILE_BACKUP,
                                     const time_t *backupTime = nullptr) = 0;

    /**
     * @brief Finds an existing backup element by its name.
     *
     * @param strBackupName Name of the backup element to find.
     * @return Pointer to the IBackupElement instance if found; nullptr otherwise.
     */
    virtual IBackupElement *findBackupElement(const char *strBackupName) = 0;

    /**
     * @brief Removes a backup element from the repository.
     *
     * @param strBackupName Name of the backup element to remove.
     * @param bReleaseResource true to release associated resources immediately; false otherwise.
     * @return true if the backup element was removed successfully; false otherwise.
     */
    virtual bool removeBackupElement(const char *strBackupName, bool bReleaseResource = false) = 0;

    /**
     * @brief Renames a backup element within the repository.
     *
     * @param strOldBackupName Current name of the backup element.
     * @param strNewBackupName New name to assign to the backup element.
     * @return true if the backup element was renamed successfully; false otherwise.
     */
    virtual bool renameBackupElement(const char *strOldBackupName, const char *strNewBackupName) = 0;

    /**
     * @brief Enumerates backup elements using a custom filter function.
     *
     * @param lpParam User-defined parameter passed to the filter function.
     * @param func Filter function used to select which elements to include.
     * @return Pointer to an INameEnum object containing matching names.
     */
    virtual INameEnum *lsBackupRepositories(void *lpParam, BACKUP_FILTER_FUNC func) = 0;

    /**
     * @brief Checks whether any changes have been made to the repository.
     *
     * @return true if the repository has unsaved changes; false otherwise.
     */
    virtual bool getRepositoriesModified() = 0;

    /**
     * @brief Retrieves the current callback interface for repository events.
     *
     * @return Pointer to the IBackupRepositoryCallback implementation.
     */
    virtual IBackupRepositoryCallback *getBackupRepositoryCallback() = 0;

    /**
     * @brief Sets the callback interface for repository events.
     *
     * @param callback Pointer to the IBackupRepositoryCallback implementation.
     */
    virtual void setBackupRepositoryCallback(IBackupRepositoryCallback * callback) = 0;
};

/**
 * @class CBackupRepositoryHelper
 * @brief Helper class for BackupRepository component.
 *
 * This class may provide utility functions or implementation support for the
 * BackupRepository component. The exact functionality is not defined in the provided context.
 */
class CBackupRepositoryHelper {};

/// Enables runtime recognition of the IBackupRepositoryCallback interface
ENABLE_RECOGNIZE_MACRO(IBackupRepositoryCallback)

/// Enables runtime recognition of the IBackupRepository interface
ENABLE_RECOGNIZE_MACRO(IBackupRepository)
