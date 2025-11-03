#pragma once

#include "IBackupRepository.h"

/**
 * @def UNIQUE_BACKUPDATABASE_NAME
 * @brief Unique identifier name for the BackupDatabase component.
 *
 * This macro defines the unique name used to identify the BackupDatabase component
 * within the system, useful for logging, debugging, and component management.
 */
#define UNIQUE_BACKUPDATABASE_NAME "BackupDatabase"

/**
 * @interface IBackupDatabaseInner
 * @brief Interface for managing backup database operations.
 *
 * This interface provides methods to configure the backup workspace, create,
 * find, and destroy backup repositories with customizable storage strategies.
 */
interface IBackupDatabaseInner : public IBase {
    /**
     * @brief Sets the workspace directory for the backup database.
     *
     * @param strWorkspace Path to the directory where backups will be stored.
     * @return true if the workspace was successfully set; false otherwise.
     */
    virtual bool setBackupDatabaseWorkspace(const char *strWorkspace) = 0;

    /**
     * @brief Creates a new backup repository with the specified name and strategy.
     *
     * @param strRepositoryName Name of the repository to create.
     * @param strategy Storage strategy to use (default is ROLLING_STORAGE).
     * @return true if the repository was successfully created; false otherwise.
     */
    virtual bool createBackupRepository(const char *strRepositoryName, STORAGE_STRATEGY strategy = ROLLING_STORAGE) = 0;

    /**
     * @brief Finds an existing backup repository by name.
     *
     * @param strRepositoryName Name of the repository to find.
     * @return Pointer to the IBackupRepository instance if found; nullptr otherwise.
     */
    virtual IBackupRepository *findBackupRepository(const char *strRepositoryName) = 0;

    /**
     * @brief Destroys a backup repository identified by its name.
     *
     * @param strRepositoryName Name of the repository to destroy.
     * @return true if the repository was successfully destroyed; false otherwise.
     */
    virtual bool destroyBackupRepository(const char *strRepositoryName) = 0;
};

/**
 * @class CBackupDatabaseHelper
 * @brief Helper class for BackupDatabase component.
 *
 * This class may provide utility functions or implementation support for the
 * BackupDatabase component. The exact functionality is not defined in the provided context.
 */
class CBackupDatabaseHelper {};

/// Enables runtime recognition of the IBackupDatabaseInner interface
ENABLE_RECOGNIZE_MACRO(IBackupDatabaseInner)
