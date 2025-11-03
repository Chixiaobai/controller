#pragma once

#define UNIQUE_USERMANAGER_NAME "UserManager"

#include "IAuthorized.h"

/**
 * @interface IUserManagerInner
 * @brief Interface for managing user authentication and authorization within the system.
 *
 * This interface provides methods to retrieve and set the current user's authorization,
 * create and remove users, set passwords, and manage persistent user database operations.
 */
interface IUserManagerInner : public IBase {
    /**
     * @brief Retrieves the system-level authorized user object with highest privileges.
     *
     * @return Pointer to the IUserAuthorized instance representing the system user.
     */
    virtual IUserAuthorized *getSystemUserAuthorized() = 0;

    /**
     * @brief Retrieves the currently authenticated user's authorization information.
     *
     * @return Pointer to the IUserAuthorized instance representing the current user.
     */
    virtual IUserAuthorized *getCurrentUserAuthorized() = 0;

    /**
     * @brief Sets the current user's authorization using provided credentials.
     *
     * @param strUserGroup Name of the user group.
     * @param strUserName Username of the user.
     * @param strPassword Password for authentication.
     * @return true if the current user was successfully set; false otherwise.
     */
    virtual bool setCurrentUserAuthorized(const char *strUserGroup, const char *strUserName,
                                          const char *strPassword) = 0;

    /**
     * @brief Creates a new user with the specified group, name, and password.
     *
     * @param strUserGroup Name of the user group.
     * @param strUserName Username of the new user.
     * @param strPassword Password for the new user.
     * @return true if the user was successfully created; false otherwise.
     */
    virtual bool createUser(const char *strUserGroup, const char *strUserName, const char *strPassword) = 0;

    /**
     * @brief Removes an existing user from the specified group.
     *
     * @param strUserGroup Name of the user group.
     * @param strUserName Username of the user to remove.
     * @return true if the user was successfully removed; false otherwise.
     */
    virtual bool removeUser(const char *strUserGroup, const char *strUserName) = 0;

    /**
     * @brief Sets or updates the password for a specific user in the given group.
     *
     * @param strUserGroup Name of the user group.
     * @param strUserName Username of the target user.
     * @param strPassword New password to set.
     * @param pUserAuthorizedCallback Optional callback for additional authorization steps.
     * @return true if the password was successfully updated; false otherwise.
     */
    virtual bool setUserPassword(const char *strUserGroup, const char *strUserName, const char *strPassword,
                                 IUserAuthorizedCallback *pUserAuthorizedCallback = nullptr) = 0;

    /**
     * @brief Loads the user database from persistent storage (e.g., configuration files).
     */
    virtual void loadUserDatabase() = 0;

    /**
     * @brief Saves the current user database to persistent storage (e.g., configuration files).
     */
    virtual void saveUserDatabase() = 0;
};

ENABLE_RECOGNIZE_MACRO(IUserManagerInner)
