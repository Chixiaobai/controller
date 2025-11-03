#pragma once

#include "Base/IBase.h"

#ifdef GetUserName
#undef GetUserName
#endif

// ACCESS_TYPE definition：
// READ_ACCESS, this object can be read;
// WRITE_ACCESS, this object can be writen;
// CONTROL_ACCESS, this object can be controlled which means you can change the object owner and the access flag(such as
// read access flag and write access flag).
enum ACCESS_TYPE { READ_ACCESS, WRITE_ACCESS, CONTROL_ACCESS };

#define SYSTEM_GROUP_NAME "SYSTEM"                // highest authority,can only be created by code
#define ADMINISTRATOR_GROUP_NAME "Administrator"  // can access and manage resources other than system group
#define USER_GROUP_NAME "User"                    // normal user,created by administrator

#define DEFAULT_ADMINISTRATOR_NAME "Administrator"  // Default administrator name
#define SYSTEM_USER_NAME "SYSTEM"                   // System user name

// object access flag
#define OTHER_READ_ACCESS 0x0001
#define OTHER_WRITE_ACCESS 0x0002
#define OTHER_ALL_ACCESS 0x000F

#define GROUP_READ_ACCESS 0x0010
#define GROUP_WRITE_ACCESS 0x0020
#define GROUP_ALL_ACCESS 0x00F0

#define USER_READ_ACCESS 0x0100
#define USER_WRITE_ACCESS 0x0200
#define USER_ALL_ACCESS 0x0F00

/**
 * @interface IUserAuthorized
 * @brief Interface representing user authorization information.
 *
 * This interface provides methods to retrieve the associated user group and username.
 */
interface IUserAuthorized : public IBase {
    /**
     * @brief Retrieves the user group of the authorized user.
     *
     * @param strUserGroup Buffer to receive the user group string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getUserGroup(char *strUserGroup, unsigned long &ulSize) = 0;

    /**
     * @brief Retrieves the username of the authorized user.
     *
     * @param strUserName Buffer to receive the username string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getUserName(char *strUserName, unsigned long &ulSize) = 0;
};

/**
 * @interface IAuthorizedItem
 * @brief Interface for objects that require access control and user authorization.
 *
 * This interface defines methods to set and query user ownership, manage access modes,
 * and check whether a specific access type is permitted for an authorized user.
 */
interface IAuthorizedItem : public IBase  {
    /**
     * @brief Sets the user ownership for this object.
     *
     * @param strUserGroup Name of the user group.
     * @param strUserName Name of the user.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the user was successfully set; false otherwise.
     */
    virtual bool setUser(const char *strUserGroup, const char *strUserName, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves the current user group associated with this object.
     *
     * @param strUserGroup Buffer to receive the user group string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getUserGroup(char *strUserGroup, unsigned long &ulSize) = 0;

    /**
     * @brief Retrieves the current username associated with this object.
     *
     * @param strUserName Buffer to receive the username string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getUserName(char *strUserName, unsigned long &ulSize) = 0;

    /**
     * @brief Retrieves the current access mode flags for this object.
     *
     * @return The access mode bitflags (combination of USER_*, GROUP_*, OTHER_* access flags).
     */
    virtual DWORD getAccessMode() = 0;

    /**
     * @brief Sets the access mode flags for this object.
     *
     * @param dwAccessMode New access mode bitflags.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the access mode was successfully updated; false otherwise.
     */
    virtual bool setAccessMode(DWORD dwAccessMode, IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Checks whether the specified access type is allowed for the given user.
     *
     * @param type ACCESS_TYPE to check (READ_ACCESS, WRITE_ACCESS, or CONTROL_ACCESS).
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if access is granted; false otherwise.
     */
    virtual bool access(ACCESS_TYPE type, IUserAuthorized * pUserAuthorized) = 0;
};

/**
 * @interface IUserAuthorizedCallback
 * @brief Interface for callbacks related to user authorization operations.
 *
 * This interface provides methods to retrieve user credentials such as password.
 */
interface IUserAuthorizedCallback : public IBase {
    /**
     * @brief Retrieves the user password for authentication purposes.
     *
     * @param strUserPassword Buffer to receive the password string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getUserPassword(char *strUserPassword, unsigned long &ulSize) = 0;
};
#ifndef UserManager_EXPORTS
DECLARE_STUB_INTERFACE(IAuthorizedItemStub, IAuthorizedItem)
#endif

ENABLE_RECOGNIZE_MACRO(IUserAuthorized)
ENABLE_RECOGNIZE_MACRO(IAuthorizedItem)
ENABLE_RECOGNIZE_MACRO(IUserAuthorizedCallback)
