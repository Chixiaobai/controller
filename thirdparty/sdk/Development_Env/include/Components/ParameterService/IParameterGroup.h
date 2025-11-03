#pragma once

#include "IParameter.h"

/**
 * @interface IParameterGroup
 * @brief Interface representing a group of related parameters.
 *
 * This interface provides methods to create, find, retrieve, rename, and remove parameters
 * within the group. It also supports enumeration of parameters, modification tracking,
 * and requires user authorization for most operations.
 */
interface IParameterGroup : public IBase {
    /**
     * @brief Creates a new parameter within the group.
     *
     * @param strParameterName Name of the new parameter.
     * @param defaultValue Pointer to a TVARIANT structure containing the default value.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the created IParameter instance if successful; nullptr otherwise.
     */
    virtual IParameter *createParameter(const char *strParameterName, TVARIANT *defaultValue,
                                        IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Finds an existing parameter by name within the group.
     *
     * @param strParameterName Name of the parameter to find.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameter instance if found; nullptr otherwise.
     */
    virtual IParameter *findParameter(const char *strParameterName, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves a parameter by name, creating it with a default value if it doesn't exist.
     *
     * @param strParameterName Name of the parameter to retrieve or create.
     * @param defaultValue Pointer to a TVARIANT structure providing the default value.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameter instance if successful; nullptr otherwise.
     */
    virtual IParameter *getParameter(const char *strParameterName, TVARIANT *defaultValue,
                                     IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Renames an existing parameter within the group.
     *
     * @param strOldParameterName Current name of the parameter.
     * @param strNewParameterName Desired new name for the parameter.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if renaming succeeded; false otherwise.
     */
    virtual bool renameParameter(const char *strOldParameterName, const char *strNewParameterName,
                                 IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes the specified parameter from the group.
     *
     * @param strParameterName Name of the parameter to remove.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if removal succeeded; false otherwise.
     */
    virtual bool removeParameter(const char *strParameterName, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes all parameters from the group.
     *
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if all parameters were removed successfully; false otherwise.
     */
    virtual bool removeAllParameter(IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the number of parameters currently in the group.
     *
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return The count of parameters in the group.
     */
    virtual size_t getParameterCount(IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the name of the parameter at the specified index.
     *
     * @param index Zero-based index of the parameter name to retrieve.
     * @param strParameterName Buffer to receive the parameter name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if successful; false otherwise.
     */
    virtual bool getParameterName(size_t index, char *strParameterName, unsigned long &ulSize,
                                  IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Marks the entire parameter group as modified or unmodified.
     *
     * @param bModified Boolean flag indicating whether the group is modified.
     */
    virtual void setModified(bool bModified) = 0;

    /**
     * @brief Checks whether the parameter group has been modified.
     *
     * @return true if the group is marked as modified; false otherwise.
     */
    virtual bool getModified() = 0;
};

class CParameterGroupHelper {
public:
    static string getParameterName(IParameterGroup *pParameterGroup, size_t index, IUserAuthorized *pUserAuthorized) {
        string strParameterName;
        if (pParameterGroup != nullptr) {
            unsigned long ulSize = 0;
            if (pParameterGroup->getParameterName(index, nullptr, ulSize, pUserAuthorized) == true && ulSize > 0) {
                char *strParameterNameA = new char[ulSize + 1];
                bzero(strParameterNameA, (ulSize + 1) * sizeof(char));
                pParameterGroup->getParameterName(index, strParameterNameA, ulSize, pUserAuthorized);
                strParameterName = strParameterNameA;
                delete[] strParameterNameA;
                strParameterNameA = nullptr;
            }
        }
        return strParameterName;
    }
};

ENABLE_RECOGNIZE_MACRO(IParameterGroup)
