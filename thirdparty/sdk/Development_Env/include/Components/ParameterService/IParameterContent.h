#pragma once

#include "UserManager/IAuthorized.h"
#include "IParameterGroup.h"

/**
 * @interface IParameterContent
 * @brief Interface for managing parameter content including groups and individual parameters.
 *
 * This interface provides methods to initialize workspace and configuration directories,
 * create, find, rename, and remove parameter groups, manage parameters within groups,
 * and handle saving or exporting the parameter content. Authorization is required for most operations.
 */
interface IParameterContent : public IBase {
    /**
     * @brief Initializes the workspace directory for parameter content.
     *
     * @param strWorkspace Path to the workspace directory.
     * @return true if initialization succeeded; false otherwise.
     */
    virtual bool initWorkspace(const char *strWorkspace) = 0;

    /**
     * @brief Initializes the configuration directory for storing parameter data.
     *
     * @param strConfigurationDir Path to the configuration directory.
     * @return true if initialization succeeded; false otherwise.
     */
    virtual bool initConfigurationDir(const char *strConfigurationDir) = 0;

    /**
     * @brief Creates a new parameter group with the specified name.
     *
     * @param strParameterGroupName Name of the new parameter group.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the created IParameterGroup instance if successful; nullptr otherwise.
     */
    virtual IParameterGroup *createParameterGroup(const char *strParameterGroupName,
                                                  IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Finds an existing parameter group by name.
     *
     * @param strParameterGroupName Name of the parameter group to find.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameterGroup instance if found; nullptr otherwise.
     */
    virtual IParameterGroup *findParameterGroup(const char *strParameterGroupName,
                                                IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves a parameter group by name, creating it if it doesn't exist.
     *
     * @param strParameterGroupName Name of the parameter group to retrieve.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameterGroup instance if successful; nullptr otherwise.
     */
    virtual IParameterGroup *getParameterGroup(const char *strParameterGroupName, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Checks whether the specified parameter content file can be merged.
     *
     * @param strContentFilePath Path to the parameter content file.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the file can be merged; false otherwise.
     */
    virtual bool canMergeParameterContent(const char *strContentFilePath, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Merges the content from the specified parameter file.
     *
     * @param strContentFilePath Path to the parameter content file.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if merging succeeded; false otherwise.
     */
    virtual bool mergeParameterContent(const char *strContentFilePath, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Saves the current parameter content using the current configuration.
     *
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if saving succeeded; false otherwise.
     */
    virtual bool saveParameterContent(IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Saves the current parameter content under a new configuration name.
     *
     * @param strConfiguration New configuration name/path.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if saving succeeded; false otherwise.
     */
    virtual bool saveParameterContentAs(const char *strConfiguration, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Renames an existing parameter group.
     *
     * @param strOldParameterGroupName Current name of the parameter group.
     * @param strNewParameterGroupName Desired new name for the parameter group.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if renaming succeeded; false otherwise.
     */
    virtual bool renameParameterGroup(const char *strOldParameterGroupName, const char *strNewParameterGroupName,
                                      IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes the specified parameter group.
     *
     * @param strParameterGroupName Name of the parameter group to remove.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if removal succeeded; false otherwise.
     */
    virtual bool removeParameterGroup(const char *strParameterGroupName, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes all parameter groups managed by this content.
     *
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if all groups were removed successfully; false otherwise.
     */
    virtual bool removeAllParameterGroup(IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the number of parameter groups currently managed.
     *
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return The count of parameter groups.
     */
    virtual size_t getParameterGroupCount(IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the name of the parameter group at the specified index.
     *
     * @param index Zero-based index of the parameter group name to retrieve.
     * @param strParameterGroupName Buffer to receive the group name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if successful; false otherwise.
     */
    virtual bool getParameterGroupName(size_t index, char *strParameterGroupName, unsigned long &ulSize,
                                       IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Finds a parameter within the specified group by its name.
     *
     * @param strGroupName Name of the group containing the parameter.
     * @param strParameterName Name of the parameter to find.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameter instance if found; nullptr otherwise.
     */
    virtual IParameter *findParameter(const char *strGroupName, const char *strParameterName,
                                      IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves a parameter from the specified group, optionally setting a default value.
     *
     * @param strGroupName Name of the group containing the parameter.
     * @param strParameterName Name of the parameter to retrieve.
     * @param defaultValue Optional pointer to a TVARIANT structure providing the default value.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameter instance if successful; nullptr otherwise.
     */
    virtual IParameter *getParameter(const char *strGroupName, const char *strParameterName, TVARIANT *defaultValue,
                                     IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Marks the entire parameter content as modified or unmodified.
     *
     * @param bModified Boolean flag indicating whether the content is modified.
     */
    virtual void setModified(bool bModified) = 0;

    /**
     * @brief Checks whether the parameter content has been modified.
     *
     * @return true if the content is marked as modified; false otherwise.
     */
    virtual bool getModified() = 0;
};

class CParameterContentHelper {
public:
    static string getParameterGroupName(IParameterContent *pParameterContent, size_t index,
                                        IUserAuthorized *pUserAuthorized) {
        string strParameterGroupName;
        if (pParameterContent != nullptr) {
            unsigned long ulSize = 0;
            if (pParameterContent->getParameterGroupName(index, nullptr, ulSize, pUserAuthorized) == true &&
                ulSize > 0) {
                char *strParameterGroupNameA = new char[ulSize + 1];
                bzero(strParameterGroupNameA, (ulSize + 1) * sizeof(char));
                pParameterContent->getParameterGroupName(index, strParameterGroupNameA, ulSize, pUserAuthorized);
                strParameterGroupName = strParameterGroupNameA;
                delete[] strParameterGroupNameA;
                strParameterGroupNameA = nullptr;
            }
        }
        return strParameterGroupName;
    }
};

ENABLE_RECOGNIZE_MACRO(IParameterContent)
