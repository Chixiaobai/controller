#pragma once

#define UNIQUE_PARAMETERSERVICE_NAME "ParameterService"

#define GLOBAL_PARAMETER_NAMESPACE "Global"

#include "IParameterContent.h"

/**
 * @interface IParameterAcceleratedAgent
 * @brief Interface for accelerating parameter access through array index mapping.
 *
 * This interface provides methods to map parameter names to specific array indices
 * for faster access, and retrieve or release these mapped parameters.
 */
interface IParameterAcceleratedAgent : public IBase {
    /**
     * @brief Maps a parameter to a specific array index for accelerated access.
     *
     * @param strNamespace Namespace of the parameter.
     * @param strGroupName Group name containing the parameter.
     * @param strParameterName Name of the parameter.
     * @param uItemId Array index used for mapping (accelerated ID).
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the mapping was successfully set; false otherwise.
     */
    virtual bool setAcceleratedItem(const char *strNamespace, const char *strGroupName, const char *strParameterName,
                                    unsigned int uItemId, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes the mapping for a specific array index.
     *
     * @param uItemId Array index (accelerated ID) to unmap.
     * @return true if the mapping was successfully removed; false otherwise.
     */
    virtual bool unsetAcceleratedItem(unsigned int uItemId) = 0;

    /**
     * @brief Retrieves the parameter mapped to the specified array index.
     *
     * @param uItemId Array index (accelerated ID) to query.
     * @return Pointer to the IParameter instance if found; nullptr otherwise.
     */
    virtual IParameter *getAcceleratedItem(unsigned int uItemId) = 0;
};

/**
 * @interface IParameterServiceInner
 * @brief Core interface for managing parameter contents and global parameter operations.
 *
 * This interface provides methods to create, find, rename, and remove parameter contents,
 * access individual parameters with namespace support, and manage accelerated access mappings.
 */
interface IParameterServiceInner : public IBase {
    /**
     * @brief Sets the workspace directory for parameter service operations.
     *
     * @param strWorkspace Path to the workspace directory.
     */
    virtual void setWorkspace(const char *strWorkspace) = 0;

    /**
     * @brief Sets the configuration directory for storing parameter data.
     *
     * @param strConfigurationDir Path to the configuration directory.
     */
    virtual void setConfigurationDir(const char *strConfigurationDir) = 0;

    /**
     * @brief Creates a new parameter content with the specified name.
     *
     * @param strParameterContentName Name of the new parameter content.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the created IParameterContent instance if successful; nullptr otherwise.
     */
    virtual IParameterContent *createParameterContent(const char *strParameterContentName,
                                                      IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Creates a new parameter content with custom workspace and configuration directories.
     *
     * @param strWorkspace Custom workspace path.
     * @param strConfigurationDir Custom configuration directory path.
     * @param strParameterContentName Name of the new parameter content.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the created IParameterContent instance if successful; nullptr otherwise.
     */
    virtual IParameterContent *createParameterContentEx(const char *strWorkspace, const char *strConfigurationDir,
                                                        const char *strParameterContentName,
                                                        IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Finds an existing parameter content by name.
     *
     * @param strParameterContentName Name of the parameter content to find.
     * @return Pointer to the IParameterContent instance if found; nullptr otherwise.
     */
    virtual IParameterContent *findParameterContent(const char *strParameterContentName) = 0;

    /**
     * @brief Retrieves a parameter content by name, creating it if it doesn't exist.
     *
     * @param strParameterContentName Name of the parameter content to retrieve.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameterContent instance if successful; nullptr otherwise.
     */
    virtual IParameterContent *getParameterContent(const char *strParameterContentName,
                                                   IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Renames an existing parameter content.
     *
     * @param strOldParameterContentName Current name of the parameter content.
     * @param strNewParameterContentName Desired new name.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if renaming succeeded; false otherwise.
     */
    virtual bool renameParameterContent(const char *strOldParameterContentName, const char *strNewParameterContentName,
                                        IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes the specified parameter content.
     *
     * @param strParameterContentName Name of the parameter content to remove.
     * @param bRemoveConfig true to remove associated configuration files; false otherwise.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if removal succeeded; false otherwise.
     */
    virtual bool removeParameterContent(const char *strParameterContentName, bool bRemoveConfig,
                                        IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Removes all parameter contents managed by this service.
     *
     * @param bRemoveConfig true to remove associated configuration files; false otherwise.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if all contents were removed successfully; false otherwise.
     */
    virtual bool removeAllParameterContent(bool bRemoveConfig, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves the number of parameter contents currently managed.
     *
     * @return The count of parameter contents.
     */
    virtual size_t getParameterContentCount() = 0;

    /**
     * @brief Retrieves the name of the parameter content at the specified index.
     *
     * @param index Zero-based index of the parameter content name to retrieve.
     * @param strParameterContentName Buffer to receive the content name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getParameterContentName(size_t index, char *strParameterContentName, unsigned long &ulSize) = 0;

    /**
     * @brief Finds a parameter across namespaces, groups, and contents by its full path.
     *
     * @param strNamespace Namespace containing the parameter.
     * @param strGroupName Group containing the parameter.
     * @param strParameterName Name of the parameter.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameter instance if found; nullptr otherwise.
     */
    virtual IParameter *findParameter(const char *strNamespace, const char *strGroupName, const char *strParameterName,
                                      IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves a parameter by its full path, creating it with a default value if it doesn't exist.
     *
     * @param strNamespace Namespace containing the parameter.
     * @param strGroupName Group containing the parameter.
     * @param strParameterName Name of the parameter.
     * @param defaultValue Pointer to a TVARIANT structure providing the default value.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return Pointer to the IParameter instance if successful; nullptr otherwise.
     */
    virtual IParameter *getParameter(const char *strNamespace, const char *strGroupName, const char *strParameterName,
                                     TVARIANT *defaultValue, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Creates a new parameter acceleration agent.
     *
     * @param uMaxItemCount Maximum number of items that can be mapped.
     * @return Pointer to the created IParameterAcceleratedAgent instance if successful; nullptr otherwise.
     */
    virtual IParameterAcceleratedAgent *createAcceleratedAgent(unsigned int uMaxItemCount) = 0;

    /**
     * @brief Locks all parameter contents to prevent concurrent modifications.
     */
    virtual void lockParameterContents() = 0;

    /**
     * @brief Unlocks all parameter contents after modification.
     */
    virtual void unlockParameterContents() = 0;
};

class CParameterServiceHelper {
public:
    static string getParameterContentName(IParameterServiceInner *pParameterService, size_t index) {
        string strParameterContentName;
        if (pParameterService != nullptr) {
            unsigned long ulSize = 0;
            if (pParameterService->getParameterContentName(index, nullptr, ulSize) == true && ulSize > 0) {
                char *strParameterContentNameA = new char[ulSize + 1];
                bzero(strParameterContentNameA, (ulSize + 1) * sizeof(char));
                pParameterService->getParameterContentName(index, strParameterContentNameA, ulSize);
                strParameterContentName = strParameterContentNameA;
                delete[] strParameterContentNameA;
                strParameterContentNameA = nullptr;
            }
        }
        return strParameterContentName;
    }
    static bool isAbsoluteParameterName(string strAbsoluteParameterName) {
        size_t first_split_pos = strAbsoluteParameterName.find("::");
        if (first_split_pos == strAbsoluteParameterName.npos)
            return false;
        else {
            size_t second_split_pos = strAbsoluteParameterName.find("::", first_split_pos + 2);
            if (second_split_pos == strAbsoluteParameterName.npos) return false;
            return true;
        }
    }
    static bool parserAbsoluteParameterName(string strAbsoluteParameterName, string &strNamespace, string &strGroupName,
                                            string &strParameterName) {
        size_t first_split_pos = strAbsoluteParameterName.find("::");
        if (first_split_pos == strAbsoluteParameterName.npos)
            return false;
        else {
            size_t second_split_pos = strAbsoluteParameterName.find("::", first_split_pos + 2);
            if (second_split_pos == strAbsoluteParameterName.npos) return false;
            strNamespace = strAbsoluteParameterName.substr(0, first_split_pos);
            strGroupName = strAbsoluteParameterName.substr(first_split_pos + 2, second_split_pos - first_split_pos - 2);
            strParameterName = strAbsoluteParameterName.substr(second_split_pos + 2,
                                                               strAbsoluteParameterName.size() - second_split_pos - 2);
            return true;
        }
    }
    static bool getAbsoluteParameterName(string strNamespace, string strGroupName, string strParameterName,
                                         string &strAbsoluteParameterName) {
        if (strNamespace.empty() == true || strGroupName.empty() == true || strParameterName.empty() == true)
            return false;
        strAbsoluteParameterName = strNamespace + "::" + strGroupName + "::" + strParameterName;
        return true;
    }
};

ENABLE_RECOGNIZE_MACRO(IParameterAcceleratedAgent)
ENABLE_RECOGNIZE_MACRO(IParameterServiceInner)
