#pragma once

#include "Base/IBase.h"

enum BACKUP_TYPE {
    INVALID_BACKUP = 0,
    FILE_BACKUP,
};

interface IBackupCallback;
/**
 * @interface IBackupElement
 * @brief Interface for managing backup elements in a backup system.
 *
 * This interface provides methods to access and manipulate backup information including
 * workspace, name, type, size, properties, timestamp, and modification status. It also
 * supports performing backup and restore operations.
 */
interface IBackupElement : public IBase {
    /**
     * @brief Retrieves the workspace path associated with this backup element.
     *
     * @param strWorkspace Buffer to receive the workspace path string.
     * @param dwLen On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getBackupWorkspace(char *strWorkspace, DWORD &dwLen) = 0;

    /**
     * @brief Retrieves the repository name of this backup element.
     *
     * @param strRepositoryName Buffer to receive the repository name string.
     * @param dwLen On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getBackupName(char *strRepositoryName, DWORD &dwLen) = 0;

    /**
     * @brief Retrieves the type of this backup element.
     *
     * @return BACKUP_TYPE value indicating the type of backup.
     */
    virtual BACKUP_TYPE getBackupType() = 0;

    /**
     * @brief Retrieves the total size of the backup data in bytes.
     *
     * @return Size of the backup in bytes.
     */
    virtual unsigned long getBackupSize() = 0;

    /**
     * @brief Retrieves the number of properties associated with this backup element.
     *
     * @return Number of properties available.
     */
    virtual unsigned int getBackupPropertyCount() = 0;

    /**
     * @brief Retrieves the name of a property at the specified index.
     *
     * @param uintIndex Zero-based index of the property name to retrieve.
     * @param strPropertyName Buffer to receive the property name string.
     * @param dwLen On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getBackupPropertyName(unsigned int uintIndex, char *strPropertyName, DWORD &dwLen) = 0;

    /**
     * @brief Retrieves the timestamp when this backup was last modified.
     *
     * @param timestamp Pointer to time_t where the timestamp will be stored.
     */
    virtual void getBackupTimestamp(time_t * timestamp) = 0;

    /**
     * @brief Retrieves the value of a specific property by name.
     *
     * @param strPropertyName Name of the property to retrieve.
     * @param strPropertyValue Buffer to receive the property value string.
     * @param dwLen On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getBackupProperty(const char *strPropertyName, char *strPropertyValue, DWORD &dwLen) = 0;

    /**
     * @brief Sets or updates the value of a specific property.
     *
     * @param strPropertyName Name of the property to set.
     * @param strPropertyValue New value for the property.
     * @return true if the property was successfully updated; false otherwise.
     */
    virtual bool setBackupProperty(const char *strPropertyName, const char *strPropertyValue) = 0;

    /**
     * @brief Checks whether this backup element has been modified.
     *
     * @return true if modified; false otherwise.
     */
    virtual bool getBackupModified() = 0;

    /**
     * @brief Marks this backup element as modified or unmodified.
     *
     * @param bModified true to mark as modified; false to mark as clean.
     */
    virtual void setBackupModified(const bool bModified) = 0;

    /**
     * @brief Performs the backup operation for this element.
     *
     * @return true if the backup succeeded; false otherwise.
     */
    virtual bool doBackup() = 0;

    /**
     * @brief Restores the data from this backup element.
     *
     * @return true if the restore succeeded; false otherwise.
     */
    virtual bool doRestore() = 0;
};

class CBackupElementHelper {
public:
    static string getBackupWorkspace(IBackupElement *pBackupElement) {
        string strBackupElementWorkspace;
        if (pBackupElement != nullptr) {
            unsigned long ulSize = 0;
            if (pBackupElement->getBackupWorkspace(nullptr, ulSize) == true && ulSize > 0) {
                char *strBackupElementWorkspaceA = new char[ulSize + 1];
                bzero(strBackupElementWorkspaceA, (ulSize + 1) * sizeof(char));
                pBackupElement->getBackupWorkspace(strBackupElementWorkspaceA, ulSize);
                strBackupElementWorkspace = strBackupElementWorkspaceA;
                delete[] strBackupElementWorkspaceA;
                strBackupElementWorkspaceA = nullptr;
            }
        }
        return strBackupElementWorkspace;
    }
    static string getBackupName(IBackupElement *pBackupElement) {
        string strBackupElementName;
        if (pBackupElement != nullptr) {
            unsigned long ulSize = 0;
            if (pBackupElement->getBackupName(nullptr, ulSize) == true && ulSize > 0) {
                char *strBackupElementNameA = new char[ulSize + 1];
                bzero(strBackupElementNameA, (ulSize + 1) * sizeof(char));
                pBackupElement->getBackupName(strBackupElementNameA, ulSize);
                strBackupElementName = strBackupElementNameA;
                delete[] strBackupElementNameA;
                strBackupElementNameA = nullptr;
            }
        }
        return strBackupElementName;
    }
    static string getBackupPropertyName(IBackupElement *pBackupElement, unsigned int uintIndex) {
        string strBackupPropertyName;
        if (pBackupElement != nullptr) {
            unsigned long ulSize = 0;
            if (pBackupElement->getBackupPropertyName(uintIndex, nullptr, ulSize) == true && ulSize > 0) {
                char *strBackupPropertyNameA = new char[ulSize + 1];
                bzero(strBackupPropertyNameA, (ulSize + 1) * sizeof(char));
                pBackupElement->getBackupPropertyName(uintIndex, strBackupPropertyNameA, ulSize);
                strBackupPropertyName = strBackupPropertyNameA;
                delete[] strBackupPropertyNameA;
                strBackupPropertyNameA = nullptr;
            }
        }
        return strBackupPropertyName;
    }
    static string getBackupProperty(IBackupElement *pBackupElement, const char *strPropertyName) {
        string strBackupPropertyValue;
        if (pBackupElement != nullptr) {
            unsigned long ulSize = 0;
            if (pBackupElement->getBackupProperty(strPropertyName, nullptr, ulSize) == true && ulSize > 0) {
                char *strBackupPropertyValueA = new char[ulSize + 1];
                bzero(strBackupPropertyValueA, (ulSize + 1) * sizeof(char));
                pBackupElement->getBackupProperty(strPropertyName, strBackupPropertyValueA, ulSize);
                strBackupPropertyValue = strBackupPropertyValueA;
                delete[] strBackupPropertyValueA;
                strBackupPropertyValueA = nullptr;
            }
        }
        return strBackupPropertyValue;
    }
};

ENABLE_RECOGNIZE_MACRO(IBackupElement)
ENABLE_RECOGNIZE_MACRO(IBackupCallback)
