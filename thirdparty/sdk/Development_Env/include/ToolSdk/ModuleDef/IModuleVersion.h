/**
 * @file IModuleVersion.h
 * @brief Header file for module version management.
 *
 * This file defines an interface and a helper class for managing module versions.
 */

#ifndef __TOOLSDK_I_MODULE_VERSION_H__
#define __TOOLSDK_I_MODULE_VERSION_H__

#include "Base/IBase.h"

/**
 * @interface IModuleVersion
 * @brief Interface for managing module versions.
 *
 * This interface defines methods for retrieving different versions of a module.
 */
interface IModuleVersion : public IBase {
    /**
     * @brief Retrieves the build version of the module.
     *
     * Retrieves the build version of the module and stores it in the provided buffer.
     *
     * @param strBuildVersion Pointer to the buffer to store the build version.
     * @param ulVersionLen Reference to the length of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool getBuildVersion(char *strBuildVersion, unsigned long &ulVersionLen) = 0;

    /**
     * @brief Retrieves the full version of the module.
     *
     * Retrieves the full version of the module and stores it in the provided buffer.
     *
     * @param strFullVersion Pointer to the buffer to store the full version.
     * @param ulVersionLen Reference to the length of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool getFullVersion(char *strFullVersion, unsigned long &ulVersionLen) = 0;

    /**
     * @brief Retrieves the short version of the module.
     *
     * Retrieves the short version of the module and stores it in the provided buffer.
     *
     * @param strShortVersion Pointer to the buffer to store the short version.
     * @param ulVersionLen Reference to the length of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool getShortVersion(char *strShortVersion, unsigned long &ulVersionLen) = 0;

    /**
     * @brief Retrieves the major version number of the module.
     *
     * Retrieves the major version number of the module.
     *
     * @return The major version number.
     */
    virtual unsigned int getMajorVersionNum() = 0;

    /**
     * @brief Retrieves the minor version number of the module.
     *
     * Retrieves the minor version number of the module.
     *
     * @return The minor version number.
     */
    virtual unsigned int getMinorVersionNum() = 0;

    /**
     * @brief Retrieves the revision number of the module.
     *
     * Retrieves the revision number of the module.
     *
     * @return The revision number.
     */
    virtual unsigned int getRevisionNum() = 0;
};

/**
 * @class CModuleVersionHelper
 * @brief Helper class for managing module versions.
 *
 * This class provides static methods to assist with retrieving module versions.
 */
class CModuleVersionHelper {
public:
    /**
     * @brief Retrieves the build version of the module.
     *
     * Retrieves the build version of the module and stores it in the provided string.
     *
     * @param pModuleVersion Pointer to the module version.
     * @param strBuildVersion Reference to the string to store the build version.
     * @return true if the operation was successful, false otherwise.
     */
    static bool getBuildVersion(IModuleVersion *pModuleVersion, string &strBuildVersion) {
        if (pModuleVersion != nullptr) {
            DWORD dwLen = 0;
            if (pModuleVersion->getBuildVersion(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pModuleVersion->getBuildVersion(pValue, dwLen);
                strBuildVersion = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        } else
            return false;
    }

    /**
     * @brief Retrieves the full version of the module.
     *
     * Retrieves the full version of the module and stores it in the provided string.
     *
     * @param pModuleVersion Pointer to the module version.
     * @param strFullVersion Reference to the string to store the full version.
     * @return true if the operation was successful, false otherwise.
     */
    static bool getFullVersion(IModuleVersion *pModuleVersion, string &strFullVersion) {
        if (pModuleVersion != nullptr) {
            DWORD dwLen = 0;
            if (pModuleVersion->getFullVersion(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pModuleVersion->getFullVersion(pValue, dwLen);
                strFullVersion = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        } else
            return false;
    }

    /**
     * @brief Retrieves the short version of the module.
     *
     * Retrieves the short version of the module and stores it in the provided string.
     *
     * @param pModuleVersion Pointer to the module version.
     * @param strShortVersion Reference to the string to store the short version.
     * @return true if the operation was successful, false otherwise.
     */
    static bool getShortVersion(IModuleVersion *pModuleVersion, string &strShortVersion) {
        if (pModuleVersion != nullptr) {
            DWORD dwLen = 0;
            if (pModuleVersion->getShortVersion(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pModuleVersion->getShortVersion(pValue, dwLen);
                strShortVersion = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        } else
            return false;
    }
};

#endif  // __TOOLSDK_I_MODULE_VERSION_H__