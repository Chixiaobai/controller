#pragma once

#include "ModuleDef/IModuleVersion.h"

/**
 * @interface IModuleVersionList
 * @brief Interface for enumerating module version information.
 *
 * This interface provides methods to access a list of module versions by index,
 * including retrieving the count, specific module versions, and their names.
 */
interface IModuleVersionList : public IBase {
    /**
     * @brief Retrieves the number of module versions in the list.
     *
     * @return The count of module versions.
     */
    virtual unsigned int getModuleVersionCount() = 0;

    /**
     * @brief Retrieves the module version at the specified index.
     *
     * @param uintIndex Zero-based index of the module version to retrieve.
     * @return Pointer to the IModuleVersion instance if successful; nullptr otherwise.
     */
    virtual IModuleVersion *getModuleVersion(unsigned int uintIndex) = 0;

    /**
     * @brief Retrieves the name of the module version at the specified index.
     *
     * @param uintIndex Zero-based index of the module version name to retrieve.
     * @param strModuleVersionName Buffer to receive the module version name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getModuleVersionName(unsigned int uintIndex, char *strModuleVersionName, unsigned long &ulSize) = 0;
};

class CModuleVersionListHelper {
public:
    static bool getModuleVersionName(IModuleVersionList *pModuleVersionList, unsigned int uintIndex,
                                     string &strModuleVersionName) {
        if (pModuleVersionList != nullptr) {
            DWORD dwLen = 0;
            if (pModuleVersionList->getModuleVersionName(uintIndex, nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pModuleVersionList->getModuleVersionName(uintIndex, pValue, dwLen);
                strModuleVersionName = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        } else
            return false;
    }
};
