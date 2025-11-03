#pragma once

#define UNIQUE_MODULEVERSIONMANAGER_NAME "ModuleVersionManager"

#include "IModuleVersionList.h"

/**
 * @interface IModuleVersionManagerInner
 * @brief Interface for managing module version information.
 *
 * This interface provides methods to set, register, unregister, and query module versions,
 * including both root module versions and specific component module versions.
 */
interface IModuleVersionManagerInner : public IBase {
    /**
     * @brief Sets the root module version information.
     *
     * @param strBuildVersion Build version string.
     * @param strFullVersion Full version string (e.g., "1.2.3.4").
     * @param strShortVersion Short version string (e.g., "1.2").
     * @param intMajorVersion Major version number.
     * @param intMinorVersion Minor version number.
     * @param intRevisionVersion Revision version number.
     * @return true if successful; false otherwise.
     */
    virtual bool setRootModuleVersion(const char *strBuildVersion, const char *strFullVersion,
                                      const char *strShortVersion, const int intMajorVersion, const int intMinorVersion,
                                      const int intRevisionVersion) = 0;

    /**
     * @brief Registers a module with its version information.
     *
     * @param strModuleNamespace Namespace of the module.
     * @param strModuleName Name of the module.
     * @param strBuildVersion Build version string.
     * @param strFullVersion Full version string.
     * @param strShortVersion Short version string.
     * @param intMajorVersion Major version number.
     * @param intMinorVersion Minor version number.
     * @param intRevisionVersion Revision version number.
     * @return true if registration succeeded; false otherwise.
     */
    virtual bool registerModuleVersion(const char *strModuleNamespace, const char *strModuleName,
                                       const char *strBuildVersion, const char *strFullVersion,
                                       const char *strShortVersion, const int intMajorVersion,
                                       const int intMinorVersion, const int intRevisionVersion) = 0;

    /**
     * @brief Unregisters a previously registered module.
     *
     * @param strModuleNamespace Namespace of the module to unregister.
     * @param strModuleName Name of the module to unregister.
     * @return true if unregistration succeeded; false otherwise.
     */
    virtual bool unregisterModuleVersion(const char *strModuleNamespace, const char *strModuleName) = 0;

    /**
     * @brief Retrieves version information for a specific module.
     *
     * If strModuleName is nullptr, retrieves the root module version.
     *
     * @param strModuleNamespace Namespace of the requested module.
     * @param strModuleName Name of the requested module.
     * @return Pointer to the IModuleVersion instance if found; nullptr otherwise.
     */
    virtual IModuleVersion *findModuleVersion(const char *strModuleNamespace, const char *strModuleName) = 0;

    /**
     * @brief Retrieves a list of all modules in the specified namespace.
     *
     * @param strModuleNamespace Namespace to search for modules.
     * @return Pointer to an IModuleVersionList containing all modules in the namespace.
     */
    virtual IModuleVersionList *findAllModuleVersion(const char *strModuleNamespace) = 0;
};
