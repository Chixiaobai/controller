/**
 * @file ModuleDef.h
 * @brief Header file for module definition management.
 *
 * This file defines functions and a class for managing module definitions and versions.
 */

#ifndef __TOOLSDK_MODULE_DEF_H__
#define __TOOLSDK_MODULE_DEF_H__

#include "ToolSdkDef.h"
#include "ModuleDef/IModuleVariants.h"
#include "ModuleDef/IModuleVersion.h"

/**
 * @fn IModuleVariants *module_def_create_module_variables(const char *module_config)
 * @brief Creates module variables.
 *
 * Creates module variables based on the provided configuration.
 *
 * @param module_config The configuration string for the module variables.
 * @return Pointer to the created module variables, or nullptr if the operation failed.
 */
TOOLSDK_API IModuleVariants *module_def_create_module_variables(const char *module_config);

/**
 * @fn IModuleVersion *module_def_get_module_version()
 * @brief Retrieves the ToolSDK library version.
 *
 * Retrieves the version of the ToolSDK library.
 *
 * @return Pointer to the module version, or nullptr if the operation failed.
 */
TOOLSDK_API IModuleVersion *module_def_get_module_version();

/**
 * @class CModuleDef
 * @brief Class for managing module definitions and versions.
 *
 * This class provides static methods to create module variables and retrieve the ToolSDK library version.
 */
class CModuleDef {
public:
    /**
     * @brief Creates module variables.
     *
     * Creates module variables based on the provided configuration.
     *
     * @param module_config The configuration string for the module variables.
     * @return Pointer to the created module variables, or nullptr if the operation failed.
     */
    static IModuleVariants *createModulesVariables(const char *module_config) {
        return module_def_create_module_variables(module_config);
    }

    /**
     * @brief Retrieves the ToolSDK library version.
     *
     * Retrieves the version of the ToolSDK library.
     *
     * @return Pointer to the module version, or nullptr if the operation failed.
     */
    static IModuleVersion *getModuleVersion() { return module_def_get_module_version(); }
};

#endif  // __TOOLSDK_MODULE_DEF_H__