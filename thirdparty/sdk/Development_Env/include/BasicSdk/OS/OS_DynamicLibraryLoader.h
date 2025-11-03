#ifndef __BASICSDK_OS_DYNAMIC_LIBRARY_LOADER_H__
#define __BASICSDK_OS_DYNAMIC_LIBRARY_LOADER_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Enumeration for dynamic library loading hints
 */
enum os_load_hint {
    resolve_all_symbols_hint = 0x01,     /**< Resolve all symbols at load time */
    export_external_symbols_hint = 0x02, /**< Export external symbols */
    prevent_unload_hint = 0x04,          /**< Prevent the library from being unloaded */
    deep_bind_hint = 0x08                /**< Use deep binding for symbols */
};

/**
 * @brief Load a dynamic library with default hints
 *
 * This function uses the default hint `resolve_all_symbols_hint`.
 *
 * @param strLibraryPath Path to the dynamic library
 * @return OS_MODULE_HANDLE Handle to the loaded module
 */
BASICSDK_API OS_MODULE_HANDLE os_dynamic_library_loader_load(const char *strLibraryPath);

/**
 * @brief Load a dynamic library with specified hints
 *
 * @param strLibraryPath Path to the dynamic library
 * @param hints Loading hints (combination of os_load_hint values)
 * @return OS_MODULE_HANDLE Handle to the loaded module
 */
BASICSDK_API OS_MODULE_HANDLE os_dynamic_library_loader_load_ex(const char *strLibraryPath, int hints);

/**
 * @brief Get a procedure address from a loaded dynamic library
 *
 * @param hModuleHandle Handle to the loaded module
 * @param strProcName Name of the procedure
 * @return void* Pointer to the procedure, or nullptr if not found
 */
BASICSDK_API void *os_dynamic_library_loader_get_proc(OS_MODULE_HANDLE hModuleHandle, const char *strProcName);

/**
 * @brief Free a loaded dynamic library
 *
 * @param hModuleHandle Handle to the loaded module
 */
BASICSDK_API void os_dynamic_library_loader_free(OS_MODULE_HANDLE hModuleHandle);

/**
 * @brief Namespace for dynamic library loading functions and classes
 */
namespace OS {
class DynamicLibraryLoader {
public:
    /**
     * @brief Load a dynamic library with default hints
     *
     * This function uses the default hint `resolve_all_symbols_hint`.
     *
     * @param strLibraryPath Path to the dynamic library
     * @return OS_MODULE_HANDLE Handle to the loaded module
     */
    static OS_MODULE_HANDLE load(const string &strLibraryPath) {
        return os_dynamic_library_loader_load(strLibraryPath.c_str());
    }

    /**
     * @brief Load a dynamic library with specified hints
     *
     * @param strLibraryPath Path to the dynamic library
     * @param hints Loading hints (combination of os_load_hint values)
     * @return OS_MODULE_HANDLE Handle to the loaded module
     */
    static OS_MODULE_HANDLE loadEx(const string &strLibraryPath, int hints) {
        return os_dynamic_library_loader_load_ex(strLibraryPath.c_str(), hints);
    }

    /**
     * @brief Get a procedure address from a loaded dynamic library
     *
     * @param hModuleHandle Handle to the loaded module
     * @param strProcName Name of the procedure
     * @return void* Pointer to the procedure, or nullptr if not found
     */
    static void *getProc(OS_MODULE_HANDLE hModuleHandle, const string &strProcName) {
        return os_dynamic_library_loader_get_proc(hModuleHandle, strProcName.c_str());
    }

    /**
     * @brief Free a loaded dynamic library
     *
     * @param hModuleHandle Handle to the loaded module
     */
    static void free(OS_MODULE_HANDLE hModuleHandle) { return os_dynamic_library_loader_free(hModuleHandle); }
};
}  // namespace OS

#endif