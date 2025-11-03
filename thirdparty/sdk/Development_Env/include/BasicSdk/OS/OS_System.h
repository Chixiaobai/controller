#ifndef __BASICSDK_OS_SYSTEM_H__
#define __BASICSDK_OS_SYSTEM_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Run an application
 *
 * @param strApplicationName Name of the application to run
 * @param strCommandLine Command line arguments for the application (optional)
 * @param bBlockParent If true, block the parent process until the application exits
 * @return bool True if the application is started successfully, false otherwise
 */
BASICSDK_API bool os_system_run_application(const char *strApplicationName, const char *strCommandLine = "",
                                            bool bBlockParent = false);

/**
 * @brief Forcefully kill an application
 *
 * Note: This function will forcefully terminate the application, potentially causing data loss.
 * It is not recommended for use in normal circumstances.
 *
 * @param strApplicationName Name of the application to kill
 * @return bool True if the application is killed successfully, false otherwise
 */
BASICSDK_API bool os_system_kill_application(const char *strApplicationName);

/**
 * @brief Get the value of an operating system environment variable
 *
 * @param strEnvironmentVariableName Name of the environment variable
 * @param strEnvironmentVariableValue Buffer to store the value of the environment variable
 * @param dwLen Length of the buffer
 * @return bool True if the environment variable is retrieved successfully, false otherwise
 */
BASICSDK_API bool os_system_get_os_environmentVariable(const char *strEnvironmentVariableName,
                                                       char *strEnvironmentVariableValue, DWORD &dwLen);

/**
 * @brief Set the value of an operating system environment variable
 *
 * @param strEnvironmentVariableName Name of the environment variable
 * @param strEnvironmentVariableValue Value to set for the environment variable
 * @param bReplace If true, replace the existing value; if false, do not replace if the variable already exists
 */
BASICSDK_API void os_system_set_os_environmentVariable(const char *strEnvironmentVariableName,
                                                       const char *strEnvironmentVariableValue, bool bReplace = true);

/**
 * @brief Invoke a system command
 *
 * @param strCommand Command to execute
 * @return bool True if the command is executed successfully, false otherwise
 */
BASICSDK_API bool os_system_invoke_command(const char *strCommand);

/**
 * @brief Close any pending exceptions
 */
BASICSDK_API void os_system_close_exception();

/**
 * @brief Check if the current process has root privileges
 *
 * @return bool True if the process has root privileges, false otherwise
 */
BASICSDK_API bool os_system_check_root_privileges();

/**
 * @brief Namespace for system-related functions and classes
 */
namespace OS {
class System {
public:
    /**
     * @brief Run an application
     *
     * @param strApplicationName Name of the application to run
     * @param strCommandLine Command line arguments for the application (optional)
     * @param bBlockParent If true, block the parent process until the application exits
     * @return bool True if the application is started successfully, false otherwise
     */
    static bool runApplication(const string &strApplicationName, const string &strCommandLine = "",
                               bool bBlockParent = false) {
        return os_system_run_application(strApplicationName.c_str(), strCommandLine.c_str(), bBlockParent);
    }

    /**
     * @brief Forcefully kill an application
     *
     * Note: This function will forcefully terminate the application, potentially causing data loss.
     * It is not recommended for use in normal circumstances.
     *
     * @param strApplicationName Name of the application to kill
     * @return bool True if the application is killed successfully, false otherwise
     */
    static bool killApplication(const string &strApplicationName) {
        return os_system_kill_application(strApplicationName.c_str());
    }

    /**
     * @brief Get the value of an operating system environment variable
     *
     * @param strEnvironmentVariableName Name of the environment variable
     * @param strEnvironmentVariableValue String to store the value of the environment variable
     * @return bool True if the environment variable is retrieved successfully, false otherwise
     */
    static bool getOSEnvironmentVariable(const string &strEnvironmentVariableName,
                                         string &strEnvironmentVariableValue) {
        DWORD dwLen = 0;
        if (os_system_get_os_environmentVariable(strEnvironmentVariableName.c_str(), nullptr, dwLen) == true &&
            dwLen > 0) {
            char *pValue = new char[dwLen];
            os_system_get_os_environmentVariable(strEnvironmentVariableName.c_str(), pValue, dwLen);
            strEnvironmentVariableValue = pValue;
            delete[] pValue;
            pValue = nullptr;
            return true;
        } else
            return false;
    }

    /**
     * @brief Set the value of an operating system environment variable
     *
     * @param strEnvironmentVariableName Name of the environment variable
     * @param strEnvironmentVariableValue Value to set for the environment variable
     * @param bReplace If true, replace the existing value; if false, do not replace if the variable already exists
     */
    static void setOSEnvironmentVariable(const string &strEnvironmentVariableName,
                                         const string &strEnvironmentVariableValue, bool bReplace = true) {
        return os_system_set_os_environmentVariable(strEnvironmentVariableName.c_str(),
                                                    strEnvironmentVariableValue.c_str(), bReplace);
    }

    /**
     * @brief Invoke a system command
     *
     * @param strSystemCommand Command to execute
     * @return bool True if the command is executed successfully, false otherwise
     */
    static bool invokeCommand(const string &strSystemCommand) {
        return os_system_invoke_command(strSystemCommand.c_str());
    }

    /**
     * @brief Close any pending exceptions
     */
    static void closeException() { return os_system_close_exception(); }

    /**
     * @brief Check if the current process has root privileges
     *
     * @return bool True if the process has root privileges, false otherwise
     */
    static bool checkRootPrivileges() { return os_system_check_root_privileges(); }
};
}  // namespace OS

#endif