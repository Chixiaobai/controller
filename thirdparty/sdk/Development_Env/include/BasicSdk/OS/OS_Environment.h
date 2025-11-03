#ifndef __BASICSDK_OS_ENVIRONMENT_H__
#define __BASICSDK_OS_ENVIRONMENT_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Get the application path
 *
 * This function retrieves the path of the application executable.
 *
 * @param __buf Buffer to store the application path
 * @param __len Length of the buffer
 * @return SSIZE_T Number of characters written to the buffer, or -1 on failure
 */
BASICSDK_API SSIZE_T os_environment_get_application_path(char *__restrict __buf, size_t __len);

/**
 * @brief Namespace for environment-related functions and classes
 */
namespace OS {
class Environment {
public:
    /**
     * @brief Get the application path
     *
     * This function retrieves the path of the application executable.
     *
     * @param __buf Buffer to store the application path
     * @param __len Length of the buffer
     * @return SSIZE_T Number of characters written to the buffer, or -1 on failure
     */
    static SSIZE_T getApplicationPath(char *__restrict __buf, size_t __len) {
        return os_environment_get_application_path(__buf, __len);
    }
};
}  // namespace OS

#endif