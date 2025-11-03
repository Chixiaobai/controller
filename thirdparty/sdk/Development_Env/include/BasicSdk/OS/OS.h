/**
 * @file OS.h
 * @brief Provides a collection of utility classes and functions for operating system operations.
 *
 * This header file includes various classes and functions that facilitate common operating system tasks such as
 * thread management, clock operations, environment handling, folder and file operations, memory management,
 * UTF-8 encoding conversion, dynamic library loading, system information retrieval, and shared memory management.
 */

#ifndef __BASICSDK_OS_H__
#define __BASICSDK_OS_H__

#include "OS/OS_CLibrary.h"
#include "OS/OS_Thread.h"
#include "OS/OS_Clock.h"
#include "OS/OS_Environment.h"
#include "OS/OS_Folder.h"
#include "OS/OS_File.h"
#include "OS/OS_FileSystem.h"
#include "OS/OS_MemoryManager.h"
#include "OS/OS_UTF8Helper.h"
#include "OS/OS_DynamicLibraryLoader.h"
#include "OS/OS_System.h"
#include "OS/OS_ShareMemory.h"

/**
 * @brief Retrieves the default real-time mode of the system.
 *
 * This function returns the current default real-time mode setting of the system.
 *
 * @return The current default real-time mode.
 */
BASICSDK_API RT_MODE get_default_rt_mode();

/**
 * @brief Sets the default real-time mode of the system.
 *
 * This function sets the default real-time mode of the system to the specified mode.
 *
 * @param mode The real-time mode to be set as the default.
 * @return True if the operation is successful, false otherwise.
 */
BASICSDK_API bool set_default_rt_mode(RT_MODE mode);

#endif