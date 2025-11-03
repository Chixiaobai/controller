/**
 * @file OSType.h
 * @brief Defines platform-independent types and macros for operating system operations.
 *
 * This header file includes various definitions and macros that provide a consistent interface
 * for different operating systems, such as Windows and Linux. It defines types for handles,
 * file operations, threading, and other system-related functionalities.
 */

#ifndef __BASICSDK_OS_TYPE_H__
#define __BASICSDK_OS_TYPE_H__

#ifdef _WIN32
/**
 * @brief Handle type for threads on Windows.
 */
#define OS_THREAD_HANDLE HANDLE

/**
 * @brief Handle type for modules on Windows.
 */
#define OS_MODULE_HANDLE HMODULE
#else
/**
 * @brief Handle type for threads on non-Windows systems.
 */
#define OS_THREAD_HANDLE pthread_t

/**
 * @brief Handle type for modules on non-Windows systems.
 */
#define OS_MODULE_HANDLE void *
#endif

#ifdef _WIN32
#include "time.h"
#ifndef __MINGW32__
/**
 * @brief Enumeration of clock identifiers on Windows.
 */
enum clockid_t { CLOCK_REALTIME = 0, CLOCK_MONOTONIC = 1 };
#endif
#ifdef WIN32_LEAN_AND_MEAN
#include "combaseapi.h"
#endif
#else
#include <cstdint>
/**
 * @brief 64-bit integer type.
 */
typedef int64_t __int64;

/**
 * @brief Defines interface as struct on non-Windows systems.
 */
#define interface struct
#endif

#ifdef _WIN32
/**
 * @brief Macro indicating support for C++11 on Windows.
 */
#define STD_CPLUSPLUS_11

/**
 * @brief Code page identifier for GB2312 on Windows.
 */
#define CP_GB2312 936

/**
 * @brief File mode for read, write, and execute permissions for the owner on Windows.
 */
#define S_IRWXU (_S_IREAD | _S_IWRITE | _S_IEXEC)

/**
 * @brief File mode for read, write, and execute permissions for the group on Windows.
 */
#define S_IRWXG S_IRWXU

/**
 * @brief File mode for read, write, and execute permissions for others on Windows.
 */
#define S_IRWXO S_IRWXU

/**
 * @brief Macro indicating file existence check on Windows.
 */
#define F_OK 0

/**
 * @brief Type for file mode on Windows.
 */
#define FILE_MODE_T int

/**
 * @brief Prefix for library files on Windows.
 */
#define OS_LIBRARY_PREFIX ""

/**
 * @brief Suffix for static library files on Windows.
 */
#define OS_STATIC_LIBRARY_SUFFIX "lib"

/**
 * @brief Suffix for import library files on Windows.
 */
#define OS_IMPORT_LIBRARY_SUFFIX "lib"

/**
 * @brief Suffix for dynamic library files on Windows.
 */
#define OS_DYNAMIC_LIBRARY_SUFFIX "dll"

/**
 * @brief String used to split environment variables on Windows.
 */
#define OS_ENV_SPLIT_STRING ";"

/**
 * @brief Character used to split environment variables on Windows.
 */
#define OS_ENV_SPLIT_CHAR ';'

/**
 * @brief String used to split paths on Windows.
 */
#define PATH_SPLIT_STRING "\\"

/**
 * @brief Character used to split paths on Windows.
 */
#define PATH_SPLIT_CHAR '\\'
#else
#if __cplusplus >= 201103L
/**
 * @brief Macro indicating support for C++11 on non-Windows systems.
 */
#define STD_CPLUSPLUS_11
#endif

/**
 * @brief Type for file mode on non-Windows systems.
 */
#define FILE_MODE_T mode_t

/**
 * @brief Prefix for library files on non-Windows systems.
 */
#define OS_LIBRARY_PREFIX "lib"

/**
 * @brief Suffix for static library files on non-Windows systems.
 */
#define OS_STATIC_LIBRARY_SUFFIX "a"

/**
 * @brief Suffix for import library files on non-Windows systems.
 */
#define OS_IMPORT_LIBRARY_SUFFIX "so"

/**
 * @brief Suffix for dynamic library files on non-Windows systems.
 */
#define OS_DYNAMIC_LIBRARY_SUFFIX "so"

/**
 * @brief String used to split environment variables on non-Windows systems.
 */
#define OS_ENV_SPLIT_STRING ":"

/**
 * @brief Character used to split environment variables on non-Windows systems.
 */
#define OS_ENV_SPLIT_CHAR ':'

/**
 * @brief String used to split paths on non-Windows systems.
 */
#define PATH_SPLIT_STRING "/"

/**
 * @brief Character used to split paths on non-Windows systems.
 */
#define PATH_SPLIT_CHAR '/'
#endif

#ifndef STD_CPLUSPLUS_11
/**
 * @brief Definition of nullptr for systems not supporting C++11.
 */
#define nullptr 0
#endif

/**
 * @brief Structure representing a BSTR unit.
 */
typedef struct {
    char character;
} BSTR_UNIT, *LPBSTRUNIT;  // BSTR字符串，前4个字节为长度

// 操作系统标准头文件
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#ifdef _WIN32
#include "windows.h"
#include "io.h"
#include "direct.h"
#include "Mmsystem.h"
#include "sys/utime.h"
#ifndef __MINGW32__
#pragma comment(lib, "winmm.lib")
#endif
/**
 * @brief Double precision floating point type.
 */
typedef double DOUBLE;

/**
 * @brief Boolean type for variant use.
 */
typedef bool TVARIANT_BOOL;
#else
#include "sys/mman.h"
#include "sys/wait.h"
#include "utime.h"
#include "errno.h"
#include "stdint.h"
#include "unistd.h"
#include "dlfcn.h"
#include "dirent.h"
/**
 * @brief Character type.
 */
typedef char CHAR;

/**
 * @brief Unsigned character type.
 */
typedef unsigned char UCHAR;

/**
 * @brief Short integer type.
 */
typedef short SHORT;

/**
 * @brief Unsigned short integer type.
 */
typedef unsigned short USHORT;

/**
 * @brief Integer type.
 */
typedef int INT;

/**
 * @brief Unsigned integer type.
 */
typedef unsigned int UINT;

/**
 * @brief Long integer type.
 */
typedef long LONG;

/**
 * @brief Unsigned long integer type.
 */
typedef unsigned long ULONG;

/**
 * @brief Long long integer type.
 */
typedef long long LONGLONG;

/**
 * @brief Unsigned long long integer type.
 */
typedef unsigned long long ULONGLONG;

/**
 * @brief Single precision floating point type.
 */
typedef float FLOAT;

/**
 * @brief Double precision floating point type.
 */
typedef double DOUBLE;

/**
 * @brief Boolean type for variant use.
 */
typedef bool TVARIANT_BOOL;

/**
 * @brief Unsigned character type.
 */
typedef unsigned char BYTE;

/**
 * @brief Unsigned short integer type.
 */
typedef unsigned short WORD;

/**
 * @brief Unsigned long integer type.
 */
typedef unsigned long DWORD;

/**
 * @brief Signed size type.
 */
typedef ssize_t SSIZE_T;

#if (defined(__LP64__))
/**
 * @brief Integer pointer type for 64-bit systems.
 */
typedef int64_t INT_PTR, *PINT_PTR;

/**
 * @brief Unsigned integer pointer type for 64-bit systems.
 */
typedef uint64_t UINT_PTR, *PUINT_PTR;

/**
 * @brief Long pointer type for 64-bit systems.
 */
typedef int64_t LONG_PTR, *PLONG_PTR;

/**
 * @brief Unsigned long pointer type for 64-bit systems.
 */
typedef uint64_t ULONG_PTR, *PULONG_PTR;
#else
/**
 * @brief Integer pointer type for 32-bit systems.
 */
typedef int INT_PTR, *PINT_PTR;

/**
 * @brief Unsigned integer pointer type for 32-bit systems.
 */
typedef unsigned int UINT_PTR, *PUINT_PTR;

/**
 * @brief Long pointer type for 32-bit systems.
 */
typedef long LONG_PTR, *PLONG_PTR;

/**
 * @brief Unsigned long pointer type for 32-bit systems.
 */
typedef unsigned long ULONG_PTR, *PULONG_PTR;
#endif

/**
 * @brief Pointer to character type.
 */
typedef char *LPCHAR;

/**
 * @brief Pointer to unsigned character type.
 */
typedef unsigned char *LPUCHAR;

/**
 * @brief Pointer to short integer type.
 */
typedef short *LPSHORT;

/**
 * @brief Pointer to unsigned short integer type.
 */
typedef unsigned short *LPUSHORT;

/**
 * @brief Pointer to integer type.
 */
typedef int *LPINT;

/**
 * @brief Pointer to unsigned integer type.
 */
typedef unsigned int *LPUINT;

/**
 * @brief Pointer to long integer type.
 */
typedef long *LPLONG;

/**
 * @brief Pointer to unsigned long integer type.
 */
typedef unsigned long *LPULONG;

/**
 * @brief Pointer to long long integer type.
 */
typedef long long *LPLONGLONG;

/**
 * @brief Pointer to unsigned long long integer type.
 */
typedef unsigned long long *LPULONGLONG;

/**
 * @brief Pointer to single precision floating point type.
 */
typedef float *LPFLOAT;

/**
 * @brief Pointer to double precision floating point type.
 */
typedef double *LPDOUBLE;

/**
 * @brief Pointer to boolean type for variant use.
 */
typedef bool *LPVARIANT_BOOL;

/**
 * @brief Pointer to unsigned character type.
 */
typedef unsigned char *LPBYTE;

/**
 * @brief Pointer to unsigned long integer type.
 */
typedef DWORD *LPDWORD;

/**
 * @brief Unsigned integer pointer type.
 */
typedef UINT_PTR WPARAM;

/**
 * @brief Long pointer type.
 */
typedef LONG_PTR LPARAM;

/**
 * @brief Long pointer type for result.
 */
typedef LONG_PTR LRESULT;

/**
 * @brief Unsigned long pointer type.
 */
typedef ULONG_PTR DWORD_PTR, *PDWORD_PTR;

/**
 * @brief Void pointer type.
 */
typedef void *LPVOID;

/**
 * @brief Void pointer type.
 */
typedef void *HANDLE;
#endif

#ifndef _XENOMAI_
/**
 * @brief Standard call macro for non-Xenomai systems.
 */
#define _STD_CALL(x) ::x

/**
 * @brief Real-time call macro for non-Xenomai systems.
 */
#define _RT_CALL(x) ::x
#else
#include "cobalt/wrappers.h"
/**
 * @brief Standard call macro for Xenomai systems.
 */
#define _STD_CALL(x) ::x

/**
 * @brief Real-time call macro for Xenomai systems.
 */
#define _RT_CALL(x) __RT(x)
#endif

/**
 * @brief Enumeration of real-time modes.
 */
enum RT_MODE { USE_DEFAULT = 0, USE_RT = 1, USE_STD = 2 };

/**
 * @brief Type for thread function.
 */
typedef LPVOID (*OS_THREAD_FUNCTION)(void *);

#ifdef _WIN32
/**
 * @brief Macro to zero out memory on Windows.
 */
#define bzero(pt, len) memset(pt, 0, len);
#endif

#ifndef _WIN32
/**
 * @brief Macro to create a 16-bit value from two 8-bit values.
 */
#define MAKEWORD(a, b) ((WORD)(((BYTE)(((DWORD_PTR)(a)) & 0xff)) | ((WORD)((BYTE)(((DWORD_PTR)(b)) & 0xff))) << 8))

/**
 * @brief Macro to create a 32-bit value from two 16-bit values.
 */
#define MAKELONG(a, b) \
    ((LONG)(((WORD)(((DWORD_PTR)(a)) & 0xffff)) | ((DWORD)((WORD)(((DWORD_PTR)(b)) & 0xffff))) << 16))

/**
 * @brief Macro to get the low 16 bits of a 32-bit value.
 */
#define LOWORD(l) ((WORD)(((DWORD_PTR)(l)) & 0xffff))

/**
 * @brief Macro to get the high 16 bits of a 32-bit value.
 */
#define HIWORD(l) ((WORD)((((DWORD_PTR)(l)) >> 16) & 0xffff))

/**
 * @brief Macro to get the low 8 bits of a 16-bit value.
 */
#define LOBYTE(w) ((BYTE)(((DWORD_PTR)(w)) & 0xff))

/**
 * @brief Macro to get the high 8 bits of a 16-bit value.
 */
#define HIBYTE(w) ((BYTE)((((DWORD_PTR)(w)) >> 8) & 0xff))
#endif

#ifndef _WIN32
/**
 * @brief Macro indicating an infinite timeout.
 */
#define INFINITE 0xFFFFFFFF

/**
 * @brief Macro indicating an invalid handle value.
 */
#define INVALID_HANDLE_VALUE (HANDLE)(-1)
#endif

#ifdef _WIN32
/**
 * @brief Handle type for devices on Windows.
 */
#define OS_DEVICE_HANDLE HANDLE

/**
 * @brief String indicating the operating system type on Windows.
 */
#define OS_TYPE_STRING "Windows"
#else
/**
 * @brief Handle type for devices on non-Windows systems.
 */
#define OS_DEVICE_HANDLE int

/**
 * @brief String indicating the operating system type on non-Windows systems.
 */
#define OS_TYPE_STRING "Linux"
#endif

#ifdef _WIN32
/**
 * @brief Handle type for files on Windows.
 */
typedef HANDLE OS_FILE_HANDLE;

/**
 * @brief Type for file time on Windows.
 */
typedef _utimbuf OS_FILE_TIME;
#else
/**
 * @brief Handle type for files on non-Windows systems.
 */
typedef int OS_FILE_HANDLE;

/**
 * @brief Type for file time on non-Windows systems.
 */
typedef utimbuf OS_FILE_TIME;
#endif

/**
 * @brief Macro indicating an invalid file descriptor value.
 */
#define INVALID_FD_VALUE (int)(-1)

/**
 * @brief Number of milliseconds per second.
 */
#define MSEC_PER_SEC 1000 /* The number of msecs per sec. */

/**
 * @brief Number of microseconds per second.
 */
#define USEC_PER_SEC 1000000 /* The number of usecs per sec. */

/**
 * @brief Number of nanoseconds per second.
 */
#define NSEC_PER_SEC 1000000000 /* The number of nsecs per sec. */

#ifndef UNREFERENCED_PARAMETER
/**
 * @brief Macro to indicate that a parameter is intentionally unused.
 */
#define UNREFERENCED_PARAMETER(x) \
    {                             \
        (x) = (x);                \
    }
#endif

#endif