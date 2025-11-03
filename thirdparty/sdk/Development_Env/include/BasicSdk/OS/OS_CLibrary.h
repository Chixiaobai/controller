#ifndef __BASICSDK_OS_CLIBRARY_H__
#define __BASICSDK_OS_CLIBRARY_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Open a file
 *
 * @param _FileName Name of the file to open
 * @param _OpenFlag Flags specifying the mode in which the file is opened
 * @param _PMode Permission mode
 * @return int File handle on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_open(const char *_FileName, int _OpenFlag, int _PMode);

/**
 * @brief Close a file
 *
 * @param _FileHandle File handle to close
 * @return int 0 on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_close(int _FileHandle);

/**
 * @brief Check file access permissions
 *
 * @param _FileName Name of the file to check
 * @param _AccessMode Access mode to check
 * @return int 0 if the file has the specified access, or -1 if it does not
 */
BASICSDK_API int os_clibrary_file_access(const char *_FileName, int _AccessMode);

/**
 * @brief Set the file mode creation mask
 *
 * @param _Mode New file mode creation mask
 * @return int Previous file mode creation mask
 */
BASICSDK_API int os_clibrary_file_umask(int _Mode);

/**
 * @brief Move the file read/write position
 *
 * @param _FileHandle File handle
 * @param _Offset Offset to move the file pointer
 * @param _Origin Origin point for the offset (SEEK_SET, SEEK_CUR, SEEK_END)
 * @return long New file position on success, or -1 on failure
 */
BASICSDK_API long os_clibrary_file_seek(int _FileHandle, long _Offset, int _Origin);

/**
 * @brief Read data from a file
 *
 * @param _FileHandle File handle
 * @param _DstBuf Buffer to store the read data
 * @param _MaxCharCount Maximum number of characters to read
 * @return int Number of characters read on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_read(int _FileHandle, void *_DstBuf, unsigned int _MaxCharCount);

/**
 * @brief Write data to a file
 *
 * @param _FileHandle File handle
 * @param _Buf Buffer containing data to write
 * @param _MaxCharCount Maximum number of characters to write
 * @return int Number of characters written on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_write(int _FileHandle, void const *_Buf, unsigned int _MaxCharCount);

/**
 * @brief Delete a file
 *
 * @param _FileName Name of the file to delete
 * @return int 0 on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_unlink(const char *_FileName);

/**
 * @brief Create a directory
 *
 * @param _Path Path of the directory to create
 * @param iFlag Flags specifying the mode of the directory
 * @return int 0 on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_mkdir(const char *_Path, FILE_MODE_T iFlag);

/**
 * @brief Change the current working directory
 *
 * @param path New current working directory
 * @return int 0 on success, or -1 on failure
 */
BASICSDK_API int os_clibrary_file_chdir(const char *path);

/**
 * @brief Get the current working directory
 *
 * @param buf Buffer to store the current working directory
 * @param size Size of the buffer
 * @return char* Pointer to the buffer containing the current working directory on success, or NULL on failure
 */
BASICSDK_API char *os_clibrary_file_getcwd(char *buf, int size);

/**
 * @brief Check if a string contains only ASCII characters
 *
 * @param strVarName1 String to check
 * @return bool True if the string contains only ASCII characters, false otherwise
 */
BASICSDK_API bool os_clibrary_string_is_ascii(const char *strVarName1);

/**
 * @brief Get the formatted length of a string
 *
 * @param strFormat Format string
 * @param ap Variable argument list
 * @return int Length of the formatted string including the null terminator
 */
BASICSDK_API int os_clibrary_string_get_formatted_length(const char *strFormat, va_list ap);

/**
 * @brief Compare two strings intelligently
 *
 * Letters are compared by ASCII value, numbers by their numeric value.
 *
 * @param strVarName1 First string to compare
 * @param strVarName2 Second string to compare
 * @return bool True if the first string is greater than the second string, false otherwise
 */
BASICSDK_API bool os_clibrary_string_intelligent_greater(const char *strVarName1, const char *strVarName2);

/**
 * @brief Compare two strings intelligently
 *
 * Letters are compared by ASCII value, numbers by their numeric value.
 *
 * @param strVarName1 First string to compare
 * @param strVarName2 Second string to compare
 * @return bool True if the first string is less than the second string, false otherwise
 */
BASICSDK_API bool os_clibrary_string_intelligent_less(const char *strVarName1, const char *strVarName2);

/**
 * @brief Namespace for OS-related library functions and classes
 */
namespace OS {
class CLibrary {
public:
    /**
     * @brief Class for file operations
     */
    class File {
    public:
        /**
         * @brief Null file handle constant
         */
        static constexpr int hFileNull = -1;

    public:
        /**
         * @brief Open a file
         *
         * @param _FileName Name of the file to open
         * @param _OpenFlag Flags specifying the mode in which the file is opened
         * @param _PMode Permission mode
         * @return int File handle on success, or -1 on failure
         */
        static int open(const char *_FileName, int _OpenFlag, int _PMode) {
            return os_clibrary_file_open(_FileName, _OpenFlag, _PMode);
        }

        /**
         * @brief Close a file
         *
         * @param _FileHandle File handle to close
         * @return int 0 on success, or -1 on failure
         */
        static int close(int _FileHandle) { return os_clibrary_file_close(_FileHandle); }

        /**
         * @brief Check file access permissions
         *
         * @param _FileName Name of the file to check
         * @param _AccessMode Access mode to check
         * @return int 0 if the file has the specified access, or -1 if it does not
         */
        static int access(const char *_FileName, int _AccessMode) {
            return os_clibrary_file_access(_FileName, _AccessMode);
        }

        /**
         * @brief Set the file mode creation mask
         *
         * @param _Mode New file mode creation mask
         * @return int Previous file mode creation mask
         */
        static int umask(int _Mode) { return os_clibrary_file_umask(_Mode); }

        /**
         * @brief Move the file read/write position
         *
         * @param _FileHandle File handle
         * @param _Offset Offset to move the file pointer
         * @param _Origin Origin point for the offset (SEEK_SET, SEEK_CUR, SEEK_END)
         * @return long New file position on success, or -1 on failure
         */
        static long seek(int _FileHandle, long _Offset, int _Origin) {
            return os_clibrary_file_seek(_FileHandle, _Offset, _Origin);
        }

        /**
         * @brief Read data from a file
         *
         * @param _FileHandle File handle
         * @param _DstBuf Buffer to store the read data
         * @param _MaxCharCount Maximum number of characters to read
         * @return int Number of characters read on success, or -1 on failure
         */
        static int read(int _FileHandle, void *_DstBuf, unsigned int _MaxCharCount) {
            return os_clibrary_file_read(_FileHandle, _DstBuf, _MaxCharCount);
        }

        /**
         * @brief Write data to a file
         *
         * @param _FileHandle File handle
         * @param _Buf Buffer containing data to write
         * @param _MaxCharCount Maximum number of characters to write
         * @return int Number of characters written on success, or -1 on failure
         */
        static int write(int _FileHandle, void const *_Buf, unsigned int _MaxCharCount) {
            return os_clibrary_file_write(_FileHandle, _Buf, _MaxCharCount);
        }

        /**
         * @brief Delete a file
         *
         * @param _FileName Name of the file to delete
         * @return int 0 on success, or -1 on failure
         */
        static int unlink(const char *_FileName) { return os_clibrary_file_unlink(_FileName); }

        /**
         * @brief Create a directory
         *
         * @param _Path Path of the directory to create
         * @param iFlag Flags specifying the mode of the directory
         * @return int 0 on success, or -1 on failure
         */
        static int mkdir(const char *_Path, FILE_MODE_T iFlag) { return os_clibrary_file_mkdir(_Path, iFlag); }

        /**
         * @brief Change the current working directory
         *
         * @param _Path New current working directory
         * @return int 0 on success, or -1 on failure
         */
        static int chdir(const char *_Path) { return os_clibrary_file_chdir(_Path); }

        /**
         * @brief Get the current working directory
         *
         * @param _Buf Buffer to store the current working directory
         * @param _BufSize Size of the buffer
         * @return char* Pointer to the buffer containing the current working directory on success, or NULL on failure
         */
        static char *getcwd(char *_Buf, int _BufSize) { return os_clibrary_file_getcwd(_Buf, _BufSize); }
    };

    /**
     * @brief Class for string operations
     */
    class String {
    public:
        /**
         * @brief Check if a string contains only ASCII characters
         *
         * @param strVarName1 String to check
         * @return bool True if the string contains only ASCII characters, false otherwise
         */
        static bool isAscii(const char *strVarName1) { return os_clibrary_string_is_ascii(strVarName1); }

        /**
         * @brief Get the formatted length of a string
         *
         * @param strFormat Format string
         * @param ap Variable argument list
         * @return int Length of the formatted string including the null terminator
         */
        static int getFormattedLength(const char *strFormat, va_list ap) {
            return os_clibrary_string_get_formatted_length(strFormat, ap);
        }

        /**
         * @brief Compare two strings intelligently
         *
         * Letters are compared by ASCII value, numbers by their numeric value.
         *
         * @param strVarName1 First string to compare
         * @param strVarName2 Second string to compare
         * @return bool True if the first string is greater than the second string, false otherwise
         */
        static bool intelligentGreater(const char *strVarName1, const char *strVarName2) {
            return os_clibrary_string_intelligent_greater(strVarName1, strVarName2);
        }

        /**
         * @brief Compare two strings intelligently
         *
         * Letters are compared by ASCII value, numbers by their numeric value.
         *
         * @param strVarName1 First string to compare
         * @param strVarName2 Second string to compare
         * @return bool True if the first string is less than the second string, false otherwise
         */
        static bool intelligentLess(const char *strVarName1, const char *strVarName2) {
            return os_clibrary_string_intelligent_less(strVarName1, strVarName2);
        }
    };
};
}  // namespace OS

/**
 * @brief User-defined comparison function type
 *
 * @param strVarName1 First string to compare
 * @param strVarName2 Second string to compare
 * @return bool True if the first string is considered less than the second string, false otherwise
 */
typedef bool (*USER_COMPARE_FUNC)(const char *strVarName1, const char *strVarName2);

#endif