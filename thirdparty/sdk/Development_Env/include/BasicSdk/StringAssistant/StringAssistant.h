/**
 * @file StringAssistant.h
 * @brief Provides utility functions for string manipulation.
 *
 * This header file includes a class `CStringAssistant` that offers various static methods for string operations,
 * such as conversion, comparison, trimming, formatting, splitting, and validation.
 */

#ifndef __BASICSDK_STRING_ASSISTANT_H__
#define __BASICSDK_STRING_ASSISTANT_H__

#include "BasicSdkDef.h"

/**
 * @brief Utility class for string manipulation.
 *
 * This class provides a set of static methods to perform common string operations, including conversion,
 * comparison, trimming, formatting, splitting, and validation.
 */
class CStringAssistant {
public:
    /**
     * @brief Converts an STL string to a C-style string.
     *
     * This method converts an STL string to a C-style string and stores it in the provided buffer.
     * If the buffer is null, it returns the required buffer size.
     *
     * @param strValue The source STL string.
     * @param pValue Pointer to the buffer where the C-style string will be stored.
     * @param dwLen Reference to a DWORD variable that will hold the length of the C-style string.
     * @return True if the conversion is successful, false if the buffer is too small.
     */
    static bool getBaseStringFromSTLString(const string &strValue, char *pValue, DWORD &dwLen) {
        size_t uValueLen = strValue.size();
        if (pValue == nullptr) {
            dwLen = (DWORD)uValueLen + 1;
            return true;
        } else {
            if (dwLen < (DWORD)uValueLen + 1) {
                return false;
            } else {
#ifdef _WIN32
                strcpy_s(pValue, dwLen, strValue.c_str());
#else
                strcpy(pValue, strValue.c_str());
#endif
                dwLen = (DWORD)uValueLen + 1;
                return true;
            }
        }
    }

    /**
     * @brief Replaces occurrences of a substring within an STL string.
     *
     * This method replaces all or the first occurrence of a substring with another substring in the source string.
     *
     * @param strSrcText The source STL string.
     * @param strOldText The substring to be replaced.
     * @param strNewText The substring to replace with.
     * @param bReplaceAll Flag indicating whether to replace all occurrences (default is true).
     * @return The modified STL string with the replacements made.
     */
    static string replaceSTLString(const string &strSrcText, const string &strOldText, const string &strNewText,
                                   bool bReplaceAll = true) {
        if (strOldText != strNewText) {
            string strDstText;
            string::size_type pos_begin = 0;
            string::size_type pos = strSrcText.find(strOldText);
            if (bReplaceAll == true) {
                while (pos != string::npos) {
                    strDstText.append(strSrcText.data() + pos_begin, pos - pos_begin);
                    strDstText += strNewText;
                    pos_begin = pos + strOldText.length();
                    pos = strSrcText.find(strOldText, pos_begin);
                }
            } else {
                if (pos != string::npos) {
                    strDstText.append(strSrcText.data() + pos_begin, pos - pos_begin);
                    strDstText += strNewText;
                    pos_begin = pos + strOldText.length();
                }
            }
            if (pos_begin < strSrcText.length()) {
                strDstText.append(strSrcText.begin() + pos_begin, strSrcText.end());
            }
            return strDstText;
        } else
            return strSrcText;
    }

    /**
     * @brief Compares two STL strings.
     *
     * This method compares two STL strings using the `strcmp` function.
     *
     * @param str1 The first STL string.
     * @param str2 The second STL string.
     * @return The result of the comparison: 0 if equal, <0 if str1 is less than str2, >0 if str1 is greater than str2.
     */
    static int compare(const string &str1, const string &str2) { return strcmp(str1.c_str(), str2.c_str()); }

    /**
     * @brief Compares two STL strings, ignoring case.
     *
     * This method compares two STL strings in a case-insensitive manner.
     *
     * @param str1 The first STL string.
     * @param str2 The second STL string.
     * @return The result of the comparison: 0 if equal, <0 if str1 is less than str2, >0 if str1 is greater than str2.
     */
    static int compareNoCase(const string &str1, const string &str2) {
        string upper_str1 = str1, upper_str2 = str2;
        transform(upper_str1.begin(), upper_str1.end(), upper_str1.begin(), ::toupper);
        transform(upper_str2.begin(), upper_str2.end(), upper_str2.begin(), ::toupper);
        return strcmp(upper_str1.c_str(), upper_str2.c_str());
    }

    /**
     * @brief Trims leading whitespace from an STL string.
     *
     * This method removes leading whitespace characters from the source string.
     *
     * @param strSource The source STL string.
     * @return The modified STL string with leading whitespace removed.
     */
    static string trimLeft(string &strSource) {
        const char *p = " \t\n\r\x0b";
        strSource.erase(0, strSource.find_first_not_of(p));
        return strSource;
    }

    /**
     * @brief Trims trailing whitespace from an STL string.
     *
     * This method removes trailing whitespace characters from the source string.
     *
     * @param strSource The source STL string.
     * @return The modified STL string with trailing whitespace removed.
     */
    static string trimRight(string &strSource) {
        const char *p = " \t\n\r\x0b";
        strSource.erase(strSource.find_last_not_of(p) + 1);
        return strSource;
    }

    /**
     * @brief Trims leading and trailing whitespace from an STL string.
     *
     * This method removes leading and trailing whitespace characters from the source string.
     *
     * @param strSource The source STL string.
     * @return The modified STL string with leading and trailing whitespace removed.
     */
    static string trim(string &strSource) {
        const char *p = " \t\n\r\x0b";
        strSource.erase(0, strSource.find_first_not_of(p));
        strSource.erase(strSource.find_last_not_of(p) + 1);
        return strSource;
    }

    /**
     * @brief Converts an STL string to uppercase.
     *
     * This method converts all characters in the source string to uppercase.
     *
     * @param strSource The source STL string.
     * @return The modified STL string with all characters in uppercase.
     */
    static string upper(string &strSource) {
        transform(strSource.begin(), strSource.end(), strSource.begin(), ::toupper);
        return strSource;
    }

    /**
     * @brief Converts an STL string to lowercase.
     *
     * This method converts all characters in the source string to lowercase.
     *
     * @param strSource The source STL string.
     * @return The modified STL string with all characters in lowercase.
     */
    static string lower(string &strSource) {
        transform(strSource.begin(), strSource.end(), strSource.begin(), ::tolower);
        return strSource;
    }

    /**
     * @brief Checks if an STL string starts with a specified substring.
     *
     * This method checks if the source string starts with the specified substring.
     *
     * @param strSource The source STL string.
     * @param strHead The substring to check for at the start.
     * @return True if the source string starts with the specified substring, false otherwise.
     */
    static bool startsWith(const string &strSource, const string &strHead) {
        size_t szSourceLen = strSource.size();
        size_t szHeadLen = strHead.size();
        if (szSourceLen < szHeadLen) {
            return false;
        } else {
            return strncmp(strSource.c_str(), strHead.c_str(), szHeadLen) == 0;
        }
    }

    /**
     * @brief Checks if an STL string ends with a specified substring.
     *
     * This method checks if the source string ends with the specified substring.
     *
     * @param strSource The source STL string.
     * @param strTail The substring to check for at the end.
     *0 @return True if the source string ends with the specified substring, false otherwise.
     */
    static bool endsWith(const string &strSource, const string &strTail) {
        size_t szSourceLen = strSource.size();
        size_t szTailLen = strTail.size();
        if (szSourceLen < szTailLen) {
            return false;
        } else {
            return strncmp(strSource.c_str() + szSourceLen - szTailLen, strTail.c_str(), szTailLen) == 0;
        }
    }

    /**
     * @brief Formats a string using a format string and variable arguments.
     *
     * This method formats a string using a format string and a variable number of arguments.
     *
     * @param strFormat The format string.
     * @param ... Variable arguments to be formatted into the string.
     * @return The formatted STL string.
     */
    static string format(const char *strFormat, ...) {
        string strFormatValue;
        va_list ap;
        va_start(ap, strFormat);
        strFormatValue = formatV(strFormat, ap);
        va_end(ap);
        return strFormatValue;
    }

    /**
     * @brief Formats a string using a format string and a va_list.
     *
     * This method formats a string using a format string and a va_list.
     *
     * @param strFormat The format string.
     * @param ap The va_list containing the arguments to be formatted into the string.
     * @return The formatted STL string.
     */
    static string formatV(const char *strFormat, va_list ap) {
        string strFormatValue;
        int iLen = OS::CLibrary::String::getFormattedLength(strFormat, ap);
        if (iLen > 0) {
            char *strBuffer = new char[iLen + 1];
            bzero(strBuffer, iLen + 1);
            vsnprintf(strBuffer, iLen, strFormat, ap);
            strFormatValue = strBuffer;
            delete[] strBuffer;
            strBuffer = nullptr;
        }
        return strFormatValue;
    }

    /**
     * @brief Splits an STL string into a list of substrings based on a delimiter.
     *
     * This method splits the source string into a list of substrings using the specified delimiter.
     *
     * @param strSource The source STL string.
     * @param strSeperator The delimiter string.
     * @return A list of substrings.
     */
    static list<string> split(const string &strSource, const string &strSeperator) {
        list<string> strTargetList;
        typedef string::size_type string_size;
        string_size i = 0;
        while (i != strSource.size()) {
            // Find the first character that is not a separator
            int iFoundNormal = 0;
            while (i != strSource.size() && iFoundNormal == 0) {
                iFoundNormal = 1;
                for (string_size x = 0; x < strSeperator.size(); x++) {
                    if (strSource[i] == strSeperator[x]) {
                        i++;
                        iFoundNormal = 0;
                        break;
                    }
                }
            }

            // Find the next separator and extract the substring
            int iFoundSeperator = 0;
            string_size j = i;
            while (j != strSource.size() && iFoundSeperator == 0) {
                for (string_size x = 0; x < strSeperator.size(); x++)
                    if (strSource[j] == strSeperator[x]) {
                        iFoundSeperator = 1;
                        break;
                    }
                if (iFoundSeperator == 0) j++;
            }
            if (i != j) {
                strTargetList.push_back(strSource.substr(i, j - i));
                i = j;
            }
        }
        return strTargetList;
    }

    /**
     * @brief Fixes the file path by normalizing the path separators.
     *
     * This method normalizes the path separators in the source string to the specified separator.
     *
     * @param strSource The source file path string.
     * @param path_split The path separator character (default is PATH_SPLIT_CHAR).
     * @return The fixed file path string.
     */
    static string fixFilePath(const string &strSource, char path_split = PATH_SPLIT_CHAR) {
        bool is_last_split = false;
        int intStringLen = (int)strSource.length(), new_index = 0;
        char *new_path = new char[intStringLen + 1];
        for (int i = 0; i < intStringLen; i++) {
            if (strSource[i] != '/' && strSource[i] != '\\') {
                new_path[new_index++] = strSource[i];
                is_last_split = false;
            } else {
                if (is_last_split == false) {
                    new_path[new_index++] = path_split;
                    is_last_split = true;
                }
            }
        }
        new_path[new_index] = '\0';
        string strNewPath = new_path;
        delete[] new_path;
        new_path = nullptr;
        return strNewPath;
    }

    /**
     * @brief Checks if a string is a valid identifier.
     *
     * This method checks if the source string is a valid identifier according to C/C++ rules.
     *
     * @param strSource The source string.
     * @return True if the string is a valid identifier, false otherwise.
     */
    static bool isIdentifier(const string &strSource) {
        if (strSource.empty() == true) {
            return false;
        }
        if ((strSource[0] >= 'a' && strSource[0] <= 'z') || (strSource[0] >= 'A' && strSource[0] <= 'Z') ||
            strSource[0] == '_') {
            size_t szLen = strSource.size();
            for (size_t i = 1; i < szLen; i++) {
                if ((strSource[i] >= 'a' && strSource[i] <= 'z') || (strSource[i] >= 'A' && strSource[i] <= 'Z') ||
                    (strSource[i] >= '0' && strSource[i] <= '9') || strSource[i] == '_') {
                    continue;
                } else {
                    return false;
                }
            }
            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief Checks if a string is a valid filename.
     *
     * This method checks if the source string is a valid filename by ensuring it does not contain invalid characters.
     *
     * @param strSource The source string.
     * @return True if the string is a valid filename, false otherwise.
     */
    static bool isFilename(const string &strSource) {
        char exclude_char[] = {'/', '\\', ':', '*', '?', '\'', '\"', '<', '>', '|'};
        size_t exclude_count = sizeof(exclude_char) / sizeof(exclude_char[0]);
        size_t szLen = strSource.size();
        for (size_t i = 0; i < szLen; i++) {
            for (size_t j = 0; j < exclude_count; j++) {
                if (strSource[i] == exclude_char[j]) {
                    return false;
                }
            }
        }
        return true;
    }
};

#endif