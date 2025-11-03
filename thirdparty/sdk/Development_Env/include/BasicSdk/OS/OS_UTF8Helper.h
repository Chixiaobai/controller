/**
 * @file OS_UTF8Helper.h
 * @brief Provides utility functions and a class for converting between UTF-8 and OEM character encodings.
 *
 * This header file includes functions and a class that facilitate the conversion of strings between UTF-8 and OEM
 * encodings. The functions are designed to handle the conversion process efficiently and safely.
 */

#ifndef __BASICSDK_OS_UTF8HELPER_H__
#define __BASICSDK_OS_UTF8HELPER_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Converts a UTF-8 encoded string to an OEM encoded string.
 *
 * This function takes a UTF-8 encoded string and converts it to an OEM encoded string.
 * The length of the resulting OEM string is returned through the dwLen parameter.
 *
 * @param strUTF8 Pointer to the UTF-8 encoded string to be converted.
 * @param strOEM Pointer to the buffer where the converted OEM string will be stored.
 * @param dwLen Reference to a DWORD variable that will hold the length of the converted OEM string.
 * @return True if the conversion is successful, false otherwise.
 */
BASICSDK_API bool os_utf8helper_oem_from_utf8(const char *strUTF8, char *strOEM, DWORD &dwLen);

/**
 * @brief Converts an OEM encoded string to a UTF-8 encoded string.
 *
 * This function takes an OEM encoded string and converts it to a UTF-8 encoded string.
 * The length of the resulting UTF-8 string is returned through the dwLen parameter.
 *
 * @param strOEM Pointer to the OEM encoded string to be converted.
 * @param strUTF8 Pointer to the buffer where the converted UTF-8 string will be stored.
 * @param dwLen Reference to a DWORD variable that will hold the length of the converted UTF-8 string.
 * @return True if the conversion is successful, false otherwise.
 */
BASICSDK_API bool os_utf8helper_oem_to_utf8(const char *strOEM, char *strUTF8, DWORD &dwLen);

namespace OS {
/**
 * @class UTF8Helper
 * @brief A utility class for converting between UTF-8 and OEM character encodings.
 *
 * This class provides static methods for converting strings between UTF-8 and OEM encodings.
 * It uses the underlying functions os_utf8helper_oem_from_utf8 and os_utf8helper_oem_to_utf8 for the conversion
 * process.
 */
class UTF8Helper {
public:
    /**
     * @brief Converts a UTF-8 encoded string to an OEM encoded string.
     *
     * This method takes a UTF-8 encoded string and converts it to an OEM encoded string.
     * It uses the os_utf8helper_oem_from_utf8 function internally.
     *
     * @param strUTF8 Reference to the UTF-8 encoded string to be converted.
     * @return The converted OEM encoded string.
     */
    static string oemFromUTF8(const string &strUTF8) { return oemFromUTF8(strUTF8.c_str()); }

    /**
     * @brief Converts a UTF-8 encoded string to an OEM encoded string.
     *
     * This method takes a UTF-8 encoded string and converts it to an OEM encoded string.
     * It uses the os_utf8helper_oem_from_utf8 function internally.
     *
     * @param strUTF8 Pointer to the UTF-8 encoded string to be converted.
     * @return The converted OEM encoded string.
     */
    static string oemFromUTF8(const char *strUTF8) {
        string strOEM;
        DWORD dwLen = 0;
        if (os_utf8helper_oem_from_utf8(strUTF8, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            os_utf8helper_oem_from_utf8(strUTF8, pValue, dwLen);
            strOEM = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strOEM;
    }

    /**
     * @brief Converts an OEM encoded string to a UTF-8 encoded string.
     *
     * This method takes an OEM encoded string and converts it to a UTF-8 encoded string.
     * It uses the os_utf8helper_oem_to_utf8 function internally.
     *
     * @param strOEM Reference to the OEM encoded string to be converted.
     * @return The converted UTF-8 encoded string.
     */
    static string oemToUTF8(const string &strOEM) { return oemToUTF8(strOEM.c_str()); }

    /**
     * @brief Converts an OEM encoded string to a UTF-8 encoded string.
     *
     * This method takes an OEM encoded string and converts it to a UTF-8 encoded string.
     * It uses the os_utf8helper_oem_to_utf8 function internally.
     *
     * @param strOEM Pointer to the OEM encoded string to be converted.
     * @return The converted UTF-8 encoded string.
     */
    static string oemToUTF8(const char *strOEM) {
        string strUTF8;
        DWORD dwLen = 0;
        if (os_utf8helper_oem_to_utf8(strOEM, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            os_utf8helper_oem_to_utf8(strOEM, pValue, dwLen);
            strUTF8 = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strUTF8;
    }
};
}  // namespace OS

#endif