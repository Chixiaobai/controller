/**
 * @file INameEnum.h
 * @brief Header file for name enumeration management.
 *
 * This file defines an interface and a helper class for enumerating names.
 */

#ifndef __TOOLSDK_I_NAME_ENUM_H__
#define __TOOLSDK_I_NAME_ENUM_H__

#include "Base/IBase.h"

/**
 * @interface INameEnum
 * @brief Interface for name enumeration.
 *
 * This interface defines methods for enumerating names.
 */
interface INameEnum : public IBase {
    /**
     * @brief Retrieves the current name.
     *
     * Retrieves the current name in the enumeration and stores it in the provided buffer.
     *
     * @param pName Pointer to the buffer to store the current name.
     * @param ulSize Reference to the length of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool currentName(char *pName, unsigned long &ulSize) = 0;

    /**
     * @brief Moves to the next name in the enumeration.
     *
     * Moves to the next name in the enumeration.
     *
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool nextName() = 0;

    /**
     * @brief Resets the enumeration iterator.
     *
     * Resets the iterator to the beginning of the enumeration.
     */
    virtual void resetIt() = 0;
};

/**
 * @class CNameEnumHelper
 * @brief Helper class for name enumeration.
 *
 * This class provides static methods to assist with name enumeration.
 */
class CNameEnumHelper {
public:
    /**
     * @brief Retrieves the current name.
     *
     * Retrieves the current name in the enumeration and stores it in the provided string.
     *
     * @param pNameEnum Pointer to the name enumeration.
     * @param strName Reference to the string to store the current name.
     * @return true if the operation was successful, false otherwise.
     */
    static bool currentName(INameEnum *pNameEnum, string &strName) {
        if (pNameEnum != nullptr) {
            DWORD dwLen = 0;
            if (pNameEnum->currentName(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pNameEnum->currentName(pValue, dwLen);
                strName = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
};

#endif  // __TOOLSDK_I_NAME_ENUM_H__