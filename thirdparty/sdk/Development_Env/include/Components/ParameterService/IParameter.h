#pragma once

#include "UserManager/IAuthorized.h"
#include "DataType/Variant.h"

/**
 * @interface IParameter
 * @brief Interface representing a configurable parameter with authorization control.
 *
 * This interface provides methods to set and retrieve parameter properties such as name, type,
 * value, and default value. It also supports modification tracking and requires user
 * authorization for most operations.
 */
interface IParameter : public IBase {
    /**
     * @brief Sets the name of the parameter.
     *
     * @param strParameterName The new name of the parameter.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the name was successfully set; false otherwise.
     */
    virtual bool setParameterName(const char *strParameterName, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Sets the data type of the parameter.
     *
     * @param type The TVARTYPE indicating the new data type.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the type was successfully set; false otherwise.
     */
    virtual bool setParameterType(TVARTYPE type, IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the name of the parameter.
     *
     * @param strName Buffer to receive the parameter name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if successful; false otherwise.
     */
    virtual bool getName(char *strName, unsigned long &ulSize, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Retrieves the current value of the parameter.
     *
     * @param pVariant Reference to IVariant pointer that will receive the value.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if successful; false otherwise.
     */
    virtual bool getValue(IVariant * &pVariant, IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the default value of the parameter.
     *
     * @param pVariant Reference to IVariant pointer that will receive the default value.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if successful; false otherwise.
     */
    virtual bool getDefaultValue(IVariant * &pVariant, IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Retrieves the data type of the parameter as a string.
     *
     * @param strType Buffer to receive the type string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if successful; false otherwise.
     */
    virtual bool getType(char *strType, unsigned long &ulSize, IUserAuthorized *pUserAuthorized) = 0;

    /**
     * @brief Resets the parameter value to its default.
     *
     * @param pUserAuthorized Pointer to an authorized user object.
     * @return true if the reset succeeded; false otherwise.
     */
    virtual bool resetValue(IUserAuthorized * pUserAuthorized) = 0;

    /**
     * @brief Marks the parameter as modified or unmodified.
     *
     * @param bModified Boolean flag indicating whether the parameter is modified.
     */
    virtual void setModified(bool bModified) = 0;

    /**
     * @brief Checks whether the parameter has been modified.
     *
     * @return true if the parameter is marked as modified; false otherwise.
     */
    virtual bool getModified() = 0;
};

class CParameterHelper {
public:
    static string getName(IParameter *pParameter, IUserAuthorized *pUserAuthorized) {
        string strName;
        if (pParameter != nullptr) {
            unsigned long ulSize = 0;
            if (pParameter->getName(nullptr, ulSize, pUserAuthorized) == true && ulSize > 0) {
                char *strNameA = new char[ulSize + 1];
                bzero(strNameA, (ulSize + 1) * sizeof(char));
                pParameter->getName(strNameA, ulSize, pUserAuthorized);
                strName = strNameA;
                delete[] strNameA;
                strNameA = nullptr;
            }
        }
        return strName;
    }
    static string GetType(IParameter *pParameter, IUserAuthorized *pUserAuthorized) {
        string strType;
        if (pParameter != nullptr) {
            unsigned long ulSize = 0;
            if (pParameter->getType(nullptr, ulSize, pUserAuthorized) == true && ulSize > 0) {
                char *strTypeA = new char[ulSize + 1];
                bzero(strTypeA, (ulSize + 1) * sizeof(char));
                pParameter->getType(strTypeA, ulSize, pUserAuthorized);
                strType = strTypeA;
                delete[] strTypeA;
                strTypeA = nullptr;
            }
        }
        return strType;
    }
};

ENABLE_RECOGNIZE_MACRO(IParameter)
