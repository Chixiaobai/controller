#ifndef __BASICSDK_I_VARIANT_H__
#define __BASICSDK_I_VARIANT_H__

#include "Base/IBase.h"

/**
 * @brief Enumeration of variant types.
 */
#ifdef STD_CPLUSPLUS_11
enum TVARTYPE : unsigned short {
#else
enum TVARTYPE {
#endif
    TVT_NULL = 0,  /**< Null type. */
    TVT_I1 = 1,    /**< 8-bit signed integer. */
    TVT_UI1 = 2,   /**< 8-bit unsigned integer. */
    TVT_I2 = 3,    /**< 16-bit signed integer. */
    TVT_UI2 = 4,   /**< 16-bit unsigned integer. */
    TVT_INT = 5,   /**< 32-bit signed integer. */
    TVT_UINT = 6,  /**< 32-bit unsigned integer. */
    TVT_I4 = 7,    /**< 32-bit signed integer. */
    TVT_UI4 = 8,   /**< 32-bit unsigned integer. */
    TVT_I8 = 9,    /**< 64-bit signed integer. */
    TVT_UI8 = 10,  /**< 64-bit unsigned integer. */
    TVT_R4 = 11,   /**< 32-bit floating-point number. */
    TVT_R8 = 12,   /**< 64-bit floating-point number. */
    TVT_BSTR = 13, /**< Binary string. */
    TVT_BOOL = 14, /**< Boolean. */
};

/**
 * @brief Interface for reading variant values.
 */
interface IVariantRead : public IBase {
    /**
     * @brief Retrieves the type of the variant.
     * @param pType Buffer to store the type.
     * @param dwLen Length of the buffer.
     * @return True if successful, false otherwise.
     */
    virtual bool getType(char *pType, DWORD &dwLen) = 0;

    /**
     * @brief Retrieves the value as an 8-bit unsigned integer.
     * @param byteValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(unsigned char &byteValue) = 0;

    /**
     * @brief Retrieves the value as an 8-bit signed integer.
     * @param cValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(char &cValue) = 0;

    /**
     * @brief Retrieves the value as a 16-bit unsigned integer.
     * @param uiValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(unsigned short &uiValue) = 0;

    /**
     * @brief Retrieves the value as a 16-bit signed integer.
     * @param iValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(short &iValue) = 0;

    /**
     * @brief Retrieves the value as a 32-bit unsigned integer.
     * @param uintValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(unsigned int &uintValue) = 0;

    /**
     * @brief Retrieves the value as a 32-bit signed integer.
     * @param intValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(int &intValue) = 0;

    /**
     * @brief Retrieves the value as a 32-bit unsigned long.
     * @param ulValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(unsigned long &ulValue) = 0;

    /**
     * @brief Retrieves the value as a 32-bit signed long.
     * @param lValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(long &lValue) = 0;

    /**
     * @brief Retrieves the value as a 64-bit unsigned long long.
     * @param ullValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(unsigned long long &ullValue) = 0;

    /**
     * @brief Retrieves the value as a 64-bit signed long long.
     * @param llValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(long long &llValue) = 0;

    /**
     * @brief Retrieves the value as a 32-bit floating-point number.
     * @param fltValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(float &fltValue) = 0;

    /**
     * @brief Retrieves the value as a 64-bit floating-point number.
     * @param dblValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(double &dblValue) = 0;

    /**
     * @brief Retrieves the value as a string.
     * @param pValue Buffer to store the string.
     * @param dwLen Length of the buffer.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(char *pValue, DWORD &dwLen) = 0;

    /**
     * @brief Retrieves the value as a boolean.
     * @param boolValue Variable to store the value.
     * @return True if successful, false otherwise.
     */
    virtual bool getValue(bool &boolValue) = 0;
};

/**
 * @brief Interface for writing variant values.
 */
interface IVariantWrite : public IBase {
    /**
     * @brief Sets the type of the variant.
     * @param pType The type to set.
     * @return True if successful, false otherwise.
     */
    virtual bool setType(const char *pType) = 0;

    /**
     * @brief Sets the value as an 8-bit unsigned integer.
     * @param byteValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(unsigned char byteValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as an 8-bit signed integer.
     * @param cValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(char cValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 16-bit unsigned integer.
     * @param uiValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(unsigned short uiValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 16-bit signed integer.
     * @param iValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(short iValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 32-bit unsigned integer.
     * @param uintValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(unsigned int uintValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 32-bit signed integer.
     * @param intValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(int intValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 32-bit unsigned long.
     * @param ulValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(unsigned long ulValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 32-bit signed long.
     * @param lValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(long lValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 64-bit unsigned long long.
     * @param ullValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(unsigned long long ullValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 64-bit signed long long.
     * @param llValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(long long llValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 32-bit floating-point number.
     * @param fltValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(float fltValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a 64-bit floating-point number.
     * @param dblValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(double dblValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a string.
     * @param pValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(const char *pValue, bool bIncludeType) = 0;

    /**
     * @brief Sets the value as a boolean.
     * @param boolValue The value to set.
     * @param bIncludeType Whether to include the type.
     * @return True if successful, false otherwise.
     */
    virtual bool setValue(bool boolValue, bool bIncludeType) = 0;
};

/**
 * @brief Interface for managing variants.
 */
interface IVariant : public IBase {
    /**
     * @brief Retrieves the type of the variant.
     * @param pType Buffer to store the type.
     * @param dwLen Length of the buffer.
     * @return True if successful, false otherwise.
     */
    virtual bool getType(char *pType, DWORD &dwLen) = 0;

    /**
     * @brief Sets the type of the variant.
     * @param pType The type to set.
     * @return True if successful, false otherwise.
     */
    virtual bool setType(const char *pType) = 0;

    /**
     * @brief Retrieves the IVariantRead interface.
     * @return A pointer to the IVariantRead interface.
     */
    virtual IVariantRead *getVariantRead() = 0;

    /**
     * @brief Retrieves the IVariantWrite interface.
     * @return A pointer to the IVariantWrite interface.
     */
    virtual IVariantWrite *getVariantWrite() = 0;

    /**
     * @brief Creates a clone of the variant.
     * @return A pointer to the cloned variant.
     */
    virtual IVariant *clone() = 0;
};

/**
 * @brief Helper class for variant operations.
 */
class CVariantHelper {
public:
    /**
     * @brief Retrieves the value of a variant as a string.
     * @param pVariant The IVariantRead interface.
     * @return The value as a string.
     */
    static string getValue(IVariantRead *pVariant) {
        string strValue;
        if (pVariant != nullptr) {
            DWORD dwLen = 0;
            if (pVariant->getValue(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pVariant->getValue(pValue, dwLen);
                strValue = pValue;
                delete[] pValue;
                pValue = nullptr;
            }
        }
        return strValue;
    }

    /**
     * @brief Retrieves the type of a variant as a string.
     * @param pVariant The IVariantRead interface.
     * @return The type as a string.
     */
    static string getType(IVariantRead *pVariant) {
        string strType;
        if (pVariant != nullptr) {
            DWORD dwLen = 0;
            if (pVariant->getType(nullptr, dwLen) == true && dwLen > 0) {
                char *pType = new char[dwLen];
                pVariant->getType(pType, dwLen);
                strType = pType;
                delete[] pType;
                pType = nullptr;
            }
        }
        return strType;
    }
};

/**
 * @brief Macro to declare a variant exporter.
 */
#define DECLARE_VARIANT_EXPORTER() IVariantRead *pVariantRead = NULL;

/**
 * @brief Macro to export a numerical variant.
 * @param var The IVariant object.
 * @param value The variable to store the value.
 */
#define EXPORT_NUMERICAL_VARIANT(var, value) \
    pVariantRead = var->getVariantRead();    \
    if (pVariantRead != NULL) {              \
        pVariantRead->getValue(value);       \
        pVariantRead->releaseRef();          \
    }

/**
 * @brief Macro to export a string variant.
 * @param var The IVariant object.
 * @param value The variable to store the value.
 */
#define EXPORT_STRING_VARIANT(var, value)               \
    pVariantRead = var->getVariantRead();               \
    if (pVariantRead != NULL) {                         \
        value = CVariantHelper::getValue(pVariantRead); \
        pVariantRead->releaseRef();                     \
    }

/**
 * @brief Macro to declare a variant importer.
 */
#define DECLARE_VARIANT_IMPORTER() IVariantWrite *pVariantWrite = NULL;

/**
 * @brief Macro to import a numerical variant with type inclusion.
 * @param var The IVariant object.
 * @param value The variable to set the value.
 * @param include_type Whether to include the type.
 */
#define IMPORT_NUMERICAL_VARIANT_EX(var, value, include_type) \
    pVariantWrite = var->getVariantWrite();                   \
    if (pVariantWrite != NULL) {                              \
        pVariantWrite->setValue(value, include_type);         \
        pVariantWrite->releaseRef();                          \
    }

/**
 * @brief Macro to import a string variant with type inclusion.
 * @param var The IVariant object.
 * @param value The variable to set the value.
 * @param include_type Whether to include the type.
 */
#define IMPORT_STRING_VARIANT_EX(var, value, include_type)    \
    pVariantWrite = var->getVariantWrite();                   \
    if (pVariantWrite != NULL) {                              \
        pVariantWrite->setValue(value.c_str(), include_type); \
        pVariantWrite->releaseRef();                          \
    }

/**
 * @brief Macro to import a numerical variant.
 * @param var The IVariant object.
 * @param value The variable to set the value.
 */
#define IMPORT_NUMERICAL_VARIANT(var, value) IMPORT_NUMERICAL_VARIANT_EX(var, value, true)

/**
 * @brief Macro to import a string variant.
 * @param var The IVariant object.
 * @param value The variable to set the value.
 */
#define IMPORT_STRING_VARIANT(var, value) IMPORT_STRING_VARIANT_EX(var, value, true)

#endif