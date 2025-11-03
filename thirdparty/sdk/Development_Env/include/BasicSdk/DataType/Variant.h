/**
 * @file Variant.h
 * @brief Header file for the TVARIANT structure and related functions.
 */

#ifndef __BASICSDK_VARIANT_H__
#define __BASICSDK_VARIANT_H__

#include "BasicSdkDef.h"
#include "BStr.h"
#include "IVariant.h"

/**
 * @brief Structure representing a variant data type.
 *
 * The TVARIANT structure is used to hold different types of data values, identified by the TVARTYPE.
 */
typedef struct tagTVARIANT {
    TVARTYPE vt;  ///< Type of the variant data.
    union {
        CHAR cVal;              ///< CHAR value.
        BYTE bVal;              ///< BYTE value.
        SHORT iVal;             ///< SHORT value.
        USHORT uiVal;           ///< USHORT value.
        INT intVal;             ///< INT value.
        UINT uintVal;           ///< UINT value.
        LONG lVal;              ///< LONG value.
        ULONG ulVal;            ///< ULONG value.
        LONGLONG llVal;         ///< LONGLONG value.
        ULONGLONG ullVal;       ///< ULONGLONG value.
        FLOAT fltVal;           ///< FLOAT value.
        DOUBLE dblVal;          ///< DOUBLE value.
        LPBSTRUNIT bstrVal;     ///< LPBSTRUNIT value.
        TVARIANT_BOOL boolVal;  ///< TVARIANT_BOOL value.
    };
} TVARIANT, *LPTVARIANT;

/**
 * @brief Constant pointer to a TVARIANT structure.
 */
typedef const LPTVARIANT LPCTVARIANT;

/**
 * @brief Initializes a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to initialize.
 */
BASICSDK_API void variant_init(TVARIANT &srcVar);

/**
 * @brief Sets the IVariantRead interface for a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param pVar A pointer to the IVariantRead interface.
 */
BASICSDK_API void variant_set_variantread(TVARIANT &srcVar, IVariantRead *pVar);

/**
 * @brief Sets the value of a TVARIANT structure from another TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param newVar The TVARIANT structure to copy the value from.
 */
BASICSDK_API void variant_set_variant(TVARIANT &srcVar, const TVARIANT &newVar);

/**
 * @brief Sets the string value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The string value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_string(TVARIANT &srcVar, const char *var, bool bIncludeType);

/**
 * @brief Sets the CHAR value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The CHAR value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_char(TVARIANT &srcVar, CHAR var, bool bIncludeType);

/**
 * @brief Sets the BYTE value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The BYTE value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_byte(TVARIANT &srcVar, BYTE var, bool bIncludeType);

/**
 * @brief Sets the SHORT value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The SHORT value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_short(TVARIANT &srcVar, SHORT var, bool bIncludeType);

/**
 * @brief Sets the USHORT value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The USHORT value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_ushort(TVARIANT &srcVar, USHORT var, bool bIncludeType);

/**
 * @brief Sets the INT value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The INT value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_int(TVARIANT &srcVar, INT var, bool bIncludeType);

/**
 * @brief Sets the UINT value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The UINT value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_uint(TVARIANT &srcVar, UINT var, bool bIncludeType);

/**
 * @brief Sets the LONG value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The LONG value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_long(TVARIANT &srcVar, LONG var, bool bIncludeType);

/**
 * @brief Sets the ULONG value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The ULONG value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_ulong(TVARIANT &srcVar, ULONG var, bool bIncludeType);

/**
 * @brief Sets the LONGLONG value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The LONGLONG value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_longlong(TVARIANT &srcVar, LONGLONG var, bool bIncludeType);

/**
 * @brief Sets the ULONGLONG value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The ULONGLONG value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_ulonglong(TVARIANT &srcVar, ULONGLONG var, bool bIncludeType);

/**
 * @brief Sets the FLOAT value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The FLOAT value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_float(TVARIANT &srcVar, FLOAT var, bool bIncludeType);

/**
 * @brief Sets the DOUBLE value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The DOUBLE value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_double(TVARIANT &srcVar, DOUBLE var, bool bIncludeType);

/**
 * @brief Sets the LPBSTRUNIT value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The LPBSTRUNIT value to set.
 */
BASICSDK_API void variant_set_bstrunit(TVARIANT &srcVar, LPBSTRUNIT var);

/**
 * @brief Sets the TVARIANT_BOOL value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to modify.
 * @param var The TVARIANT_BOOL value to set.
 * @param bIncludeType Boolean indicating whether to include the type information.
 */
BASICSDK_API void variant_set_bool(TVARIANT &srcVar, TVARIANT_BOOL var, bool bIncludeType);

/**
 * @brief Clears the value of a TVARIANT structure.
 *
 * @param srcVar The TVARIANT structure to clear.
 */
BASICSDK_API void variant_clear(TVARIANT &srcVar);

/**
 * @brief Compares two TVARIANT structures for equality.
 *
 * @param srcVar The first TVARIANT structure.
 * @param dstVar The second TVARIANT structure.
 * @return True if the structures are equal, false otherwise.
 */
BASICSDK_API bool variant_equal_variant(const TVARIANT &srcVar, const TVARIANT &dstVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a CHAR.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The CHAR value.
 */
BASICSDK_API CHAR variant_resolve_as_char(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a BYTE.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The BYTE value.
 */
BASICSDK_API BYTE variant_resolve_as_byte(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a SHORT.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The SHORT value.
 */
BASICSDK_API SHORT variant_resolve_as_short(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a USHORT.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The USHORT value.
 */
BASICSDK_API USHORT variant_resolve_as_ushort(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as an INT.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The INT value.
 */
BASICSDK_API INT variant_resolve_as_int(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a UINT.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The UINT value.
 */
BASICSDK_API UINT variant_resolve_as_uint(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a LONG.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The LONG value.
 */
BASICSDK_API LONG variant_resolve_as_long(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a ULONG.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The ULONG value.
 */
BASICSDK_API ULONG variant_resolve_as_ulong(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a LONGLONG.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The LONGLONG value.
 */
BASICSDK_API LONGLONG variant_resolve_as_longlong(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a ULONGLONG.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The ULONGLONG value.
 */
BASICSDK_API ULONGLONG variant_resolve_as_ulonglong(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a FLOAT.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The FLOAT value.
 */
BASICSDK_API FLOAT variant_resolve_as_float(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a DOUBLE.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The DOUBLE value.
 */
BASICSDK_API DOUBLE variant_resolve_as_double(TVARIANT &srcVar);

/**
 * @brief Resolves the value of a TVARIANT structure as a TVARIANT_BOOL.
 *
 * @param srcVar The TVARIANT structure to resolve.
 * @return The TVARIANT_BOOL value.
 */
BASICSDK_API TVARIANT_BOOL variant_resolve_as_bool(TVARIANT &srcVar);

/**
 * @brief Changes the type of a TVARIANT structure.
 *
 * @param var The TVARIANT structure to modify.
 * @param type The new TVARTYPE.
 * @return True if the type change succeeds, false otherwise.
 */
BASICSDK_API bool variant_change_type(TVARIANT &var, TVARTYPE type);

/**
 * @brief Gets the size of a data type in bits.
 *
 * @param type The TVARTYPE.
 * @param uMaxBit A reference to store the size of the data type in bits.
 * @return True if the operation succeeds, false otherwise.
 */
BASICSDK_API bool variant_get_datatype_size(TVARTYPE type, UINT &uMaxBit);

/**
 * @brief Adjusts the data value of a TVARIANT structure based on the specified size in bits.
 *
 * @param var The TVARIANT structure to modify.
 * @param iSize The size in bits.
 * @return True if the adjustment succeeds, false otherwise.
 */
BASICSDK_API bool variant_ajust_data_value(TVARIANT &var, int iSize);

/**
 * @brief Resolves the display type of a TVARTYPE.
 *
 * @param type The TVARTYPE.
 * @param strType A buffer to store the display type string.
 * @param dwLen A reference to the length of the buffer.
 * @return True if the operation succeeds, false otherwise.
 */
BASICSDK_API bool variant_resolve_display_type(TVARTYPE type, char *strType, DWORD &dwLen);

/**
 * @brief Resolves the TVARTYPE from a display type string.
 *
 * @param strType The display type string.
 * @param type A reference to store the resolved TVARTYPE.
 * @return True if the operation succeeds, false otherwise.
 */
BASICSDK_API bool variant_resolve_type(const char *strType, TVARTYPE &type);

class CVariant : public tagTVARIANT {
public:
    /**
     * @brief Default constructor for CVariant.
     *
     * Initializes a new CVariant object and calls variant_init to set default values.
     */
    CVariant() { variant_init(*this); }

    /**
     * @brief Constructor for CVariant from IVariantRead interface.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the IVariantRead interface using variant_set_variantread.
     *
     * @param pVar A pointer to the IVariantRead interface.
     */
    CVariant(IVariantRead *pVar) {
        variant_init(*this);
        variant_set_variantread(*this, pVar);
    }

    /**
     * @brief Constructor for CVariant from TVARIANT structure.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the value from another TVARIANT structure using variant_set_variant.
     *
     * @param var The TVARIANT structure to copy the value from.
     */
    CVariant(const TVARIANT &var) {
        variant_init(*this);
        variant_set_variant(*this, var);
    }

    /**
     * @brief Constructor for CVariant from constant pointer to TVARIANT structure.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the value from another TVARIANT structure if the pointer is not null.
     *
     * @param pVar A constant pointer to the TVARIANT structure to copy the value from.
     */
    CVariant(LPCTVARIANT pVar) {
        variant_init(*this);
        if (pVar != nullptr) {
            variant_set_variant(*this, *pVar);
        }
    }

    /**
     * @brief Copy constructor for CVariant.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the value from another CVariant object using variant_set_variant.
     *
     * @param var The CVariant object to copy the value from.
     */
    CVariant(const CVariant &var) {
        variant_init(*this);
        variant_set_variant(*this, var);
    }

    /**
     * @brief Constructor for CVariant from C-style string.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the string value using variant_set_string.
     *
     * @param var The C-style string to set as the value.
     */
    CVariant(const char *var) {
        variant_init(*this);
        variant_set_string(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from std::string.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the string value using variant_set_string.
     *
     * @param var The std::string to set as the value.
     */
    CVariant(const string &var) {
        variant_init(*this);
        variant_set_string(*this, var.c_str(), true);
    }

    /**
     * @brief Constructor for CVariant from CHAR.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the CHAR value using variant_set_char.
     *
     * @param var The CHAR value to set.
     */
    CVariant(CHAR var) {
        variant_init(*this);
        variant_set_char(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from BYTE.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the BYTE value using variant_set_byte.
     *
     * @param var The BYTE value to set.
     */
    CVariant(BYTE var) {
        variant_init(*this);
        variant_set_byte(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from SHORT.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the SHORT value using variant_set_short.
     *
     * @param var The SHORT value to set.
     */
    CVariant(SHORT var) {
        variant_init(*this);
        variant_set_short(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from USHORT.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the USHORT value using variant_set_ushort.
     *
     * @param var The USHORT value to set.
     */
    CVariant(USHORT var) {
        variant_init(*this);
        variant_set_ushort(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from INT.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the INT value using variant_set_int.
     *
     * @param var The INT value to set.
     */
    CVariant(INT var) {
        variant_init(*this);
        variant_set_int(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from UINT.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the UINT value using variant_set_uint.
     *
     * @param var The UINT value to set.
     */
    CVariant(UINT var) {
        variant_init(*this);
        variant_set_uint(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from LONG.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the LONG value using variant_set_long.
     *
     * @param var The LONG value to set.
     */
    CVariant(LONG var) {
        variant_init(*this);
        variant_set_long(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from ULONG.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the ULONG value using variant_set_ulong.
     *
     * @param var The ULONG value to set.
     */
    CVariant(ULONG var) {
        variant_init(*this);
        variant_set_ulong(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from LONGLONG.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the LONGLONG value using variant_set_longlong.
     *
     * @param var The LONGLONG value to set.
     */
    CVariant(LONGLONG var) {
        variant_init(*this);
        variant_set_longlong(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from ULONGLONG.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the ULONGLONG value using variant_set_ulonglong.
     *
     * @param var The ULONGLONG value to set.
     */
    CVariant(ULONGLONG var) {
        variant_init(*this);
        variant_set_ulonglong(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from FLOAT.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the FLOAT value using variant_set_float.
     *
     * @param var The FLOAT value to set.
     */
    CVariant(FLOAT var) {
        variant_init(*this);
        variant_set_float(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from DOUBLE.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the DOUBLE value using variant_set_double.
     *
     * @param var The DOUBLE value to set.
     */
    CVariant(DOUBLE var) {
        variant_init(*this);
        variant_set_double(*this, var, true);
    }

    /**
     * @brief Constructor for CVariant from LPBSTRUNIT.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the LPBSTRUNIT value using variant_set_bstrunit.
     *
     * @param var The LPBSTRUNIT value to set.
     */
    CVariant(LPBSTRUNIT var) {
        variant_init(*this);
        variant_set_bstrunit(*this, var);
    }

    /**
     * @brief Constructor for CVariant from TVARIANT_BOOL.
     *
     * Initializes a new CVariant object, sets default values using variant_init,
     * and sets the TVARIANT_BOOL value using variant_set_bool.
     *
     * @param var The TVARIANT_BOOL value to set.
     */
    CVariant(TVARIANT_BOOL var) {
        variant_init(*this);
        variant_set_bool(*this, var, true);
    }

    /**
     * @brief Destructor for CVariant.
     *
     * Clears the CVariant object using the clear method.
     */
    ~CVariant() { clear(); }

public:
    /**
     * @brief Clears the value of the CVariant object.
     *
     * Calls variant_clear to reset the CVariant object to its default state.
     */
    void clear() { variant_clear(*this); }

    /**
     * @brief Changes the type of the CVariant object.
     *
     * Calls variant_change_type to change the type of the CVariant object to the specified TVARTYPE.
     *
     * @param vartype The new TVARTYPE for the CVariant object.
     */
    void changeType(TVARTYPE vartype) { changeType(vartype, *this); }

    /**
     * @brief Sets the IVariantRead interface for the CVariant object.
     *
     * Calls variant_set_variantread to set the IVariantRead interface for the CVariant object.
     *
     * @param pVar A pointer to the IVariantRead interface.
     */
    void setVariant(IVariantRead *pVar) { variant_set_variantread(*this, pVar); }

    /**
     * @brief Compares the CVariant object with a TVARIANT structure for equality.
     *
     * Calls variant_equal_variant to compare the CVariant object with the specified TVARIANT structure.
     *
     * @param var The TVARIANT structure to compare with.
     * @return True if the objects are equal, false otherwise.
     */
    bool operator==(const TVARIANT &var) const { return variant_equal_variant(*this, var); }

    /**
     * @brief Compares the CVariant object with a constant pointer to a TVARIANT structure for equality.
     *
     * Calls variant_equal_variant to compare the CVariant object with the specified TVARIANT structure if the pointer
     * is not null.
     *
     * @param pVar A constant pointer to the TVARIANT structure to compare with.
     * @return True if the objects are equal, false otherwise.
     */
    bool operator==(LPCTVARIANT pVar) const {
        if (pVar != nullptr) {
            return variant_equal_variant(*this, *pVar);
        } else
            return false;
    }

    /**
     * @brief Compares the CVariant object with another CVariant object for equality.
     *
     * Calls variant_equal_variant to compare the CVariant object with the specified CVariant object.
     *
     * @param var The CVariant object to compare with.
     * @return True if the objects are equal, false otherwise.
     */
    bool operator==(const CVariant &var) const { return variant_equal_variant(*this, var); }

    /**
     * @brief Assigns the value of a TVARIANT structure to the CVariant object.
     *
     * Calls variant_set_variant to set the value of the CVariant object from the specified TVARIANT structure.
     *
     * @param var The TVARIANT structure to assign the value from.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(const TVARIANT &var) {
        variant_set_variant(*this, var);
        return *this;
    }

    /**
     * @brief Assigns the value of a constant pointer to a TVARIANT structure to the CVariant object.
     *
     * Calls variant_set_variant to set the value of the CVariant object from the specified TVARIANT structure if the
     * pointer is not null. Clears the CVariant object if the pointer is null.
     *
     * @param var A constant pointer to the TVARIANT structure to assign the value from.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(LPCTVARIANT var) {
        if (var != nullptr) {
            variant_set_variant(*this, *var);
            return *this;
        } else {
            variant_clear(*this);
            return *this;
        }
    }

    /**
     * @brief Assigns the value of another CVariant object to the CVariant object.
     *
     * Calls variant_set_variant to set the value of the CVariant object from the specified CVariant object.
     *
     * @param var The CVariant object to assign the value from.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(const CVariant &var) {
        variant_set_variant(*this, var);
        return *this;
    }

    /**
     * @brief Assigns a C-style string to the CVariant object.
     *
     * Calls variant_set_string to set the string value of the CVariant object.
     *
     * @param var The C-style string to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(const char *var) {
        variant_set_string(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a std::string to the CVariant object.
     *
     * Calls variant_set_string to set the string value of the CVariant object.
     *
     * @param var The std::string to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(const string &var) {
        variant_set_string(*this, var.c_str(), true);
        return *this;
    }

    /**
     * @brief Assigns a CHAR value to the CVariant object.
     *
     * Calls variant_set_char to set the CHAR value of the CVariant object.
     *
     * @param var The CHAR value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(CHAR var) {
        variant_set_char(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a BYTE value to the CVariant object.
     *
     * Calls variant_set_byte to set the BYTE value of the CVariant object.
     *
     * @param var The BYTE value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(BYTE var) {
        variant_set_byte(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a SHORT value to the CVariant object.
     *
     * Calls variant_set_short to set the SHORT value of the CVariant object.
     *
     * @param var The SHORT value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(SHORT var) {
        variant_set_short(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a USHORT value to the CVariant object.
     *
     * Calls variant_set_ushort to set the USHORT value of the CVariant object.
     *
     * @param var The USHORT value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(USHORT var) {
        variant_set_ushort(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns an INT value to the CVariant object.
     *
     * Calls variant_set_int to set the INT value of the CVariant object.
     *
     * @param var The INT value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(INT var) {
        variant_set_int(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a UINT value to the CVariant object.
     *
     * Calls variant_set_uint to set the UINT value of the CVariant object.
     *
     * @param var The UINT value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(UINT var) {
        variant_set_uint(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a LONG value to the CVariant object.
     *
     * Calls variant_set_long to set the LONG value of the CVariant object.
     *
     * @param var The LONG value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(LONG var) {
        variant_set_long(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a ULONG value to the CVariant object.
     *
     * Calls variant_set_ulong to set the ULONG value of the CVariant object.
     *
     * @param var The ULONG value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(ULONG var) {
        variant_set_ulong(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a LONGLONG value to the CVariant object.
     *
     * Calls variant_set_longlong to set the LONGLONG value of the CVariant object.
     *
     * @param var The LONGLONG value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(LONGLONG var) {
        variant_set_longlong(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a ULONGLONG value to the CVariant object.
     *
     * Calls variant_set_ulonglong to set the ULONGLONG value of the CVariant object.
     *
     * @param var The ULONGLONG value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(ULONGLONG var) {
        variant_set_ulonglong(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a FLOAT value to the CVariant object.
     *
     * Calls variant_set_float to set the FLOAT value of the CVariant object.
     *
     * @param var The FLOAT value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(FLOAT var) {
        variant_set_float(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a DOUBLE value to the CVariant object.
     *
     * Calls variant_set_double to set the DOUBLE value of the CVariant object.
     *
     * @param var The DOUBLE value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(DOUBLE var) {
        variant_set_double(*this, var, true);
        return *this;
    }

    /**
     * @brief Assigns a LPBSTRUNIT value to the CVariant object.
     *
     * Calls variant_set_bstrunit to set the LPBSTRUNIT value of the CVariant object.
     *
     * @param var The LPBSTRUNIT value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(LPBSTRUNIT var) {
        variant_set_bstrunit(*this, var);
        return *this;
    }

    /**
     * @brief Assigns a TVARIANT_BOOL value to the CVariant object.
     *
     * Calls variant_set_bool to set the TVARIANT_BOOL value of the CVariant object.
     *
     * @param var The TVARIANT_BOOL value to assign.
     * @return A reference to the current CVariant object.
     */
    const CVariant &operator=(TVARIANT_BOOL var) {
        variant_set_bool(*this, var, true);
        return *this;
    }

    /**
     * @brief Conversion operator to LPTVARIANT.
     *
     * Returns a pointer to the CVariant object cast to LPTVARIANT.
     *
     * @return A pointer to the CVariant object as LPTVARIANT.
     */
    operator LPTVARIANT() { return this; }
    /**
     * @brief Conversion operator to CHAR.
     *
     * Resolves the value of the CVariant object as a CHAR.
     *
     * @return The CHAR value of the CVariant object.
     */
    operator CHAR() { return variant_resolve_as_char(*this); }

    /**
     * @brief Conversion operator to BYTE.
     *
     * Resolves the value of the CVariant object as a BYTE.
     *
     * @return The BYTE value of the CVariant object.
     */
    operator BYTE() { return variant_resolve_as_byte(*this); }

    /**
     * @brief Conversion operator to SHORT.
     *
     * Resolves the value of the CVariant object as a SHORT.
     *
     * @return The SHORT value of the CVariant object.
     */
    operator SHORT() { return variant_resolve_as_short(*this); }

    /**
     * @brief Conversion operator to USHORT.
     *
     * Resolves the value of the CVariant object as a USHORT.
     *
     * @return The USHORT value of the CVariant object.
     */
    operator USHORT() { return variant_resolve_as_ushort(*this); }

    /**
     * @brief Conversion operator to INT.
     *
     * Resolves the value of the CVariant object as an INT.
     *
     * @return The INT value of the CVariant object.
     */
    operator INT() { return variant_resolve_as_int(*this); }

    /**
     * @brief Conversion operator to UINT.
     *
     * Resolves the value of the CVariant object as a UINT.
     *
     * @return The UINT value of the CVariant object.
     */
    operator UINT() { return variant_resolve_as_uint(*this); }

    /**
     * @brief Conversion operator to LONG.
     *
     * Resolves the value of the CVariant object as a LONG.
     *
     * @return The LONG value of the CVariant object.
     */
    operator LONG() { return variant_resolve_as_long(*this); }

    /**
     * @brief Conversion operator to ULONG.
     *
     * Resolves the value of the CVariant object as a ULONG.
     *
     * @return The ULONG value of the CVariant object.
     */
    operator ULONG() { return variant_resolve_as_ulong(*this); }

    /**
     * @brief Conversion operator to LONGLONG.
     *
     * Resolves the value of the CVariant object as a LONGLONG.
     *
     * @return The LONGLONG value of the CVariant object.
     */
    operator LONGLONG() { return variant_resolve_as_longlong(*this); }

    /**
     * @brief Conversion operator to ULONGLONG.
     *
     * Resolves the value of the CVariant object as a ULONGLONG.
     *
     * @return The ULONGLONG value of the CVariant object.
     */
    operator ULONGLONG() { return variant_resolve_as_ulonglong(*this); }

    /**
     * @brief Conversion operator to FLOAT.
     *
     * Resolves the value of the CVariant object as a FLOAT.
     *
     * @return The FLOAT value of the CVariant object.
     */
    operator FLOAT() { return variant_resolve_as_float(*this); }

    /**
     * @brief Conversion operator to DOUBLE.
     *
     * Resolves the value of the CVariant object as a DOUBLE.
     *
     * @return The DOUBLE value of the CVariant object.
     */
    operator DOUBLE() { return variant_resolve_as_double(*this); }

    /**
     * @brief Conversion operator to TVARIANT_BOOL.
     *
     * Resolves the value of the CVariant object as a TVARIANT_BOOL.
     *
     * @return The TVARIANT_BOOL value of the CVariant object.
     */
    operator TVARIANT_BOOL() { return variant_resolve_as_bool(*this); }

    /**
     * @brief Sets the CHAR value of the CVariant object.
     *
     * Calls variant_set_char to set the CHAR value of the CVariant object.
     *
     * @param cValue The CHAR value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(CHAR cValue, bool bIncludeType) { variant_set_char(*this, cValue, bIncludeType); }

    /**
     * @brief Sets the BYTE value of the CVariant object.
     *
     * Calls variant_set_byte to set the BYTE value of the CVariant object.
     *
     * @param bValue The BYTE value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(BYTE bValue, bool bIncludeType) { variant_set_byte(*this, bValue, bIncludeType); }

    /**
     * @brief Sets the SHORT value of the CVariant object.
     *
     * Calls variant_set_short to set the SHORT value of the CVariant object.
     *
     * @param iValue The SHORT value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(SHORT iValue, bool bIncludeType) { variant_set_short(*this, iValue, bIncludeType); }

    /**
     * @brief Sets the USHORT value of the CVariant object.
     *
     * Calls variant_set_ushort to set the USHORT value of the CVariant object.
     *
     * @param uiValue The USHORT value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(USHORT uiValue, bool bIncludeType) { variant_set_ushort(*this, uiValue, bIncludeType); }

    /**
     * @brief Sets the INT value of the CVariant object.
     *
     * Calls variant_set_int to set the INT value of the CVariant object.
     *
     * @param intValue The INT value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(INT intValue, bool bIncludeType) { variant_set_int(*this, intValue, bIncludeType); }

    /**
     * @brief Sets the UINT value of the CVariant object.
     *
     * Calls variant_set_uint to set the UINT value of the CVariant object.
     *
     * @param uintValue The UINT value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(UINT uintValue, bool bIncludeType) { variant_set_uint(*this, uintValue, bIncludeType); }

    /**
     * @brief Sets the LONG value of the CVariant object.
     *
     * Calls variant_set_long to set the LONG value of the CVariant object.
     *
     * @param lValue The LONG value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(LONG lValue, bool bIncludeType) { variant_set_long(*this, lValue, bIncludeType); }

    /**
     * @brief Sets the ULONG value of the CVariant object.
     *
     * Calls variant_set_ulong to set the ULONG value of the CVariant object.
     *
     * @param ulValue The ULONG value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(ULONG ulValue, bool bIncludeType) { variant_set_ulong(*this, ulValue, bIncludeType); }

    /**
     * @brief Sets the LONGLONG value of the CVariant object.
     *
     * Calls variant_set_longlong to set the LONGLONG value of the CVariant object.
     *
     * @param llValue The LONGLONG value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(LONGLONG llValue, bool bIncludeType) { variant_set_longlong(*this, llValue, bIncludeType); }

    /**
     * @brief Sets the ULONGLONG value of the CVariant object.
     *
     * Calls variant_set_ulonglong to set the ULONGLONG value of the CVariant object.
     *
     * @param ullValue The ULONGLONG value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(ULONGLONG ullValue, bool bIncludeType) { variant_set_ulonglong(*this, ullValue, bIncludeType); }

    /**
     * @brief Sets the FLOAT value of the CVariant object.
     *
     * Calls variant_set_float to set the FLOAT value of the CVariant object.
     *
     * @param fltValue The FLOAT value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(FLOAT fltValue, bool bIncludeType) { variant_set_float(*this, fltValue, bIncludeType); }

    /**
     * @brief Sets the DOUBLE value of the CVariant object.
     *
     * Calls variant_set_double to set the DOUBLE value of the CVariant object.
     *
     * @param dblValue The DOUBLE value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(DOUBLE dblValue, bool bIncludeType) { variant_set_double(*this, dblValue, bIncludeType); }

    /**
     * @brief Sets the string value of the CVariant object.
     *
     * Calls variant_set_string to set the string value of the CVariant object.
     *
     * @param strValue The C-style string to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(const char *strValue, bool bIncludeType) { variant_set_string(*this, strValue, bIncludeType); }

    /**
     * @brief Sets the TVARIANT_BOOL value of the CVariant object.
     *
     * Calls variant_set_bool to set the TVARIANT_BOOL value of the CVariant object.
     *
     * @param boolValue The TVARIANT_BOOL value to set.
     * @param bIncludeType Boolean indicating whether to include the type information.
     */
    void setValue(TVARIANT_BOOL boolValue, bool bIncludeType) { variant_set_bool(*this, boolValue, bIncludeType); }

public:
    /**
     * @brief Changes the type of a CVariant object.
     *
     * Calls variant_change_type to change the type of the specified CVariant object to the specified TVARTYPE.
     *
     * @param type The new TVARTYPE for the CVariant object.
     * @param var The CVariant object to modify.
     * @return True if the type change succeeds, false otherwise.
     */
    static bool changeType(TVARTYPE type, CVariant &var) { return variant_change_type(var, type); }

    /**
     * @brief Gets the size of a data type in bits.
     *
     * Calls variant_get_datatype_size to get the size of the specified TVARTYPE in bits.
     *
     * @param type The TVARTYPE.
     * @param uMaxBit A reference to store the size of the data type in bits.
     * @return True if the operation succeeds, false otherwise.
     */
    static bool getDataTypeSize(TVARTYPE type, UINT &uMaxBit) { return variant_get_datatype_size(type, uMaxBit); }

    /**
     * @brief Adjusts the data value of a CVariant object based on the specified size in bits.
     *
     * Calls variant_ajust_data_value to adjust the data value of the specified CVariant object.
     *
     * @param var The CVariant object to modify.
     * @param iSize The size in bits.
     * @return True if the adjustment succeeds, false otherwise.
     */
    static bool ajustDataValue(CVariant &var, int iSize)  // 结合变量大小[占用的BIT数]将值进行合适的转换以适合有符号数
    {
        return variant_ajust_data_value(var, iSize);
    }

public:
    /**
     * @brief Gets the string value of the CVariant object.
     *
     * Resolves the value of the CVariant object as a string.
     *
     * @param strValue A reference to store the string value.
     */
    void getValue(string &strValue) {
        CBStr bstrValue;
        if (vt == TVT_BSTR)
            bstrValue = bstrVal;
        else {
            TVARIANT newVar;
            variant_init(newVar);
            variant_set_variant(newVar, *this);
            variant_change_type(newVar, TVT_BSTR);
            bstrValue = newVar.bstrVal;
        }
        strValue = (const char *)bstrValue;
    }

public:
    /**
     * @brief Resolves the display type of a TVARTYPE.
     *
     * Calls variant_resolve_display_type to get the display type string of the specified TVARTYPE.
     *
     * @param type The TVARTYPE.
     * @param strType A reference to store the display type string.
     * @return True if the operation succeeds, false otherwise.
     */
    static bool resolveDataType(TVARTYPE type, string &strType) {
        DWORD dwLen = 0;
        if (variant_resolve_display_type(type, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            variant_resolve_display_type(type, pValue, dwLen);
            strType = pValue;
            delete[] pValue;
            pValue = nullptr;
            return true;
        } else
            return false;
    }

    /**
     * @brief Resolves the TVARTYPE from a display type string.
     *
     * Calls variant_resolve_type to get the TVARTYPE from the specified display type string.
     *
     * @param strType The display type string.
     * @param type A reference to store the resolved TVARTYPE.
     * @return True if the operation succeeds, false otherwise.
     */
    static bool resolveDataType(string strType, TVARTYPE &type) { return variant_resolve_type(strType.c_str(), type); }
};

#endif
