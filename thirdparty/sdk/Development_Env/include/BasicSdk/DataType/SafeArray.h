#ifndef __BASICSDK_SAFE_ARRAY_H__
#define __BASICSDK_SAFE_ARRAY_H__

/**
 * @file SafeArray.h
 * @brief Header file for the safe array management functions and classes.
 */

#include "BasicSdkDef.h"
#include "DataType/BStr.h"
#include "DataType/Variant.h"

/**
 * @brief Opaque handle to a safe array.
 */
struct safearray_handle;

/**
 * @brief Creates a new safe array.
 *
 * @param vtElementType The type of elements in the safe array.
 * @param uintElementCount The initial number of elements in the safe array (default is 0).
 * @return A pointer to the newly created safe array handle.
 */
BASICSDK_API safearray_handle *safearray_create(TVARTYPE vtElementType, unsigned int uintElementCount = 0);

/**
 * @brief Destroys a safe array.
 *
 * @param handle Reference to the safe array handle to be destroyed.
 */
BASICSDK_API void safearray_destroy(safearray_handle *&handle);

/**
 * @brief Clears all elements in a safe array.
 *
 * @param handle Pointer to the safe array handle.
 */
BASICSDK_API void safearray_clear(safearray_handle *handle);

/**
 * @brief Retrieves the type of elements in a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @return The type of elements in the safe array.
 */
BASICSDK_API TVARTYPE safearray_get_element_type(safearray_handle *handle);

/**
 * @brief Retrieves the number of elements in a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @return The number of elements in the safe array.
 */
BASICSDK_API unsigned int safearray_get_element_count(safearray_handle *handle);

/**
 * @brief Sets an element in a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @param uintIndex Index of the element to set.
 * @param element Pointer to the element value to set.
 * @return True if the element was successfully set, false otherwise.
 */
BASICSDK_API bool safearray_set_element(safearray_handle *handle, unsigned int uintIndex, void *element);

/**
 * @brief Retrieves an element from a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @param uintIndex Index of the element to retrieve.
 * @param element Pointer to store the retrieved element value.
 * @return True if the element was successfully retrieved, false otherwise.
 */
BASICSDK_API bool safearray_get_element(safearray_handle *handle, unsigned int uintIndex, void *element);

/**
 * @brief Accesses all elements in a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @param elementArray Pointer to store the array of element values.
 * @return True if the elements were successfully accessed, false otherwise.
 */
BASICSDK_API bool safearray_access_element(safearray_handle *handle, void **elementArray);

/**
 * @brief Unaccesses all elements in a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @return True if the elements were successfully unaccessed, false otherwise.
 */
BASICSDK_API bool safearray_unaccess_element(safearray_handle *handle);

/**
 * @brief Resizes a safe array.
 *
 * @param handle Pointer to the safe array handle.
 * @param uintElementCount The new number of elements in the safe array.
 * @return True if the safe array was successfully resized, false otherwise.
 */
BASICSDK_API bool safearray_resize(safearray_handle *handle, unsigned int uintElementCount);

/**
 * @brief Template class representing an item in a safe array.
 *
 * This class provides a way to access and modify elements in a safe array
 * using the array subscript operator. It handles the conversion between
 * the underlying data type and the template type.
 *
 * @tparam T The type of the elements in the safe array.
 */
template <class T>
class _ArrayItem {
public:
    /**
     * @brief Constructor.
     *
     * Initializes the array item with a handle to the safe array and the index of the element.
     *
     * @param hSafeArrayHandle Handle to the safe array.
     * @param uintIndex Index of the element in the safe array.
     */
    _ArrayItem(safearray_handle *hSafeArrayHandle, unsigned int uintIndex) {
        m_hSafeArrayHandle = hSafeArrayHandle;
        m_uintIndex = uintIndex;
    }

public:
    /**
     * @brief Returns the value of the element.
     *
     * @return The value of the element at the specified index.
     */
    const T value() { return (T)(*this); }

    /**
     * @brief Sets the value of the element.
     *
     * @param value The new value to set for the element.
     * @return Reference to the current _ArrayItem object.
     */
    const _ArrayItem &operator=(T value) {
        safearray_set_element(m_hSafeArrayHandle, m_uintIndex, (void *)&value);
        return *this;
    }

    /**
     * @brief Copies the value from another _ArrayItem.
     *
     * @param item The _ArrayItem to copy the value from.
     * @return Reference to the current _ArrayItem object.
     */
    const _ArrayItem &operator=(const _ArrayItem &item) {
        T value = 0;
        safearray_get_element(item.m_hSafeArrayHandle, item.m_uintIndex, (void *)&value);
        safearray_set_element(m_hSafeArrayHandle, m_uintIndex, (void *)&value);
        return *this;
    }

    /**
     * @brief Implicit conversion to the element's type.
     *
     * @return The value of the element at the specified index.
     */
    operator T() const noexcept {
        T value = 0;
        safearray_get_element(m_hSafeArrayHandle, m_uintIndex, (void *)&value);
        return value;
    }

protected:
    /**
     * @brief Handle to the safe array.
     */
    safearray_handle *m_hSafeArrayHandle;

    /**
     * @brief Index of the element in the safe array.
     */
    unsigned int m_uintIndex;
};

/**
 * @brief Specialization of _ArrayItem for std::string.
 *
 * This specialization handles the conversion between std::string and the underlying BSTR type.
 */
template <>
class _ArrayItem<string> {
public:
    /**
     * @brief Constructor.
     *
     * Initializes the array item with a handle to the safe array and the index of the element.
     *
     * @param hSafeArrayHandle Handle to the safe array.
     * @param uintIndex Index of the element in the safe array.
     */
    _ArrayItem(safearray_handle *hSafeArrayHandle, unsigned int uintIndex) {
        m_hSafeArrayHandle = hSafeArrayHandle;
        m_uintIndex = uintIndex;
    }

    /**
     * @brief Copy constructor.
     *
     * @param item The _ArrayItem to copy.
     */
    _ArrayItem(const _ArrayItem &item) {
        m_hSafeArrayHandle = item.m_hSafeArrayHandle;
        m_uintIndex = item.m_uintIndex;
    }

public:
    /**
     * @brief Returns the value of the element as a std::string.
     *
     * @return The value of the element at the specified index as a std::string.
     */
    const string value() { return (string)(*this); }

    /**
     * @brief Sets the value of the element.
     *
     * @param value The new value to set for the element.
     * @return Reference to the current _ArrayItem object.
     */
    const _ArrayItem &operator=(string value) {
        bstr_handle *bstrValue = bstr_create();
        bstr_set_text(bstrValue, value.c_str());
        LPBSTRUNIT bstrUnit = bstr_detach_bstrunit(bstrValue);
        safearray_set_element(m_hSafeArrayHandle, m_uintIndex, (void *)&bstrUnit);
        bstr_attach_bstrunit(bstrValue, bstrUnit);
        bstr_destroy(bstrValue);
        return *this;
    }

    /**
     * @brief Copies the value from another _ArrayItem.
     *
     * @param item The _ArrayItem to copy the value from.
     * @return Reference to the current _ArrayItem object.
     */
    const _ArrayItem &operator=(const _ArrayItem &item) {
        if (m_hSafeArrayHandle == item.m_hSafeArrayHandle && m_uintIndex == item.m_uintIndex) {
            return *this;
        }
        LPBSTRUNIT bstrUnit;
        safearray_get_element(item.m_hSafeArrayHandle, item.m_uintIndex, (void *)&bstrUnit);
        safearray_set_element(m_hSafeArrayHandle, m_uintIndex, (void *)&bstrUnit);
        bstr_handle *bstrValue = bstr_create();
        bstr_attach_bstrunit(bstrValue, bstrUnit);
        bstr_destroy(bstrValue);
        return *this;
    }

    /**
     * @brief Implicit conversion to std::string.
     *
     * @return The value of the element at the specified index as a std::string.
     */
    operator string() const noexcept {
        LPBSTRUNIT bstrUnit;
        safearray_get_element(m_hSafeArrayHandle, m_uintIndex, (void *)&bstrUnit);
        bstr_handle *bstrValue = bstr_create();
        bstr_attach_bstrunit(bstrValue, bstrUnit);
        const char *strValue = bstr_get_data(bstrValue);
        string value = (strValue != nullptr ? strValue : "");
        bstr_destroy(bstrValue);
        return value;
    }

protected:
    /**
     * @brief Handle to the safe array.
     */
    safearray_handle *m_hSafeArrayHandle;

    /**
     * @brief Index of the element in the safe array.
     */
    unsigned int m_uintIndex;
};
/**
 * @brief Template class for a restricted safe array.
 *
 * This class provides a restricted interface to manage a safe array with a specific element type.
 * It ensures type safety and provides methods to manipulate the elements of the array.
 *
 * @tparam T The type of the elements in the safe array.
 * @tparam itemType The type of the elements as defined by TVARTYPE.
 */
template <class T, TVARTYPE itemType>
class CRestrictSafeArray {
public:
    /**
     * @brief Default constructor.
     *
     * Initializes the safe array handle to nullptr.
     */
    CRestrictSafeArray() { m_hSafeArrayHandle = nullptr; }

    /**
     * @brief Constructor with element count.
     *
     * Creates a safe array with the specified number of elements.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CRestrictSafeArray(unsigned int uintElementCount) {
        m_hSafeArrayHandle = safearray_create(itemType, uintElementCount);
    }

    /**
     * @brief Destructor.
     *
     * Clears and destroys the safe array.
     */
    virtual ~CRestrictSafeArray() {
        safearray_clear(m_hSafeArrayHandle);
        safearray_destroy(m_hSafeArrayHandle);
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of the safe array to ensure unique ownership.
     */
    CRestrictSafeArray(const CRestrictSafeArray &) = delete;

public:
    /**
     * @brief Deleted copy assignment operator.
     *
     * Prevents assignment of the safe array to ensure unique ownership.
     */
    CRestrictSafeArray &operator=(const CRestrictSafeArray &) = delete;
#endif

public:
    /**
     * @brief Clears all elements in the safe array.
     */
    void clear() { safearray_clear(m_hSafeArrayHandle); }

    /**
     * @brief Retrieves the number of elements in the safe array.
     *
     * @return The number of elements in the safe array.
     */
    unsigned int getElementCount() { return safearray_get_element_count(m_hSafeArrayHandle); }

    /**
     * @brief Sets an element in the safe array.
     *
     * @param uintIndex Index of the element to set.
     * @param value The value to set for the element.
     * @return True if the element was successfully set, false otherwise.
     */
    bool setElement(unsigned int uintIndex, T value) {
        return safearray_set_element(m_hSafeArrayHandle, uintIndex, (void *)&value);
    }

    /**
     * @brief Retrieves an element from the safe array.
     *
     * @param uintIndex Index of the element to retrieve.
     * @param value Reference to store the retrieved element value.
     * @return True if the element was successfully retrieved, false otherwise.
     */
    bool getElement(unsigned int uintIndex, T &value) {
        return safearray_get_element(m_hSafeArrayHandle, uintIndex, (void *)&value);
    }

    /**
     * @brief Accesses all elements in the safe array.
     *
     * @param elementArray Pointer to store the array of element values.
     * @return True if the elements were successfully accessed, false otherwise.
     */
    bool accessElement(T **elementArray) { return safearray_access_element(m_hSafeArrayHandle, (void **)elementArray); }

    /**
     * @brief Unaccesses all elements in the safe array.
     *
     * @return True if the elements were successfully unaccessed, false otherwise.
     */
    bool unaccessElement() { return safearray_unaccess_element(m_hSafeArrayHandle); }

    /**
     * @brief Resizes the safe array.
     *
     * @param uintElementCount The new number of elements in the safe array.
     * @return True if the safe array was successfully resized, false otherwise.
     */
    bool resize(unsigned int uintElementCount) {
        if (m_hSafeArrayHandle == nullptr) m_hSafeArrayHandle = safearray_create(itemType, uintElementCount);
        return safearray_resize(m_hSafeArrayHandle, uintElementCount);
    }

    /**
     * @brief Attaches an existing safe array handle.
     *
     * @param handle The safe array handle to attach.
     * @return True if the handle was successfully attached, false otherwise.
     */
    bool attach(safearray_handle *handle) {
        if (m_hSafeArrayHandle == nullptr && handle != nullptr && safearray_get_element_type(handle) == itemType) {
            m_hSafeArrayHandle = handle;
            return true;
        } else
            return false;
    }

    /**
     * @brief Detaches the safe array handle.
     *
     * @return The detached safe array handle.
     */
    safearray_handle *detach() {
        safearray_handle *oldSafeArrayHandle = m_hSafeArrayHandle;
        m_hSafeArrayHandle = nullptr;
        return oldSafeArrayHandle;
    }

    /**
     * @brief Gets a reference to the safe array handle.
     *
     * @return Reference to the safe array handle.
     */
    safearray_handle *&handle() { return m_hSafeArrayHandle; }

    /**
     * @brief Subscript operator to access elements.
     *
     * @param uintIndex Index of the element to access.
     * @return An _ArrayItem object representing the element.
     */
    _ArrayItem<T> operator[](unsigned int uintIndex) noexcept { return _ArrayItem<T>(m_hSafeArrayHandle, uintIndex); }

    /**
     * @brief Const subscript operator to access elements.
     *
     * @param uintIndex Index of the element to access.
     * @return A const _ArrayItem object representing the element.
     */
    const _ArrayItem<T> operator[](unsigned int uintIndex) const noexcept {
        return _ArrayItem<T>(m_hSafeArrayHandle, uintIndex);
    }

protected:
    /**
     * @brief Handle to the safe array.
     */
    safearray_handle *m_hSafeArrayHandle;
};

/**
 * @brief Template class for a safe array.
 *
 * This class is a specialization of CRestrictSafeArray with a default item type of TVT_NULL.
 *
 * @tparam T The type of the elements in the safe array.
 */
template <class T>
class CSafeArray : public CRestrictSafeArray<T, TVT_NULL> {};

/**
 * @brief Specialization of CSafeArray for CHAR.
 *
 * This specialization sets the item type to TVT_I1.
 */
template <>
class CSafeArray<CHAR> : public CRestrictSafeArray<CHAR, TVT_I1> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}

public:
    /**
     * @brief Retrieves the type of elements in the safe array.
     *
     * @return The type of elements in the safe array.
     */
    virtual TVARTYPE getItemType() { return TVT_I1; }
};

/**
 * @brief Specialization of CSafeArray for BYTE.
 *
 * This specialization sets the item type to TVT_UI1.
 */
template <>
class CSafeArray<BYTE> : public CRestrictSafeArray<BYTE, TVT_UI1> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for SHORT.
 *
 * This specialization sets the item type to TVT_I2.
 */
template <>
class CSafeArray<SHORT> : public CRestrictSafeArray<SHORT, TVT_I2> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for USHORT.
 *
 * This specialization sets the item type to TVT_UI2.
 */
template <>
class CSafeArray<USHORT> : public CRestrictSafeArray<USHORT, TVT_UI2> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for INT.
 *
 * This specialization sets the item type to TVT_INT.
 */
template <>
class CSafeArray<INT> : public CRestrictSafeArray<INT, TVT_INT> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for UINT.
 *
 * This specialization sets the item type to TVT_UINT.
 */
template <>
class CSafeArray<UINT> : public CRestrictSafeArray<UINT, TVT_UINT> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for LONG.
 *
 * This specialization sets the item type to TVT_I4.
 */
template <>
class CSafeArray<LONG> : public CRestrictSafeArray<LONG, TVT_I4> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for ULONG.
 *
 * This specialization sets the item type to TVT_UI4.
 */
template <>
class CSafeArray<ULONG> : public CRestrictSafeArray<ULONG, TVT_UI4> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for LONGLONG.
 *
 * This specialization sets the item type to TVT_I8.
 */
template <>
class CSafeArray<LONGLONG> : public CRestrictSafeArray<LONGLONG, TVT_I8> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for ULONGLONG.
 *
 * This specialization sets the item type to TVT_UI8.
 */
template <>
class CSafeArray<ULONGLONG> : public CRestrictSafeArray<ULONGLONG, TVT_UI8> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for FLOAT.
 *
 * This specialization sets the item type to TVT_R4.
 */
template <>
class CSafeArray<FLOAT> : public CRestrictSafeArray<FLOAT, TVT_R4> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for DOUBLE.
 *
 * This specialization sets the item type to TVT_R8.
 */
template <>
class CSafeArray<DOUBLE> : public CRestrictSafeArray<DOUBLE, TVT_R8> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for LPBSTRUNIT.
 *
 * This specialization sets the item type to TVT_BSTR.
 */
template <>
class CSafeArray<LPBSTRUNIT> : public CRestrictSafeArray<LPBSTRUNIT, TVT_BSTR> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for TVARIANT_BOOL.
 *
 * This specialization sets the item type to TVT_BOOL.
 */
template <>
class CSafeArray<TVARIANT_BOOL> : public CRestrictSafeArray<TVARIANT_BOOL, TVT_BOOL> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}
};

/**
 * @brief Specialization of CSafeArray for std::string.
 *
 * This specialization sets the item type to TVT_BSTR and provides methods to handle string elements in a safe array.
 */
template <>
class CSafeArray<string> : public CRestrictSafeArray<LPBSTRUNIT, TVT_BSTR> {
public:
    /**
     * @brief Default constructor.
     */
    CSafeArray() : CRestrictSafeArray() {}

    /**
     * @brief Constructor with element count.
     *
     * @param uintElementCount The initial number of elements in the safe array.
     */
    CSafeArray(unsigned int uintElementCount) : CRestrictSafeArray(uintElementCount) {}

public:
    /**
     * @brief Sets an element in the safe array.
     *
     * @param uintIndex Index of the element to set.
     * @param value The string value to set for the element.
     * @return True if the element was successfully set, false otherwise.
     */
    bool setElement(unsigned int uintIndex, string value) {
        bstr_handle *bstrValue = bstr_create();
        bstr_set_text(bstrValue, value.c_str());
        LPBSTRUNIT bstrUnit = bstr_detach_bstrunit(bstrValue);
        bool ret = safearray_set_element(m_hSafeArrayHandle, uintIndex, (void *)&bstrUnit);
        bstr_attach_bstrunit(bstrValue, bstrUnit);
        bstr_destroy(bstrValue);
        return ret;
    }

    /**
     * @brief Retrieves an element from the safe array.
     *
     * @param uintIndex Index of the element to retrieve.
     * @param value Reference to store the retrieved string value.
     * @return True if the element was successfully retrieved, false otherwise.
     */
    bool getElement(unsigned int uintIndex, string &value) {
        LPBSTRUNIT bstrUnit;
        bool ret = safearray_get_element(m_hSafeArrayHandle, uintIndex, (void *)&bstrUnit);
        bstr_handle *bstrValue = bstr_create();
        bstr_attach_bstrunit(bstrValue, bstrUnit);
        const char *strValue = bstr_get_data(bstrValue);
        value = (strValue != nullptr ? strValue : "");
        bstr_destroy(bstrValue);
        return ret;
    }

    /**
     * @brief Subscript operator to access elements.
     *
     * @param uintIndex Index of the element to access.
     * @return An _ArrayItem object representing the element.
     */
    _ArrayItem<string> operator[](unsigned int uintIndex) noexcept {
        return _ArrayItem<string>(m_hSafeArrayHandle, uintIndex);
    }

    /**
     * @brief Const subscript operator to access elements.
     *
     * @param uintIndex Index of the element to access.
     * @return A const string representing the element.
     */
    const string operator[](unsigned int uintIndex) const noexcept {
        return (const string)_ArrayItem<string>(m_hSafeArrayHandle, uintIndex);
    }
};

/**
 * @brief Converts a safearray to a fixed-size C-style array.
 *
 * @tparam T The type of elements in the array.
 * @tparam N The size of the array.
 * @param handle The safearray handle to convert.
 * @param data Reference to the C-style array to store the result.
 * @return True if the conversion was successful, false otherwise.
 */
template <typename T, int N>
bool afxSafeArrayToArray(safearray_handle *handle, T (&data)[N]) {
    CSafeArray<T> safeArray;
    if (safearray_get_element_count(handle) != N || safeArray.attach(handle) == false) {
        return false;
    }
    for (unsigned int i = 0; i < N; i++) {
        safeArray.getElement(i, data[i]);
    }
    safeArray.detach();
    return true;
}

/**
 * @brief Converts a fixed-size C-style array to a safearray.
 *
 * @tparam T The type of elements in the array.
 * @tparam N The size of the array.
 * @param data Reference to the C-style array to convert.
 * @param handle The safearray handle to store the result.
 * @return True if the conversion was successful, false otherwise.
 */
template <typename T, int N>
bool afxArrayToSafeArray(const T (&data)[N], safearray_handle *&handle) {
    CSafeArray<T> safeArray;
    if (handle != nullptr && safeArray.attach(handle) == false) {
        return false;
    }
    if (safeArray.resize(N) == true) {
        for (unsigned int i = 0; i < N; i++) {
            safeArray.setElement(i, data[i]);
        }
    }
    handle = safeArray.detach();
    return true;
}

/**
 * @brief Converts a safearray to a dynamic C-style array.
 *
 * @tparam T The type of elements in the array.
 * @param handle The safearray handle to convert.
 * @param data Pointer to the C-style array to store the result.
 * @param dataLen The length of the C-style array.
 * @return True if the conversion was successful, false otherwise.
 */
template <typename T>
bool afxSafeArrayToArray(safearray_handle *handle, T *data, const unsigned int dataLen) {
    CSafeArray<T> safeArray;
    if (safearray_get_element_count(handle) != dataLen || safeArray.attach(handle) == false) {
        return false;
    }
    for (unsigned int i = 0; i < dataLen; i++) {
        safeArray.getElement(i, data[i]);
    }
    safeArray.detach();
    return true;
}

/**
 * @brief Converts a dynamic C-style array to a safearray.
 *
 * @tparam T The type of elements in the array.
 * @param data Pointer to the C-style array to convert.
 * @param dataLen The length of the C-style array.
 * @param handle The safearray handle to store the result.
 * @return True if the conversion was successful, false otherwise.
 */
template <typename T>
bool afxArrayToSafeArray(const T *data, const unsigned int dataLen, safearray_handle *&handle) {
    CSafeArray<T> safeArray;
    if (handle != nullptr && safeArray.attach(handle) == false) {
        return false;
    }
    if (safeArray.resize(dataLen) == true) {
        for (unsigned int i = 0; i < dataLen; i++) {
            safeArray.setElement(i, data[i]);
        }
    }
    handle = safeArray.detach();
    return true;
}

#endif
