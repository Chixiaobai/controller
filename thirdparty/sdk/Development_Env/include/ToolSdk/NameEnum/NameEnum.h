/**
 * @file NameEnum.h
 * @brief Header file for name enumeration management.
 *
 * This file defines functions, a stub interface, and a class for managing name enumerations.
 */

#ifndef __TOOLSDK_NAME_ENUM_H__
#define __TOOLSDK_NAME_ENUM_H__

#include "ToolSdkDef.h"
#include "NameEnum/INameEnum.h"

/**
 * @struct name_enumerator_handle
 * @brief Opaque handle for managing name enumerators.
 *
 * This structure represents an opaque handle used internally for managing name enumerators.
 */
struct name_enumerator_handle;

/**
 * @fn name_enumerator_handle *name_enumerator_create()
 * @brief Creates a new name enumerator.
 *
 * Creates a new name enumerator and returns a handle to it.
 *
 * @return A pointer to the newly created name enumerator handle, or nullptr if the operation failed.
 */
TOOLSDK_API name_enumerator_handle *name_enumerator_create();

/**
 * @fn void name_enumerator_destroy(name_enumerator_handle *&handle)
 * @brief Destroys a name enumerator.
 *
 * Destroys the specified name enumerator and frees its resources.
 *
 * @param handle A reference to the name enumerator handle to be destroyed.
 */
TOOLSDK_API void name_enumerator_destroy(name_enumerator_handle *&handle);

/**
 * @fn void name_enumerator_add(name_enumerator_handle *handle, const char *strName)
 * @brief Adds a name to the name enumerator.
 *
 * Adds a name to the specified name enumerator.
 *
 * @param handle The name enumerator handle.
 * @param strName The name to be added.
 */
TOOLSDK_API void name_enumerator_add(name_enumerator_handle *handle, const char *strName);

/**
 * @fn bool name_enumerator_current(name_enumerator_handle *handle, char *pName, unsigned long &ulSize)
 * @brief Retrieves the current name in the enumeration.
 *
 * Retrieves the current name in the enumeration and stores it in the provided buffer.
 *
 * @param handle The name enumerator handle.
 * @param pName Pointer to the buffer to store the current name.
 * @param ulSize Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool name_enumerator_current(name_enumerator_handle *handle, char *pName, unsigned long &ulSize);

/**
 * @fn bool name_enumerator_next(name_enumerator_handle *handle)
 * @brief Moves to the next name in the enumeration.
 *
 * Moves to the next name in the enumeration.
 *
 * @param handle The name enumerator handle.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool name_enumerator_next(name_enumerator_handle *handle);

/**
 * @fn void name_enumerator_reset_it(name_enumerator_handle *handle)
 * @brief Resets the enumeration iterator.
 *
 * Resets the iterator to the beginning of the enumeration.
 *
 * @param handle The name enumerator handle.
 */
TOOLSDK_API void name_enumerator_reset_it(name_enumerator_handle *handle);

/**
 * @brief Stub interface for INameEnum.
 */
DECLARE_STUB_INTERFACE(INameEnumStub, INameEnum)

/**
 * @class CNameEnum
 * @brief Class for managing name enumerations.
 *
 * This class provides methods for managing name enumerations and implements the INameEnumStub interface.
 */
class CNameEnum : public INameEnumStub {
protected:
    /**
     * @brief Constructor for CNameEnum.
     *
     * Initializes the name enumerator by creating a new handle.
     */
    CNameEnum() { m_pNameEnumeratorHandle = name_enumerator_create(); }

    /**
     * @brief Destructor for CNameEnum.
     *
     * Cleans up the name enumerator by destroying the handle.
     */
    virtual ~CNameEnum() { name_enumerator_destroy(m_pNameEnumeratorHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CNameEnum objects to ensure proper management of name enumerator handles.
     */
    CNameEnum(const CNameEnum &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CNameEnum objects to ensure proper management of name enumerator handles.
     */
    CNameEnum &operator=(const CNameEnum &) = delete;
#endif

public:
    /**
     * @brief Adds a name to the name enumerator.
     *
     * Adds a name to the name enumerator.
     *
     * @param strName The name to be added.
     */
    void addName(const string &strName) { name_enumerator_add(m_pNameEnumeratorHandle, strName.c_str()); }

public:
    /**
     * @brief Retrieves the current name in the enumeration.
     *
     * Retrieves the current name in the enumeration and stores it in the provided buffer.
     *
     * @param pName Pointer to the buffer to store the current name.
     * @param ulSize Reference to the length of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool currentName(char *pName, unsigned long &ulSize) {
        return name_enumerator_current(m_pNameEnumeratorHandle, pName, ulSize);
    }

    /**
     * @brief Moves to the next name in the enumeration.
     *
     * Moves to the next name in the enumeration.
     *
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool nextName() { return name_enumerator_next(m_pNameEnumeratorHandle); }

    /**
     * @brief Resets the enumeration iterator.
     *
     * Resets the iterator to the beginning of the enumeration.
     */
    virtual void resetIt() { name_enumerator_reset_it(m_pNameEnumeratorHandle); }

public:
    /**
     * @brief Creates an instance of CNameEnum.
     *
     * Creates a new instance of the CNameEnum class.
     *
     * @return Pointer to the newly created CNameEnum instance.
     */
    static CNameEnum *createInstance() { return new CNameEnum(); }

private:
    /**
     * @var name_enumerator_handle *m_pNameEnumeratorHandle
     * @brief Opaque handle for managing the name enumerator.
     *
     * This handle is used internally by the CNameEnum class to manage the name enumerator.
     */
    name_enumerator_handle *m_pNameEnumeratorHandle;
};

#endif  // __TOOLSDK_NAME_ENUM_H__