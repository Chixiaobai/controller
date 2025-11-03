#ifndef __BASICSDK_BSTR_H__
#define __BASICSDK_BSTR_H__

#include "BasicSdkDef.h"

/**
 * @brief Opaque structure representing a BSTR handle.
 */
struct bstr_handle;

/**
 * @brief Creates a new BSTR handle.
 * @return A pointer to the created BSTR handle.
 */
BASICSDK_API bstr_handle *bstr_create();

/**
 * @brief Destroys a BSTR handle.
 * @param handle Reference to the BSTR handle to be destroyed.
 */
BASICSDK_API void bstr_destroy(bstr_handle *&handle);

/**
 * @brief Clears the content of a BSTR handle.
 * @param handle The BSTR handle.
 */
BASICSDK_API void bstr_clear(bstr_handle *handle);

/**
 * @brief Compares two BSTR handles for equality.
 * @param handle1 The first BSTR handle.
 * @param handle2 The second BSTR handle.
 * @return True if the BSTR handles are equal, false otherwise.
 */
BASICSDK_API bool bstr_equal_bstr(bstr_handle *handle1, bstr_handle *handle2);

/**
 * @brief Compares a BSTR handle with a BSTRUNIT for equality.
 * @param handle The BSTR handle.
 * @param var The BSTRUNIT to compare.
 * @return True if the BSTR handle and BSTRUNIT are equal, false otherwise.
 */
BASICSDK_API bool bstr_equal_bstrunit(bstr_handle *handle, LPBSTRUNIT var);

/**
 * @brief Sets the content of a BSTR handle from a BSTRUNIT.
 * @param handle The BSTR handle.
 * @param var The BSTRUNIT to set.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool bstr_set_bstrunit(bstr_handle *handle, LPBSTRUNIT var);

/**
 * @brief Sets the content of a BSTR handle from a null-terminated text string.
 * @param handle The BSTR handle.
 * @param text The null-terminated text string to set.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool bstr_set_text(bstr_handle *handle, const char *text);

/**
 * @brief Sets the content of a BSTR handle from a data buffer that may contain null characters.
 * @param handle The BSTR handle.
 * @param data The data buffer to set.
 * @param len The length of the data buffer.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool bstr_set_data(bstr_handle *handle, const char *data, int len);

/**
 * @brief Sets the content of a BSTR handle from another BSTR handle.
 * @param handle1 The BSTR handle to set.
 * @param handle2 The BSTR handle to copy from.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool bstr_set_bstr(bstr_handle *handle1, bstr_handle *handle2);

/**
 * @brief Retrieves the data from a BSTR handle.
 * @param handle The BSTR handle.
 * @return A pointer to the data buffer.
 */
BASICSDK_API const char *bstr_get_data(bstr_handle *handle);

/**
 * @brief Retrieves the length of the data in a BSTR handle.
 * @param handle The BSTR handle.
 * @return The length of the data.
 */
BASICSDK_API int bstr_get_length(bstr_handle *handle);

/**
 * @brief Attaches a BSTRUNIT to a BSTR handle.
 * @param handle The BSTR handle.
 * @param var The BSTRUNIT to attach.
 */
BASICSDK_API void bstr_attach_bstrunit(bstr_handle *handle, LPBSTRUNIT var);

/**
 * @brief Detaches a BSTRUNIT from a BSTR handle.
 * @param handle The BSTR handle.
 * @return The detached BSTRUNIT.
 */
BASICSDK_API LPBSTRUNIT bstr_detach_bstrunit(bstr_handle *handle);

/**
 * @brief Class representing a BSTR (Binary String).
 */
class CBStr {
public:
    /**
     * @brief Default constructor.
     * Initializes the BSTR handle.
     */
    CBStr() { m_hBstrHandle = bstr_create(); }

    /**
     * @brief Constructor from a BSTRUNIT.
     * Initializes the BSTR handle and sets its content from the BSTRUNIT.
     * @param var The BSTRUNIT to initialize from.
     */
    CBStr(LPBSTRUNIT var) {
        m_hBstrHandle = bstr_create();
        bstr_set_bstrunit(m_hBstrHandle, var);
    }

    /**
     * @brief Constructor from a null-terminated text string.
     * Initializes the BSTR handle and sets its content from the text string.
     * @param text The null-terminated text string to initialize from.
     */
    CBStr(const char *text) {
        m_hBstrHandle = bstr_create();
        bstr_set_text(m_hBstrHandle, text);
    }

    /**
     * @brief Constructor from a data buffer.
     * Initializes the BSTR handle and sets its content from the data buffer.
     * @param data The data buffer to initialize from.
     * @param len The length of the data buffer.
     */
    CBStr(const char *data, int len) {
        m_hBstrHandle = bstr_create();
        bstr_set_data(m_hBstrHandle, data, len);
    }

    /**
     * @brief Copy constructor.
     * Initializes the BSTR handle and sets its content from another CBStr object.
     * @param var The CBStr object to copy from.
     */
    CBStr(const CBStr &var) {
        m_hBstrHandle = bstr_create();
        bstr_set_bstr(m_hBstrHandle, var.m_hBstrHandle);
    }

    /**
     * @brief Destructor.
     * Clears and destroys the BSTR handle.
     */
    virtual ~CBStr() {
        bstr_clear(m_hBstrHandle);
        bstr_destroy(m_hBstrHandle);
    }

    /**
     * @brief Clears the content of the BSTR handle.
     */
    void clear() { bstr_clear(m_hBstrHandle); }

    /**
     * @brief Compares this CBStr object with another CBStr object for equality.
     * @param var The CBStr object to compare.
     * @return True if the CBStr objects are equal, false otherwise.
     */
    bool operator==(const CBStr &var) const { return bstr_equal_bstr(m_hBstrHandle, var.m_hBstrHandle); }

    /**
     * @brief Compares this CBStr object with a BSTRUNIT for equality.
     * @param var The BSTRUNIT to compare.
     * @return True if the CBStr object and BSTRUNIT are equal, false otherwise.
     */
    bool operator==(LPBSTRUNIT var) const { return bstr_equal_bstrunit(m_hBstrHandle, var); }

    /**
     * @brief Assignment operator from a BSTRUNIT.
     * Sets the content of the BSTR handle from the BSTRUNIT.
     * @param var The BSTRUNIT to assign from.
     * @return A reference to this CBStr object.
     */
    const CBStr &operator=(LPBSTRUNIT var) {
        bstr_set_bstrunit(m_hBstrHandle, var);
        return *this;
    }

    /**
     * @brief Assignment operator from a null-terminated text string.
     * Sets the content of the BSTR handle from the text string.
     * @param text The null-terminated text string to assign from.
     * @return A reference to this CBStr object.
     */
    const CBStr &operator=(const char *text) {
        bstr_set_text(m_hBstrHandle, text);
        return *this;
    }

    /**
     * @brief Assignment operator from another CBStr object.
     * Sets the content of the BSTR handle from another CBStr object.
     * @param var The CBStr object to assign from.
     * * @return A reference to this CBStr object.
     */
    const CBStr &operator=(const CBStr &var) {
        bstr_set_bstr(m_hBstrHandle, var.m_hBstrHandle);
        return *this;
    }

    /**
     * @brief Conversion operator to const char*.
     * Retrieves the data from the BSTR handle.
     * @return A pointer to the data buffer.
     */
    operator const char *() { return bstr_get_data(m_hBstrHandle); }

    /**
     * @brief Retrieves the length of the data in the BSTR handle.
     * @return The length of the data.
     */
    int get_length() { return bstr_get_length(m_hBstrHandle); }

    /**
     * @brief Attaches a BSTRUNIT to the BSTR handle.
     * @param bstrValue The BSTRUNIT to attach.
     */
    void attach(LPBSTRUNIT bstrValue) { bstr_attach_bstrunit(m_hBstrHandle, bstrValue); }

    /**
     * @brief Detaches a BSTRUNIT from the BSTR handle.
     * @return The detached BSTRUNIT.
     */
    LPBSTRUNIT detach() { return bstr_detach_bstrunit(m_hBstrHandle); }

protected:
    /**
     * @brief The BSTR handle.
     */
    bstr_handle *m_hBstrHandle;
};

#endif