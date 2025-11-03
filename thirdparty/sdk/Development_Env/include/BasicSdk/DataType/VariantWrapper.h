/**
 * @file VariantWrapper.h
 * @brief Header file for the CVariantWrapper class and related functions.
 */

#ifndef __BASICSDK_VARIANT_WRAPPER_H__
#define __BASICSDK_VARIANT_WRAPPER_H__

#include "BasicSdkDef.h"
#include "DataType/IVariant.h"
#include "DataType/Variant.h"

/**
 * @brief Opaque handle for the variant wrapper.
 */
struct variant_wrapper_handle;

/**
 * @brief Creates a new variant wrapper handle.
 *
 * @return A pointer to the newly created variant wrapper handle.
 */
BASICSDK_API variant_wrapper_handle *variant_wrapper_create();

/**
 * @brief Destroys a variant wrapper handle.
 *
 * @param handle A reference to the variant wrapper handle to destroy.
 */
BASICSDK_API void variant_wrapper_destroy(variant_wrapper_handle *&handle);

/**
 * @brief Sets the read and write mode for the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @param bAllowRead Boolean indicating whether reading is allowed.
 * @param bAllowWrite Boolean indicating whether writing is allowed.
 */
BASICSDK_API void variant_wrapper_set_mode(variant_wrapper_handle *handle, bool bAllowRead, bool bAllowWrite);

/**
 * @brief Sets the wrapper value for the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @param wrapperVar A pointer to the TVARIANT to set as the wrapper value.
 * @param bCopyVar Boolean indicating whether to copy the TVARIANT (default is true).
 */
BASICSDK_API void variant_wrapper_set_wrapper_value(variant_wrapper_handle *handle, TVARIANT *wrapperVar,
                                                    bool bCopyVar = true);

/**
 * @brief Gets the wrapper value from the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @return A pointer to the TVARIANT representing the wrapper value.
 */
BASICSDK_API TVARIANT *variant_wrapper_get_wrapper_value(variant_wrapper_handle *handle);

/**
 * @brief Gets the type of the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @param pType A buffer to store the type string.
 * @param dwLen A reference to the length of the buffer.
 * @return True if the operation succeeds, false otherwise.
 */
BASICSDK_API bool variant_wrapper_get_type(variant_wrapper_handle *handle, char *pType, DWORD &dwLen);

/**
 * @brief Sets the type of the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @param pType A pointer to the type string to set.
 * @return True if the operation succeeds, false otherwise.
 */
BASICSDK_API bool variant_wrapper_set_type(variant_wrapper_handle *handle, const char *pType);

/**
 * @brief Gets the IVariantRead interface from the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @return A pointer to the IVariantRead interface.
 */
BASICSDK_API IVariantRead *variant_wrapper_get_variantread(variant_wrapper_handle *handle);

/**
 * @brief Gets the IVariantWrite interface from the variant wrapper.
 *
 * @param handle The variant wrapper handle.
 * @return A pointer to the IVariantWrite interface.
 */
BASICSDK_API IVariantWrite *variant_wrapper_get_variantwrite(variant_wrapper_handle *handle);

/**
 * @brief Clones a variant wrapper handle.
 *
 * @param handle The variant wrapper handle to clone.
 * @return A pointer to the cloned variant wrapper handle.
 */
BASICSDK_API variant_wrapper_handle *variant_wrapper_clone(variant_wrapper_handle *handle);

/**
 * @brief Declares the IVariantWrapperStub interface, which inherits from IVariant.
 */
DECLARE_STUB_INTERFACE(IVariantWrapperStub, IVariant)

/**
 * @brief Class representing a variant wrapper.
 *
 * This class provides a high-level interface for managing variant wrappers, including setting and getting values,
 * managing read/write modes, and cloning the wrapper.
 */
class CVariantWrapper : public IVariantWrapperStub {
protected:
    /**
     * @brief Default constructor for CVariantWrapper.
     *
     * Creates a new variant wrapper handle.
     */
    CVariantWrapper() { m_pVariantWrapperHandle = variant_wrapper_create(); }

    /**
     * @brief Copy constructor for CVariantWrapper.
     *
     * Clones the variant wrapper handle from another CVariantWrapper object.
     *
     * @param varWrapper The CVariantWrapper object to copy from.
     */
    CVariantWrapper(const CVariantWrapper &varWrapper) {
        m_pVariantWrapperHandle = variant_wrapper_clone(varWrapper.m_pVariantWrapperHandle);
    }

    /**
     * @brief Destructor for CVariantWrapper.
     *
     * Destroys the variant wrapper handle.
     */
    virtual ~CVariantWrapper() { variant_wrapper_destroy(m_pVariantWrapperHandle); }

public:
    /**
     * @brief Assignment operator for CVariantWrapper.
     *
     * Clones the variant wrapper handle from another CVariantWrapper object.
     *
     * @param varWrapper The CVariantWrapper object to assign from.
     * @return A reference to the current CVariantWrapper object.
     */
    CVariantWrapper &operator=(const CVariantWrapper &varWrapper) {
        variant_wrapper_destroy(m_pVariantWrapperHandle);
        m_pVariantWrapperHandle = nullptr;
        m_pVariantWrapperHandle = variant_wrapper_clone(varWrapper.m_pVariantWrapperHandle);
        return *this;
    }

public:
    /**
     * @brief Sets the read and write mode for the variant wrapper.
     *
     * @param bAllowRead Boolean indicating whether reading is allowed.
     * @param bAllowWrite Boolean indicating whether writing is allowed.
     */
    void setMode(bool bAllowRead, bool bAllowWrite) {
        variant_wrapper_set_mode(m_pVariantWrapperHandle, bAllowRead, bAllowWrite);
    }

    /**
     * @brief Sets the wrapper value for the variant wrapper.
     *
     * @param wrapperVar A pointer to the CVariant to set as the wrapper value.
     * @param bCopyVar Boolean indicating whether to copy the CVariant (default is true).
     */
    void setWrapperValue(CVariant *wrapperVar, bool bCopyVar = true) {
        variant_wrapper_set_wrapper_value(m_pVariantWrapperHandle, wrapperVar, bCopyVar);
    }

    /**
     * @brief Gets the wrapper value from the variant wrapper.
     *
     * @return A pointer to the TVARIANT representing the wrapper value.
     */
    TVARIANT *getWrapperValue() { return variant_wrapper_get_wrapper_value(m_pVariantWrapperHandle); }

    /**
     * @brief Clones the current CVariantWrapper object.
     *
     * @return A pointer to the cloned CVariantWrapper object.
     */
    CVariantWrapper *cloneWrapper() { return new CVariantWrapper(*this); }

public:
    /**
     * @brief Gets the type of the variant wrapper.
     *
     * @param pType A buffer to store the type string.
     * @param dwLen A reference to the length of the buffer.
     * @return True if the operation succeeds, false otherwise.
     */
    virtual bool getType(char *pType, DWORD &dwLen) {
        return variant_wrapper_get_type(m_pVariantWrapperHandle, pType, dwLen);
    }

    /**
     * @brief Sets the type of the variant wrapper.
     *
     * @param pType A pointer to the type string to set.
     * @return True if the operation succeeds, false otherwise.
     */
    virtual bool setType(const char *pType) { return variant_wrapper_set_type(m_pVariantWrapperHandle, pType); }

    /**
     * @brief Gets the IVariantRead interface from the variant wrapper.
     *
     * @return A pointer to the IVariantRead interface.
     */
    virtual IVariantRead *getVariantRead() { return variant_wrapper_get_variantread(m_pVariantWrapperHandle); }

    /**
     * @brief Gets the IVariantWrite interface from the variant wrapper.
     *
     * @return A pointer to the IVariantWrite interface.
     */
    virtual IVariantWrite *getVariantWrite() { return variant_wrapper_get_variantwrite(m_pVariantWrapperHandle); }

    /**
     * @brief Clones the current CVariantWrapper object.
     *
     * @return A pointer to the cloned IVariant object.
     */
    virtual IVariant *clone() { return cloneWrapper(); }

public:
    /**
     * @brief Static method to create a new CVariantWrapper object.
     *
     * @return A pointer to the newly created CVariantWrapper object.
     */
    static CVariantWrapper *createVariantWrapper() { return new CVariantWrapper(); }

protected:
    variant_wrapper_handle *m_pVariantWrapperHandle;  ///< Handle to the variant wrapper.
};

#endif  // __BASICSDK_VARIANT_WRAPPER_H__