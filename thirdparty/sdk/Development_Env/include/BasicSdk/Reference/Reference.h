/**
 * @file Reference.h
 * @brief Provides a reference counting mechanism for components.
 *
 * This header file defines a template class `CReference` that encapsulates reference counting for components.
 * All components must derive from the `IBase` interface, which should provide `addRef` and `releaseRef` methods.
 */

#ifndef __BASICSDK_REFERENCE_H__
#define __BASICSDK_REFERENCE_H__

#ifndef ULONG
/**
 * @brief Unsigned long integer type.
 */
typedef unsigned long ULONG;
#endif

/**
 * @brief Template class for reference counting.
 *
 * This class manages the reference count of objects that derive from the `IBase` interface.
 * It ensures that objects are properly released when no longer in use.
 *
 * @tparam T The type of the object being referenced, which must derive from `IBase`.
 */
template <class T>
class CReference {
public:
    /**
     * @brief Default constructor.
     */
    CReference() { m_pObject = nullptr; }

    /**
     * @brief Constructor that initializes the reference with an object.
     *
     * @param pObject Pointer to the object to be referenced.
     */
    CReference(T *pObject) { m_pObject = pObject; }

    /**
     * @brief Copy constructor.
     *
     * Increments the reference count of the object if it is not null.
     *
     * @param ref Reference to another `CReference` object.
     */
    CReference(const CReference &ref) {
        if (ref.m_pObject != nullptr) ref.m_pObject->addRef();
        m_pObject = ref.m_pObject;
    }

    /**
     * @brief Destructor.
     *
     * Decrements the reference count of the object and releases it if the count reaches zero.
     */
    ~CReference() {
        if (m_pObject != nullptr) {
            m_pObject->releaseRef();
            m_pObject = nullptr;
        }
    }

public:
    /**
     * @brief Assignment operator for raw pointer.
     *
     * Decrements the reference count of the current object and assigns a new object.
     *
     * @param pObject Pointer to the new object to be referenced.
     * @return Reference to the current `CReference` object.
     */
    const CReference &operator=(T *pObject) {
        if (m_pObject != nullptr) m_pObject->releaseRef();
        m_pObject = pObject;
        return *this;
    }

    /**
     * @brief Assignment operator for another `CReference` object.
     *
     * Increments the reference count of the new object and decrements the reference count of the current object.
     *
     * @param ref Reference to another `CReference` object.
     * @return Reference to the current `CReference` object.
     */
    const CReference &operator=(const CReference &ref) {
        if (ref.m_pObject != nullptr) ref.m_pObject->addRef();
        if (m_pObject != nullptr) m_pObject->releaseRef();
        m_pObject = ref.m_pObject;
        return *this;
    }

    /**
     * @brief Checks if the reference is null.
     *
     * @return True if the reference is null, false otherwise.
     */
    bool nil() const noexcept { return m_pObject == nullptr; }

    /**
     * @brief Releases the reference and returns the new reference count.
     *
     * Decrements the reference count of the object and releases it if the count reaches zero.
     *
     * @return The new reference count after releasing.
     */
    ULONG release() {
        ULONG dwRef = 0;
        if (m_pObject != nullptr) {
            T *pObject =
                m_pObject;  // When the object reference is recorded in a table, and the object destructor automatically
                            // clears the table, the object's destructor may cause the reference-occupied memory
                            // m_pObject to be released, thus m_pObject cannot be accessed.
            m_pObject = nullptr;  // Set the object to null first, must do this, because once the object is destroyed,
                                  // the reference may become invalid, thus m_pObject cannot be accessed.
            dwRef = pObject->releaseRef();  // Release the object, at this point the reference can be safely deleted.
            pObject = nullptr;
        }
        return dwRef;
    }

    /**
     * @brief Arrow operator to access members of the referenced object.
     *
     * @return Pointer to the referenced object.
     */
    T *operator->() const noexcept { return m_pObject; }

    /**
     * @brief Conversion operator to raw pointer.
     *
     * @return Pointer to the referenced object.
     */
    operator T *() const noexcept { return m_pObject; }

protected:
    /**
     * @brief Pointer to the referenced object.
     */
    T *m_pObject;
};

#endif