#ifndef __BASICSDK_I_BASE_H__
#define __BASICSDK_I_BASE_H__


/**
 * @brief Base interface for all interfaces
 */
interface IBase {
    /**
     * @brief Query for a specific interface
     *
     * @param name Name of the interface to query
     * @param pInterface Pointer to the interface pointer to be set
     * @return bool True if the interface is found and set, false otherwise
     */
    virtual bool queryInterface(const char *name, void **pInterface) = 0;

    /**
     * @brief Increment the reference count
     *
     * @return ULONG New reference count
     */
    virtual ULONG addRef() = 0;

    /**
     * @brief Decrement the reference count
     *
     * @return ULONG New reference count
     */
    virtual ULONG releaseRef() = 0;
};

/**
 * @brief Macro to declare a stub interface
 *
 * @param stubclassname Name of the stub class
 * @param interfacename Name of the interface
 */
#define DECLARE_STUB_INTERFACE(stubclassname, interfacename)                           \
    class stubclassname : public interfacename {                                       \
    public:                                                                            \
        stubclassname() { m_dwRef = 1; }                                               \
        virtual ~stubclassname() {}                                                    \
                                                                                       \
    public:                                                                            \
        virtual bool queryInterface(const char *name, void **pInterface) override {    \
            if (strcmp(name, "IBase") == 0) {                                          \
                *pInterface = (IBase *)(interfacename *)this;                          \
                addRef();                                                              \
                return true;                                                           \
            } else if (strcmp(name, #interfacename) == 0) {                            \
                *pInterface = (interfacename *)this;                                   \
                addRef();                                                              \
                return true;                                                           \
            } else                                                                     \
                return false;                                                          \
        }                                                                              \
        virtual ULONG addRef() override { return OS::Thread::syncFetchAdd(&m_dwRef); } \
        virtual ULONG releaseRef() override {                                          \
            ULONG dwRef = OS::Thread::syncFetchSub(&m_dwRef);                          \
            if (dwRef == 0) {                                                          \
                onFinalRelease();                                                      \
                delete this;                                                           \
            }                                                                          \
            return dwRef;                                                              \
        }                                                                              \
        virtual void onFinalRelease() {}                                               \
                                                                                       \
    protected:                                                                         \
        ULONG m_dwRef;                                                                 \
    };

/**
 * @brief Macro to declare a stub class that inherits from a base class and implements an interface
 *
 * @param stubclassname Name of the stub class
 * @param baseclassname Name of the base class
 * @param interfacename Name of the interface
 */
#define DECLARE_STUB_CLASS(stubclassname, baseclassname, interfacename)             \
    class stubclassname : public baseclassname, public interfacename {              \
    public:                                                                         \
        virtual bool queryInterface(const char *name, void **pInterface) override { \
            if (strcmp(name, #interfacename) == 0) {                                \
                *pInterface = (interfacename *)this;                                \
                addRef();                                                           \
                return true;                                                        \
            } else                                                                  \
                return baseclassname::queryInterface(name, pInterface);             \
        }                                                                           \
        virtual ULONG addRef() override { return baseclassname::addRef(); }         \
        virtual ULONG releaseRef() override { return baseclassname::releaseRef(); } \
    };

/**
 * @brief Macro to declare a DLL stub interface
 *
 * @param decorate Decorator for the class (e.g., __declspec(dllexport))
 * @param stubclassname Name of the stub class
 * @param interfacename Name of the interface
 */
#define DECLARE_DLL_STUB_INTERFACE(decorate, stubclassname, interfacename)             \
    class decorate stubclassname : public interfacename {                              \
    public:                                                                            \
        stubclassname() { m_dwRef = 1; }                                               \
        virtual ~stubclassname() {}                                                    \
                                                                                       \
    public:                                                                            \
        virtual bool queryInterface(const char *name, void **pInterface) override {    \
            if (strcmp(name, "IBase") == 0) {                                          \
                *pInterface = (IBase *)(interfacename *)this;                          \
                addRef();                                                              \
                return true;                                                           \
            } else if (strcmp(name, #interfacename) == 0) {                            \
                *pInterface = (interfacename *)this;                                   \
                addRef();                                                              \
                return true;                                                           \
            } else                                                                     \
                return false;                                                          \
        }                                                                              \
        virtual ULONG addRef() override { return OS::Thread::syncFetchAdd(&m_dwRef); } \
        virtual ULONG releaseRef() override {                                          \
            ULONG dwRef = OS::Thread::syncFetchSub(&m_dwRef);                          \
            if (dwRef == 0) {                                                          \
                onFinalRelease();                                                      \
                delete this;                                                           \
            }                                                                          \
            return dwRef;                                                              \
        }                                                                              \
        virtual void onFinalRelease() {}                                               \
                                                                                       \
    protected:                                                                         \
        ULONG m_dwRef;                                                                 \
    };

/**
 * @brief Macro to declare a DLL stub class that inherits from a base class and implements an interface
 *
 * @param decorate Decorator for the class (e.g., __declspec(dllexport))
 * @param stubclassname Name of the stub class
 * @param baseclassname Name of the base class
 * @param interfacename Name of the interface
 */
#define DECLARE_DLL_STUB_CLASS(decorate, stubclassname, baseclassname, interfacename) \
    class decorate stubclassname : public baseclassname, public interfacename {       \
    public:                                                                           \
        virtual bool queryInterface(const char *name, void **pInterface) override {   \
            if (strcmp(name, #interfacename) == 0) {                                  \
                *pInterface = (interfacename *)this;                                  \
                addRef();                                                             \
                return true;                                                          \
            } else                                                                    \
                return baseclassname::queryInterface(name, pInterface);               \
        }                                                                             \
        virtual ULONG addRef() override { return baseclassname::addRef(); }           \
        virtual ULONG releaseRef() override { return baseclassname::releaseRef(); }   \
    };

/**
 * @brief Macro to enable recognition of a specific interface
 *
 * @param interfacename Name of the interface
 */
#define ENABLE_RECOGNIZE_MACRO(interfacename)                                       \
    inline interfacename *as##interfacename(IBase *baseObject) {                    \
        if (baseObject != nullptr) {                                                \
            interfacename *p##interfacename = nullptr;                              \
            baseObject->queryInterface(#interfacename, (void **)&p##interfacename); \
            return p##interfacename;                                                \
        } else                                                                      \
            return nullptr;                                                         \
    }

/**
 * @brief Macro to cast an object to a specific interface
 *
 * @param object Pointer to the object
 * @param interfacename Name of the interface
 */
#define AS_INTERFACE(object, interfacename) as##interfacename(object)

#endif
