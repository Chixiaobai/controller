#pragma once

#include "IResourceContext.h"
#include "OS/OS_Mutex.h"

#define DECLARE_UNIQUE_COMPONENT_STUB(classname)                         \
    class C##classname {                                                 \
    public:                                                              \
        static bool installResource(IResourceContext *pResourceContext); \
        static I##classname##Inner *getResource();                       \
        static void uninstallResource();                                 \
                                                                         \
    protected:                                                           \
        static I##classname##Inner *m_p##classname;                      \
        static unsigned long m_ulResourceRef;                            \
        static OS::TRecursiveMutex m_ResourceMutex;                      \
    };

#define IMPLEMENT_UNIQUE_COMPONENT_STUB(classname, displayname)                                             \
    I##classname##Inner *C##classname::m_p##classname = nullptr;                                            \
    unsigned long C##classname::m_ulResourceRef = 0;                                                        \
    OS::TRecursiveMutex C##classname::m_ResourceMutex;                                                      \
    bool C##classname::installResource(IResourceContext *pResourceContext) {                                \
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_ResourceMutex);                                         \
        if (m_p##classname == nullptr) {                                                                    \
            IComponentLookup *pComponentLookup = nullptr;                                                   \
            if (pResourceContext->queryInterface("IComponentLookup", (void **)&pComponentLookup) == true) { \
                IComponent *pComponent = pComponentLookup->findComponent(displayname, nullptr);             \
                if (pComponent != nullptr) {                                                                \
                    pComponent->queryInterface("I" #classname "Inner", (void **)&m_p##classname);           \
                    if (m_p##classname != nullptr) m_ulResourceRef = 1;                                     \
                    pComponent->releaseRef();                                                               \
                }                                                                                           \
                pComponentLookup->releaseRef();                                                             \
            }                                                                                               \
        } else                                                                                              \
            OS::Thread::syncFetchAdd(&m_ulResourceRef);                                                     \
        return m_p##classname != nullptr;                                                                   \
    }                                                                                                       \
    I##classname##Inner *C##classname::getResource() {                                                      \
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_ResourceMutex);                                         \
        if (m_p##classname != nullptr) m_p##classname->addRef();                                            \
        return m_p##classname;                                                                              \
    }                                                                                                       \
    void C##classname::uninstallResource() {                                                                \
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_ResourceMutex);                                         \
        if (m_p##classname != nullptr) {                                                                    \
            if (OS::Thread::syncFetchSub(&m_ulResourceRef) == 0) {                                          \
                m_p##classname->releaseRef();                                                               \
                m_p##classname = nullptr;                                                                   \
            }                                                                                               \
        }                                                                                                   \
    }
