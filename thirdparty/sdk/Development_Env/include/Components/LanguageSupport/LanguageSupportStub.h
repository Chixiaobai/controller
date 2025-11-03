#pragma once

#include "ComponentStub.h"
#include "ILanguageSupportInner.h"

#define DECLARE_UNIQUE_LANGUAGESUPPORT_STUB()                                                                    \
    class CLanguageSupport {                                                                                     \
    public:                                                                                                      \
        static bool installResource(IResourceContext *pResourceContext);                                         \
        static ILanguageSupportInner *getResource();                                                             \
        static void uninstallResource();                                                                         \
        static bool createLanguageTranslator(const string &strTranslatorFolder, const string &strTranslatorName, \
                                             const string &strParentAbsoluteTranslatorName);                     \
        static string translate(const string &strLanguageTable, const string &strSource);                        \
                                                                                                                 \
    protected:                                                                                                   \
        static ILanguageSupportInner *m_pLanguageSupport;                                                        \
        static map<string, ILanguageTranslator *> m_CacheLanguageTranslatorMap;                                  \
        static unsigned long m_ulResourceRef;                                                                    \
    };

#define IMPLEMENT_UNIQUE_LANGUAGESUPPORT_STUB()                                                                    \
    ILanguageSupportInner *CLanguageSupport::m_pLanguageSupport = nullptr;                                         \
    map<string, ILanguageTranslator *> CLanguageSupport::m_CacheLanguageTranslatorMap;                             \
    unsigned long CLanguageSupport::m_ulResourceRef = 0;                                                           \
    bool CLanguageSupport::installResource(IResourceContext *pResourceContext) {                                   \
        if (m_pLanguageSupport == nullptr) {                                                                       \
            IComponentLookup *pComponentLookup = nullptr;                                                          \
            if (pResourceContext->queryInterface("IComponentLookup", (void **)&pComponentLookup) == true) {        \
                IComponent *pComponent = pComponentLookup->findComponent(UNIQUE_LANGUAGESUPPORT_NAME, nullptr);    \
                if (pComponent != nullptr) {                                                                       \
                    pComponent->queryInterface("ILanguageSupportInner", (void **)&m_pLanguageSupport);             \
                    if (m_pLanguageSupport != nullptr) m_ulResourceRef = 1;                                        \
                    pComponent->releaseRef();                                                                      \
                }                                                                                                  \
                pComponentLookup->releaseRef();                                                                    \
            }                                                                                                      \
        } else                                                                                                     \
            OS::Thread::syncFetchAdd(&m_ulResourceRef);                                                            \
        return m_pLanguageSupport != nullptr;                                                                      \
    }                                                                                                              \
    ILanguageSupportInner *CLanguageSupport::getResource() {                                                       \
        if (m_pLanguageSupport != nullptr) m_pLanguageSupport->addRef();                                           \
        return m_pLanguageSupport;                                                                                 \
    }                                                                                                              \
    void CLanguageSupport::uninstallResource() {                                                                   \
        if (m_pLanguageSupport != nullptr) {                                                                       \
            map<string, ILanguageTranslator *>::iterator it = m_CacheLanguageTranslatorMap.begin();                \
            while (it != m_CacheLanguageTranslatorMap.end()) {                                                     \
                it->second->releaseRef();                                                                          \
                it->second = nullptr;                                                                              \
                it++;                                                                                              \
            }                                                                                                      \
            m_CacheLanguageTranslatorMap.clear();                                                                  \
            if (OS::Thread::syncFetchSub(&m_ulResourceRef) == 0) {                                                 \
                m_pLanguageSupport->releaseRef();                                                                  \
                m_pLanguageSupport = nullptr;                                                                      \
            }                                                                                                      \
        }                                                                                                          \
    }                                                                                                              \
    bool CLanguageSupport::createLanguageTranslator(const string &strTranslatorFolder,                             \
                                                    const string &strTranslatorName,                               \
                                                    const string &strParentAbsoluteTranslatorName) {               \
        if (m_pLanguageSupport != nullptr) {                                                                       \
            string strRootLanguageDirectory =                                                                      \
                CLanguageSupportHelper::getLanagueDirectory(m_pLanguageSupport) + PATH_SPLIT_STRING;               \
            return m_pLanguageSupport->createLanguageTranslator(                                                   \
                (strRootLanguageDirectory + strTranslatorFolder).c_str(), strTranslatorName.c_str(),               \
                strParentAbsoluteTranslatorName.c_str());                                                          \
        } else                                                                                                     \
            return false;                                                                                          \
    }                                                                                                              \
    string CLanguageSupport::translate(const string &strLanguageTable, const string &strSource) {                  \
        if (m_pLanguageSupport != nullptr) {                                                                       \
            map<string, ILanguageTranslator *>::iterator it = m_CacheLanguageTranslatorMap.find(strLanguageTable); \
            if (it != m_CacheLanguageTranslatorMap.end()) {                                                        \
                return CLanguageTranslatorHelper::translate(it->second, strSource.c_str());                        \
            } else {                                                                                               \
                ILanguageTranslator *pLanguageTranslator =                                                         \
                    m_pLanguageSupport->findLanguageTranslator(strLanguageTable.c_str());                          \
                if (pLanguageTranslator != nullptr) {                                                              \
                    m_CacheLanguageTranslatorMap.insert(                                                           \
                        pair<string, ILanguageTranslator *>(strLanguageTable, pLanguageTranslator));               \
                    return CLanguageTranslatorHelper::translate(pLanguageTranslator, strSource.c_str());           \
                } else                                                                                             \
                    return strSource;                                                                              \
            }                                                                                                      \
        } else                                                                                                     \
            return strSource;                                                                                      \
    }
