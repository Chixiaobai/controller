#pragma once

#include "IPluginLoader.h"
#include "PluginLoaderDef.h"

PLUGINLOADER_API IPluginLoader *createPluginLoader();

typedef IPluginLoader *(*CREATE_PLUGIN_LOADER_FUNC)();  //导出函数名称:createPlugin

class CPluginLoader {
public:
    CPluginLoader() {
        string strPluginLoaderLibraryName = "PluginLoader";
#ifdef _DEBUG
        m_strPostFix = "d";
#endif
        string strPluginLoaderLibrary =
            OS_LIBRARY_PREFIX + strPluginLoaderLibraryName + m_strPostFix + "." OS_DYNAMIC_LIBRARY_SUFFIX;
        m_pPluginLoader = nullptr;
        m_hLoaderHandle = OS::DynamicLibraryLoader::load(strPluginLoaderLibrary.c_str());
        if (m_hLoaderHandle != (OS_MODULE_HANDLE)NULL) {
            CREATE_PLUGIN_LOADER_FUNC creatorFunc =
                (CREATE_PLUGIN_LOADER_FUNC)OS::DynamicLibraryLoader::getProc(m_hLoaderHandle, "createPluginLoader");
            if (creatorFunc != nullptr) {
                m_pPluginLoader = (*creatorFunc)();
            }
        } else {
            printf("[Error] Can not found PluginLoader library!\r\n");
        }
    }
    ~CPluginLoader() {
        if (m_pPluginLoader != nullptr) {
            m_pPluginLoader->releaseRef();
            m_pPluginLoader = nullptr;
        }
        if (m_hLoaderHandle != (OS_MODULE_HANDLE)NULL) {
            OS::DynamicLibraryLoader::free(m_hLoaderHandle);
            m_hLoaderHandle = (OS_MODULE_HANDLE)NULL;
        }
    }

public:
    INameEnum *createPluginManifest(const char *strPluginManifestFile, const char *strPostFix,
                                    IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return nullptr;
        return m_pPluginLoader->createPluginManifest(strPluginManifestFile, strPostFix, callback);
    }
    bool createPlugin(const char *strPluginClassName, const char *strPluginName, const char *strPostFix,
                      IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return false;
        return m_pPluginLoader->createPlugin(strPluginClassName, strPostFix, strPluginName, callback);
    }
    IPlugin *findPlugin(const char *strPluginClassName, const char *strPluginName) {
        if (m_pPluginLoader == nullptr) return nullptr;
        return m_pPluginLoader->findPlugin(strPluginClassName, strPluginName);
    }
    bool createUniquePlugin(const char *strPluginClassName, const char *strPostFix, IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return false;
        return m_pPluginLoader->createUniquePlugin(strPluginClassName, strPostFix, callback);
    }
    IPlugin *findUniquePlugin(const char *strPluginClassName) {
        if (m_pPluginLoader == nullptr) return nullptr;
        return m_pPluginLoader->findUniquePlugin(strPluginClassName);
    }
    bool destroyPlugin(const char *strPluginClassName, const char *strPluginName, IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return false;
        return m_pPluginLoader->destroyPlugin(strPluginClassName, strPluginName, callback);
    }
    bool destroyUniquePlugin(const char *strPluginClassName, IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return false;
        return m_pPluginLoader->destroyUniquePlugin(strPluginClassName, callback);
    }
    bool destroyAllPlugin(const char *strPluginClassName, IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return false;
        return m_pPluginLoader->destroyAllPlugin(strPluginClassName, callback);
    }
    void destroyAllPlugin(IPluginLoaderCallback *callback) {
        if (m_pPluginLoader == nullptr) return;
        return m_pPluginLoader->destroyAllPlugin(callback);
    }

protected:
    string m_strPostFix;
    OS_MODULE_HANDLE m_hLoaderHandle;
    IPluginLoader *m_pPluginLoader;
};
