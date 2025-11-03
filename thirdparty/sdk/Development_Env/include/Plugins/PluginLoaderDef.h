#pragma once

#if defined _WIN32
#ifdef PluginLoader_EXPORTS
#define PLUGINLOADER_API extern "C" __declspec(dllexport)
#define PLUGINLOADER_CLASS __declspec(dllexport)
#else
#define PLUGINLOADER_API extern "C" __declspec(dllimport)
#define PLUGINLOADER_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define PLUGINLOADER_API extern "C" __attribute__((visibility("default")))
#define PLUGINLOADER_CLASS __attribute__((visibility("default")))
#else
#define PLUGINLOADER_API extern "C"
#define PLUGINLOADER_CLASS
#endif
#endif
