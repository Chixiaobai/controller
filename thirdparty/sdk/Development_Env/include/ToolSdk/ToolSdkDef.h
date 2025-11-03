#pragma once

#if defined _WIN32
#ifdef ToolSdk_EXPORTS
#define TOOLSDK_API extern "C" __declspec(dllexport)
#define TOOLSDK_CLASS __declspec(dllexport)
#else
#define TOOLSDK_API extern "C" __declspec(dllimport)
#define TOOLSDK_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define TOOLSDK_API extern "C" __attribute__((visibility("default")))
#define TOOLSDK_CLASS __attribute__((visibility("default")))
#else
#define TOOLSDK_API extern "C"
#define TOOLSDK_CLASS
#endif
#endif
