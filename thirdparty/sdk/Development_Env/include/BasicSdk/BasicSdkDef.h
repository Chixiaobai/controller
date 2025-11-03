#pragma once

#if defined _WIN32
#ifdef BasicSdk_EXPORTS
#define BASICSDK_API extern "C" __declspec(dllexport)
#define BASICSDK_CLASS __declspec(dllexport)
#else
#define BASICSDK_API extern "C" __declspec(dllimport)
#define BASICSDK_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define BASICSDK_API extern "C" __attribute__((visibility("default")))
#define BASICSDK_CLASS __attribute__((visibility("default")))
#else
#define BASICSDK_API extern "C"
#define BASICSDK_CLASS
#endif
#endif
