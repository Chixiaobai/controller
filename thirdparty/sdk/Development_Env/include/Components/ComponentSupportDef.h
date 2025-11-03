#pragma once

#if defined _WIN32
#ifdef ComponentSupport_EXPORTS
#define COMPONENTSUPPORT_API extern "C" __declspec(dllexport)
#define COMPONENTSUPPORT_CLASS __declspec(dllexport)
#else
#define COMPONENTSUPPORT_API extern "C" __declspec(dllimport)
#define COMPONENTSUPPORT_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define COMPONENTSUPPORT_API extern "C" __attribute__((visibility("default")))
#define COMPONENTSUPPORT_CLASS __attribute__((visibility("default")))
#else
#define COMPONENTSUPPORT_API extern "C"
#define COMPONENTSUPPORT_CLASS
#endif
#endif
