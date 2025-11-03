#pragma once

#if defined _WIN32
#ifdef AMWTOOL_EXPORTS
#define AMWTOOL_API extern "C" __declspec(dllexport)
#define AMWTOOL_CLASS __declspec(dllexport)
#else
#define AMWTOOL_API extern "C" __declspec(dllimport)
#define AMWTOOL_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define AMWTOOL_API extern "C" __attribute__((visibility("default")))
#define AMWTOOL_CLASS __attribute__((visibility("default")))
#else
#define AMWTOOL_API extern "C"
#define AMWTOOL_CLASS
#endif
#endif
