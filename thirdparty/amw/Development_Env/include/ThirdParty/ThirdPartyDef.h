#pragma once

#if defined _WIN32 
#ifdef THIRDPARTY_EXPORTS
#define THIRDPARTY_API extern "C" __declspec(dllexport)
#define THIRDPARTY_CLASS __declspec(dllexport)
#else   
#define THIRDPARTY_API extern "C" __declspec(dllimport)
#define THIRDPARTY_CLASS __declspec(dllimport)
#endif  
#else
#if __GNUC__ >= 4
#define THIRDPARTY_API extern "C" __attribute__ ((visibility("default")))
#define THIRDPARTY_CLASS __attribute__ ((visibility("default")))
#else   
#define THIRDPARTY_API extern "C"
#define THIRDPARTY_CLASS
#endif  
#endif
