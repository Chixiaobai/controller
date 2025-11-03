#pragma once

#if defined _WIN32
#ifdef HardwareProxySupport_EXPORTS
#define HARDWAREPROXYSUPPORT_API extern "C" __declspec(dllexport)
#define HARDWAREPROXYSUPPORT_CLASS __declspec(dllexport)
#else
#define HARDWAREPROXYSUPPORT_API extern "C" __declspec(dllimport)
#define HARDWAREPROXYSUPPORT_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define HARDWAREPROXYSUPPORT_API extern "C" __attribute__((visibility("default")))
#define HARDWAREPROXYSUPPORT_CLASS __attribute__((visibility("default")))
#else
#define HARDWAREPROXYSUPPORT_API extern "C"
#define HARDWAREPROXYSUPPORT_CLASS
#endif
#endif
