#pragma once

#include "ApartmentErrorDef.h"

#if defined _WIN32
#ifdef APARTMENT_EXPORTS
#define APARTMENT_API extern "C" __declspec(dllexport)
#define APARTMENT_CLASS __declspec(dllexport)
#else
#define APARTMENT_API extern "C" __declspec(dllimport)
#define APARTMENT_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define APARTMENT_API extern "C" __attribute__((visibility("default")))
#define APARTMENT_CLASS __attribute__((visibility("default")))
#else
#define APARTMENT_API extern "C"
#define APARTMENT_CLASS
#endif
#endif
