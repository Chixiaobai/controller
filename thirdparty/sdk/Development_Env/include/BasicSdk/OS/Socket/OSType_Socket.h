#pragma once

/**
 * Define socket-related types and constants based on the operating system.
 * 
 * On _WIN32:
 * - OS_SOCKET_TYPE is defined as int.
 * - OS_SOCKET is defined as SOCKET.
 * - OS_SOCKET_LEN is defined as int.
 * 
 * On __APPLE__:
 * - OS_SOCKET_TYPE is defined as int.
 * - OS_SOCKET is defined as int.
 * - OS_SOCKET_LEN is defined as socklen_t.
 * - SOCKET_ERROR is defined as -1.
 * 
 * On other systems:
 * - OS_SOCKET_TYPE is defined as __socket_type.
 * - OS_SOCKET is defined as int.
 * - OS_SOCKET_LEN is defined as socklen_t.
 * - SOCKET_ERROR is defined as -1.
 */
#ifdef _WIN32
#define OS_SOCKET_TYPE int
#define OS_SOCKET SOCKET
#define OS_SOCKET_LEN int
#elif __APPLE__
#define OS_SOCKET_TYPE int
#define OS_SOCKET int
#define OS_SOCKET_LEN socklen_t
#define SOCKET_ERROR -1
#else
#define OS_SOCKET_TYPE __socket_type
#define OS_SOCKET int
#define OS_SOCKET_LEN socklen_t
#define SOCKET_ERROR -1
#endif

#ifdef _WIN32
#include "winsock2.h"
#include "WS2tcpip.h"
#ifndef __MINGW32__
#pragma comment(lib, "ws2_32.lib")
#endif
#else
#include "sys/socket.h"
#include "arpa/inet.h"
#include "sys/ioctl.h"
#endif
