#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
#if defined(RPCCLIENTSUPPORT_LIB)
#define RPCCLIENTSUPPORT_EXPORT Q_DECL_EXPORT
#else
#define RPCCLIENTSUPPORT_EXPORT Q_DECL_IMPORT
#endif
#else
#define RPCCLIENTSUPPORT_EXPORT
#endif

interface IRpcClient;
interface IRpcClientManager;
interface IRpcClientObjectCluster;
interface IRpcCallback;
extern "C" {
RPCCLIENTSUPPORT_EXPORT bool installRpcEnvironment(const char *strRemoteIPAddress, const unsigned short uiRemotePort,
                                                   const char *strProductUuid = nullptr);
RPCCLIENTSUPPORT_EXPORT bool fetchHandshakeInfo(char *strHandshakeInfo, unsigned int *uintBuflen);
RPCCLIENTSUPPORT_EXPORT bool supportRpcServant(USHORT uiRpcServantId);
RPCCLIENTSUPPORT_EXPORT IRpcClientManager *getRpcClientManager();
RPCCLIENTSUPPORT_EXPORT void setRpcCallback(IRpcCallback *pRpcCallback);
RPCCLIENTSUPPORT_EXPORT void uninstallRpcEnvironment();
RPCCLIENTSUPPORT_EXPORT bool setRpcTraceOptions(unsigned int options);
RPCCLIENTSUPPORT_EXPORT bool setRpcAliveCounter(unsigned int uintCount);
}
