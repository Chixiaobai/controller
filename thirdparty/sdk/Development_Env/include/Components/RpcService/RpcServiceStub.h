#pragma once

#include "IRpcServiceInner.h"
#include "AbstractRpcServant.h"
#include "RpcServantDef.h"
#include "ComponentStub.h"

#define DECLARE_UNIQUE_RPCSERVICE_STUB() DECLARE_UNIQUE_COMPONENT_STUB(RpcService)
#define IMPLEMENT_UNIQUE_RPCSERVICE_STUB() IMPLEMENT_UNIQUE_COMPONENT_STUB(RpcService, UNIQUE_RPCSERVICE_NAME)
