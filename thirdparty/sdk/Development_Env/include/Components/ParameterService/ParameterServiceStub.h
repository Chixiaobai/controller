#pragma once

#include "IParameterServiceInner.h"
#include "ComponentStub.h"

#define DECLARE_UNIQUE_PARAMETERSERVICE_STUB() DECLARE_UNIQUE_COMPONENT_STUB(ParameterService)
#define IMPLEMENT_UNIQUE_PARAMETERSERVICE_STUB() \
    IMPLEMENT_UNIQUE_COMPONENT_STUB(ParameterService, UNIQUE_PARAMETERSERVICE_NAME)
