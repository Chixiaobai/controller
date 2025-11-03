#pragma once

#include "IComponentSupport.h"
#include "ComponentSupportDef.h"

COMPONENTSUPPORT_API bool componentSupportLayerInitialize(IBase *pBaseOuter, IBase *&pInnerResourceContext);
COMPONENTSUPPORT_API IResourceContext *getComponentContext();
COMPONENTSUPPORT_API IComponentSupport *getComponentSupport();
COMPONENTSUPPORT_API void componentSupportLayerUninitialize();
