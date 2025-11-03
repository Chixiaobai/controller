#pragma once

#include "IHardwareCore.h"
#include "IHardwareLink.h"
#include "IHardwareEditor.h"
#include "IHardwareListener.h"
#include "IHardwareManagerInner.h"
#include "ComponentStub.h"

#define DECLARE_UNIQUE_HARDWAREMANAGER_STUB() DECLARE_UNIQUE_COMPONENT_STUB(HardwareManager)
#define IMPLEMENT_UNIQUE_HARDWAREMANAGER_STUB() \
    IMPLEMENT_UNIQUE_COMPONENT_STUB(HardwareManager, UNIQUE_HARDWAREMANAGER_NAME)
