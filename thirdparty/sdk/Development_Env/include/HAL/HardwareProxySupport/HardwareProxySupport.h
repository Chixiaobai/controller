#pragma once

#include "ModuleDef/IModuleVersion.h"
#include "HardwareProxySupport/HardwareProxySupportDef.h"
#include "HardwareProxySupport/IHardwareProxy.h"
#include "HardwareProxySupport/IHardwareProxyEditor.h"
#include "HardwareProxySupport/IHardwareProxyTable.h"
#include "HardwareProxySupport/IHardwareProxyManager.h"

HARDWAREPROXYSUPPORT_API bool hardware_proxy_support_initialize(LPVOID pvReserved);
HARDWAREPROXYSUPPORT_API void hardware_proxy_support_uninitialize();
HARDWAREPROXYSUPPORT_API IHardwareProxyManager *hardware_proxy_support_get_hardware_proxy_manager();
HARDWAREPROXYSUPPORT_API IModuleVersion *hardware_proxy_support_get_module_version();

class CHardwareProxySupport {
public:
    static bool initialize(LPVOID pvReserved) { return hardware_proxy_support_initialize(pvReserved); }
    static void uninitialize() { hardware_proxy_support_uninitialize(); }
    static IHardwareProxyManager *getHardwareProxyManager() {
        return hardware_proxy_support_get_hardware_proxy_manager();
    }
    static IModuleVersion *getVersion() { return hardware_proxy_support_get_module_version(); }
};
