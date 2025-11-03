#ifndef __BASICSDK_OS_SYNC_H__
#define __BASICSDK_OS_SYNC_H__

#ifndef __APPLE__

#include "BasicSdkDef.h"
#include "OS/OS_Condition.h"
#include "OS/OS_Mutex.h"
#include "OS/OS_RWMutex.h"
#include "OS/OS_Event.h"
#include "OS/OS_Semaphore.h"

BASICSDK_API bool os_sync_environment_initialize(bool bAsClient = true);
BASICSDK_API void os_sync_environment_uninitialize();

#else


#include "OS/OS_Mutex.h"

#endif

#endif
