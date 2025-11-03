#ifndef __BASICSDK_OS_SYNC_DEF_H__
#define __BASICSDK_OS_SYNC_DEF_H__

#define MAX_MUTEX_NAME_LENGTH 1024

#ifndef _WIN32

#define MUTEX_MANAGE_REQUEST_SEMAPHORE_NAME "/os_mutex_manage_request_semaphore"
#define MUTEX_MANAGE_FINISH_SEMAPHORE_NAME "/os_mutex_manage_finish_semaphore"
#define MUTEX_MANAGER_ENABLE_SEMAPHORE_NAME "/os_mutex_manage_enabled_semaphore"
#define MUTEX_MANAGER_ENABLE_SHAREMEMORY_NAME "os_mutex_manage_enabled_sharemomory"

enum MUTEX_MANAGE_TYPE {
    MUTEX_CREATE_METHOD = 1,
    MUTEX_DESTROY_METHOD = 2,
};
#endif

#endif
