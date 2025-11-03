#ifndef __BASICSDK_OS_SYNC_CLIENT_H__
#define __BASICSDK_OS_SYNC_CLIENT_H__

#ifndef _WIN32
#include "semaphore.h"
struct semid_ds;
struct seminfo;
union semun {
    int val;
    struct semid_ds *buf;
    unsigned short int *array;
    struct seminfo *__buf;
};

typedef union _sem_handle {
    sem_t *m_hAnonymousHandle;  //适用于进程内的Posix匿名控制信号
    int *m_hNamedHandle;        //适用于进程内的System V信号，可防止进程异常退出导致的死锁
} sem_handle;

#define init_sem_handle(handle) (handle)->m_hAnonymousHandle = nullptr;
#define is_sem_valid(handle) ((handle)->m_hAnonymousHandle != nullptr)
#endif

#define MAX_SYNCOBJECT_NAME_LENGTH 1024
#define MAX_MUTEX_NAME_LENGTH MAX_SYNCOBJECT_NAME_LENGTH
#define MAX_EVENT_NAME_LENGTH MAX_SYNCOBJECT_NAME_LENGTH
#define MAX_SEMAPHORE_NAME_LENGTH MAX_SYNCOBJECT_NAME_LENGTH
#ifndef _WIN32

#define MUTEX_MANAGE_REQUEST_SEMAPHORE_NAME "/os_mutex_manage_request_semaphore"
#define MUTEX_MANAGE_FINISH_SEMAPHORE_NAME "/os_mutex_manage_finish_semaphore"
#define MUTEX_MANAGER_ENABLE_SEMAPHORE_NAME "/os_mutex_manage_enabled_semaphore"
#define MUTEX_MANAGER_ENABLE_SHAREMEMORY_NAME "os_mutex_manage_enabled_sharemomory"

enum MUTEX_MANAGE_TYPE {
    MUTEX_CREATE_METHOD = 1,
    MUTEX_DESTROY_METHOD = 2,
};

namespace OS {
struct CSyncClient {
public:
    bool create();
    void destroy();

public:
    sem_t *enable_mutex_manager_sem;  //同步管理允许信号
    sem_t *manage_request_sem;        //创建请求信号，由客户端发起
    sem_t *manage_finish_sem;         //创建完成信号，由服务端发起
    sharememory_handle *manager_memory_handle;
};
}  // namespace OS
#endif

#endif
