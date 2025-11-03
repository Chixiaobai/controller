#pragma once

#define CREATE_RPC_SERVANT_IDENTIFY(mainid, subid) \
    ((((unsigned short)(mainid & 0x0FFF)) << 4) | ((unsigned short)(subid & 0x000F)))

#define RPC_MAIN_IDENTIFY(servant_id) (((unsigned short)(servant_id & 0xFFF0)) >> 4)
#define RPC_SUB_IDENTIFY(servant_id) ((unsigned short)(servant_id & 0x000F))

#define USER_METHOD_ID 256

enum SYSTEM_INTERFACE_ID {
    BASE_INTERFACE_ID = 1,
    CALLBACK_POINT_CONTAINER_INTERFACE_ID = 2,
    CALLBACK_POINT_INTERFACE_ID = 3,
    USER_INTERFACE_ID = 256,
};

enum BASE_METHOD_ID {
    RELEASE_OBJECT_METHOD = 1,
};

enum CALLBACK_POINT_CONTAINER_METHOD_ID {
    FIND_CALLBACK_POINT_METHOD = 1,
};

enum CALLBACK_POINT_METHOD_ID {
    ADVISE_METHOD = 1,
    UNADVISE_METHOD = 2,
};

// This struct will not transmit through RPC, it is only used for adjusting whether content is empty
typedef struct _RPC_CONTENT_HOLDER {
    unsigned long long m_ullHolderValue;
} RPC_CONTENT_HOLDER, *LPRPC_CONTENT_HOLDER;

// RELEASE_REF_METHOD
typedef struct __RELEASEREFREQUEST {
    RPC_CONTENT_HOLDER m_RpcContentHolder;
} RELEASEREFREQUEST, *LPRELEASEREFREQUEST;

typedef struct __RELEASEREFRESPONSE {
    RPC_CONTENT_HOLDER m_RpcContentHolder;
} RELEASEREFRESPONSE, *LPRELEASEREFRESPONSE;
