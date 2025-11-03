#pragma once

#include "inttypes.h"

enum ETHERCAT_DATA_TYPE {
    ETHERCAT_INVALID_TYPE = -1,
    ETHERCAT_ANY = 0,
    ETHERCAT_INT8 = 1,
    ETHERCAT_UINT8 = 2,
    ETHERCAT_INT16 = 3,
    ETHERCAT_UINT16 = 4,
    ETHERCAT_INT32 = 5,
    ETHERCAT_UINT32 = 6,
    ETHERCAT_INT64 = 7,
    ETHERCAT_UINT64 = 8,
    ETHERCAT_FLOAT = 9,
    ETHERCAT_DOUBLE = 10,
};

enum ETHERCAT_OPERATE_MODE { ETHERCAT_INVALID_MODE = 0, ETHERCAT_READ = 1, ETHERCAT_WRITE = 2 };

enum COMM_DIAG_INDEX { MIN_COMM_COST = 0, CURRENT_COMM_COST, MAX_COMM_COST, MAX_DIAG_INDEX };

typedef struct _ETHERCAT_COMM_DIAG_INFO {
    unsigned long long m_ullProcessTime;
    unsigned int m_uintProcessTxPDOCost;     // unit: us
    unsigned int m_uintProcessSdoCost;       // unit: us
    unsigned int m_uintProcessRxPDOCost;     // unit: us
    unsigned int m_uintProcessCallbackCost;  // unit: us
    unsigned int m_uintTotalExchangeCost;    // unit: us
} ETHERCAT_COMM_DIAG_INFO, *LPETHERCAT_COMM_DIAG_INFO;
