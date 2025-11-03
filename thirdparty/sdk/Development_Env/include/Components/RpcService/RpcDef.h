#pragma once

#define RPC_VERSION 1

#define DEFAULT_PRODUCT_UUID "a597749e-cc28-431b-a248-b68499c9e96d"

typedef enum class __RPC_TRACE_OPTION : unsigned int {
    RPC_NO_TRACE = 0x00000000,
    RPC_TRACE_FULL = 0x00000001,
    RPC_TRACE_REQUEST_PROFILE = 0x00000100,
    RPC_TRACE_RESPONSE_PROFILE = 0x00000200,
    RPC_TRACE_NOTIFY_PROFILE = 0x00000400,
    RPC_TRACE_REQUEST_FULL = RPC_TRACE_REQUEST_PROFILE | RPC_TRACE_FULL,
    RPC_TRACE_RESPONSE_FULL = RPC_TRACE_RESPONSE_PROFILE | RPC_TRACE_FULL,
    RPC_TRACE_NOTIFY_FULL = RPC_TRACE_NOTIFY_PROFILE | RPC_TRACE_FULL,
} RPC_TRACE_OPTION;

typedef union __RPCFLAG {
    struct {
        unsigned short m_uiResponseBit : 1;  // for rpc response,this bit should be set
        // for rpc request,this bit should be set if client support exception response.
        // for rpc response, this bit should be set if it is exception response.
        unsigned short m_uiExceptionBit : 1;
        unsigned short m_uiReserveBits : 14;
    } bits;
    USHORT m_uiValue;
} RPCFLAG, *LPRPCFLAG;

typedef struct __RPCREQUESTHEAD {
    USHORT m_uiRpcVersion;      // RPC version
    USHORT m_uiRpcClassId;      // RPC component identifier
    UINT m_uintRpcObjectId;     // RPC object identifier,equal to zero for component itself
    USHORT m_uiRpcInterfaceId;  // RPC interface identifier,resolved by component itself,zero if not used.
    USHORT m_uiRpcMethodId;     // RPC method identifier
    USHORT m_uiRpcRequestSize;  // RPC request size, not including this head
    RPCFLAG m_RPCFlag;          // RPC flag
} RPCREQUESTHEAD, *LPRPCREQUESTHEAD;

typedef struct __RPCRESPONSEHEAD {
    USHORT m_uiRpcVersion;       // RPC version
    USHORT m_uiRpcClassId;       // RPC component identifier
    UINT m_uintRpcObjectId;      // RPC object identifier,equal to zero for component itself
    USHORT m_uiRpcInterfaceId;   // RPC interface identifier,resolved by component itself
    USHORT m_uiRpcMethodId;      // RPC method identifier
    USHORT m_uiRpcResponseSize;  // RPC request size, not including this head
    RPCFLAG m_RPCFlag;           // RPC flag
} RPCRESPONSEHEAD, *LPRPCRESPONSEHEAD;

/*
RPC V1协议允许的最大长度是65535, 为了支持更大的长度，引入RPC V2协议
出于兼容性考虑，当RPC V1协议中的m_uiRpcRequestSize为0xFFFF时, 采用V2协议
当使用V2协议时，数据的真实长度由m_intRpcRequestV2Size决定
出于兼容性考虑，当RPC V1协议中的m_uiRpcResponseSize为0xFFFF时, 采用V2协议
当使用V2协议时，数据的真实长度由m_intRpcResponseV2Size决定
*/
#define MAX_RPC_REQUEST_SIZE 10485760  //考虑到网络带宽，约束V2协议支持的最大数据为10M Bytes
typedef struct __RPCREQUESTHEAD_V2 {
    RPCREQUESTHEAD m_RPCRequest_V1;  // RPC request head V1
    int m_intRpcRequestV2Size;
} RPCREQUESTHEAD_V2, *LPRPCREQUESTHEAD_V2;

#define MAX_RPC_RESPONSE_SIZE 10485760  //考虑到网络带宽，约束V2协议支持的最大数据为10M Bytes
typedef struct __RPCRESPONSEHEAD_V2 {
    RPCRESPONSEHEAD m_RPCResponse_V1;  // RPC response head V1
    int m_intRpcResponseV2Size;
} RPCRESPONSEHEAD_V2, *LPRPCRESPONSEHEAD_V2;
