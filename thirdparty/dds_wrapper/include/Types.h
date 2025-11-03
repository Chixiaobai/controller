#pragma once

#if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#ifdef DDS_WRAPPER_EXPORTS
#define DDS_WRAPPER_API extern "C" __declspec(dllexport)
#define DDS_WRAPPER_CLASS __declspec(dllexport)
#else
#define DDS_WRAPPER_API extern "C" __declspec(dllimport)
#define DDS_WRAPPER_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define DDS_WRAPPER_API extern "C" __attribute__((visibility("default")))
#define DDS_WRAPPER_CLASS __attribute__((visibility("default")))
#else
#define DDS_WRAPPER_API extern "C"
#define DDS_WRAPPER_CLASS
#endif
#endif

/**
 * @brief FastDDS错误码
 */
typedef unsigned int ReturnCode;

const ReturnCode RETURN_CODE_OK = 0;
const ReturnCode RETURN_CODE_ERROR = 1;
const ReturnCode RETURN_CODE_UNSUPPORTED = 2;
const ReturnCode RETURN_CODE_BAD_PARAMETER = 3;
const ReturnCode RETURN_CODE_PRECONDITION_NOT_MET = 4;
const ReturnCode RETURN_CODE_OUT_OF_RESOURCES = 5;
const ReturnCode RETURN_CODE_NOT_ENABLED = 6;
const ReturnCode RETURN_CODE_IMMUTABLE_POLICY = 7;
const ReturnCode RETURN_CODE_INCONSISTENT_POLICY = 8;
const ReturnCode RETURN_CODE_ALREADY_DELETED = 9;
const ReturnCode RETURN_CODE_TIMEOUT = 10;
const ReturnCode RETURN_CODE_NO_DATA = 11;
const ReturnCode RETURN_CODE_ILLEGAL_OPERATION = 12;
const ReturnCode RETURN_CODE_NOT_ALLOWED_BY_SECURITY = 13;
const ReturnCode RETURN_CODE_NO_SUBSCRIPTION_ALIVE = 14;

/**
 * @brief 执行器类型类型
 */
enum ExecutorType
{
    SingleThreaded,
    MultiThreaded
};

struct SingleThreadedExecutorParam
{
};

struct MultiThreadedExecutorParam
{
    unsigned long numberOfThreads;
    bool yieldBeforeExecute;
    long long timeoutNs;
};

struct ExecutorParam
{
    ExecutorType type;
    union
    {
        SingleThreadedExecutorParam singleThreadedParam;
        MultiThreadedExecutorParam multiThreadedParam;
    };
};

/**
 * @brief 组类型
 */
enum CallbackGroupType
{
    MutuallyExclusive,
    Reentrant
};

/**
 * @brief 创建service时需要的topic类型信息
 */
struct ServiceInfo
{
    void *requestTopicType;
    void *responseTopicType;
    void *userData;
};

/**
 * @brief topic类型
 */
enum DDSTopicType
{
    TOPIC_PUBLISHER = 0,
    TOPIC_SUBSCRIBER = 1
};

#define INVALID_CLIENT (-1)
typedef int DdsClientHandle;

// void (*FnClientResponseCallback)(void* responseDataType, void* userData)
// responseDataType：fastdds工具自动生成的 topic data type 对象指针，userData表示用户自定义数据
typedef void (*FnClientResponseCallback)(void *, void *);

struct ClientInfo {
    void *requestTopicType;               // 请求类型对象指针
    void *responseTopicType;              // 回复类型对象指针
    FnClientResponseCallback fnCallback;  // 收到 response 时调用的回调函数
    void *userData;                       // 用户自定义数据
};
