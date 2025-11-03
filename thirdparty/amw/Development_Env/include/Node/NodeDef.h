#pragma once

#if defined _WIN32 
#ifdef NODE_EXPORTS
#define NODE_API extern "C" __declspec(dllexport)
#define NODE_CLASS __declspec(dllexport)
#else   
#define NODE_API extern "C" __declspec(dllimport)
#define NODE_CLASS __declspec(dllimport)
#endif  
#else
#if __GNUC__ >= 4
#define NODE_API extern "C" __attribute__ ((visibility("default")))
#define NODE_CLASS __attribute__ ((visibility("default")))
#else   
#define NODE_API extern "C"
#define NODE_CLASS
#endif  
#endif

/**
 * service_callback_func，service服务的回调函数
 * reqLen，请求消息的长度
 * resLen，应答消息的长度
 * */
typedef int (*service_callback_func)(const void *reqData, size_t reqLen, void **resData, size_t *resLen,
                                     void *user_data);
typedef void *pService;

/**
 * @defgroup NodeErrorCodes Node Error Codes
 * @brief Definitions of error codes used throughout the node communication system.
 *
 * These constants represent various error conditions that may occur during node,
 * port, or service operations. Functions typically return these negative values
 * to indicate failure conditions.
 * @{
 */

/**
 * @brief Error: System has not been initialized.
 *
 * Returned when attempting to use functionality before calling `node_api_initialize()`.
 */
const int ERR_NOT_INIT = -1;

/**
 * @brief Error: Service or request pointer is null.
 *
 * Indicates a missing or invalid service object reference.
 */
const int ERR_SERVICE_NULL = -2;

/**
 * @brief Error: Service not registered or request does not exist.
 *
 * Used when trying to access or invoke an unknown or unregistered service.
 */
const int ERR_SERVICE_NOT_EXIST = -3;

/**
 * @brief Error: Service name is null or empty.
 *
 * Indicates invalid service name input.
 */
const int ERR_SERVICE_NAME_NULL = -4;

/**
 * @brief Error: A service provider already exists for this service.
 *
 * Prevents multiple providers registering for the same service.
 */
const int ERR_SERVICE_PROVIDER_EXIST = -5;

/**
 * @brief Error: Message serialization/deserialization failed.
 *
 * Occurs when converting data to/from byte streams for transmission.
 */
const int ERR_SERIALIZE_FAILED = -6;

/**
 * @brief Error: Request call failed unexpectedly.
 *
 * Generic failure code for service requests that did not complete successfully.
 */
const int ERR_REQUEST_FAILED = -7;

/**
 * @brief Error: Request timed out before receiving a response.
 *
 * Indicates no response was received within the specified timeout period.
 */
const int ERR_REQUEST_TIMEOUT = -8;

/**
 * @brief Error: Specified node does not exist.
 *
 * Occurs when referencing a node that has not been created.
 */
const int ERR_NODE_NOT_EXIST = -9;

/**
 * @brief Error: Node name is null or empty.
 *
 * Indicates invalid node name input.
 */
const int ERR_NODE_NAME_NULL = -10;

/** @} */ // End of NodeErrorCodes group
