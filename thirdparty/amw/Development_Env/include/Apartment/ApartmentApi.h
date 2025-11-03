#pragma once

#include "ApartmentDef.h"
#include "IApartmentCallback.h"
#include "ModuleDef/IModuleVersion.h"

#define USER_RAW_FRAME_BASE 1000

APARTMENT_API IModuleVersion *apartment_get_module_version();

class CApartment {
public:
    IModuleVersion *getModuleVersion() { return apartment_get_module_version(); }
};

enum APARTMENT_RESPONSE_CODE {
    REQUEST_OK = 0,                 // 请求执行成功
    REQUEST_METHOD_NOT_FOUND = -1,  // 请求的方法不存在
    REQUEST_PARAMETER_ERROR = -2,   // 请求的参数不正确
    REQUEST_OPERATE_FAILED = -3     // 请求的操作失败
};

/** 初始化全局的Apartment
 * Apartment Url全网络唯一，目前仅支持如下形式的定位:
 * tcp://ipaddress:ipport, 例如: tcp://localhost:7500, tcp://192.168.10.75:7500
 * 如果不需要直接建立监听服务可传参'tcp://'
 * */
APARTMENT_API bool apartment_initialize(const char *strApartmentUrl, bool supportPassiveConnection = true);

/* 销毁全局的Apartment*/
APARTMENT_API void apartment_uninitialize();

/* 与远端apartment url连接时使用的本地apartment url, 可能与初始化时候的不一样*/
APARTMENT_API bool apartment_local_url(const char *strApartmentUrl, const char *strRemoteApartmentUrl,
                                       const bool bRtConnection, char *strLocalApartmentUrl, DWORD &dwLen);

/** 在指定apartment建立一个远端连接
 *  strApartmentUrl本地url
 *  strRemoteApartmentUrl远端url，如：tcp://192.168.10.75:7500
 **/
APARTMENT_API bool apartment_connect_remote(const char *strApartmentUrl, const char *strRemoteApartmentUrl,
                                            const bool bRtConnection);

/** 在指定apartment查找远端连接
 *  strApartmentUrl本地url
 *  strRemoteApartmentUrl远端url，如：tcp://192.168.10.75:7500
 *  strLocalUrl返回用于连接远端的本地url，null时仅返回长度
 *  localUrlLen返回用于连接远端的本地url的长度
 **/
APARTMENT_API bool apartment_has_connect_remote(const char *strApartmentUrl, const char *strRemoteApartmentUrl,
                                                const bool bRtConnection);

/** 断开指定apartment一个远端连接
 *  strApartmentUrl本地url
 *  strRemoteApartmentUrl远端url，如：tcp://192.168.10.75:7500
 **/
APARTMENT_API bool apartment_disconnect_remote(const char *strApartmentUrl, const char *strRemoteApartmentUrl,
                                               const bool bRtConnection);
/*标记当前是否启动了消息派发线程 */
APARTMENT_API bool apartment_set_dispatch_enabled(const char *strApartmentUrl, bool bEnable);

/** 为指定的Apartment，进行一次事件派发，strApartmentUrl为空时表示全局的Apartment，
 * 当超时时间timeout为{-1, -1}时，表示需要无限等待，
 * 当返回值为-1时，表示指定指定时间内没有收到消息；当返回值为0时，表示指定指定时间内收到退出消息；当返回值为1时，表示指定指定时间内收到非退出消息
 * */
APARTMENT_API int apartment_dispatch_message(const char *strApartmentUrl, timespec timeout = {-1, -1});

/** 为指定的Apartment，停止事件派发，strApartmentUrl为空时表示全局的Apartment * */
APARTMENT_API bool apartment_stop_dispatch_message(const char *strApartmentUrl);

/** 在指定的Apartment中创建一个Node, strApartmentUrl为空时表示全局的Apartment
 * Node Url全网络唯一，且仅支持如下形式的定位:
 * namespace://name, 例如: default://class/objectname
 * 初始化全局的Apartment
 * */
APARTMENT_API bool apartment_node_create(const char *strApartmentUrl, const char *strNodeUrl);
/* 在指定的Apartment中销毁一个Node, strApartmentUrl为空时表示全局的Apartment */
APARTMENT_API bool apartment_node_destroy(const char *strApartmentUrl, const char *strNodeUrl);
/* 在指定的Apartment中，指定消息回调函数，strApartmentUrl为空时表示全局的Apartment*/
APARTMENT_API bool apartment_set_callback(const char *strApartmentUrl, IApartmentCallback *pCallback);
/* 在指定的Apartment中，为指定的Node指定消息回调函数，strApartmentUrl为空时表示全局的Apartment*/
APARTMENT_API bool apartment_node_set_callback(const char *strApartmentUrl, const char *strNodeUrl,
                                               IApartmentNodeCallback *pCallback);
/** 指定源Apartment向目的Apartment发送一包原始数据:
 * 原始数据中包含FrameType以及应用层数据
 * */
APARTMENT_API int apartment_raw_write(const char *strApartmentUrl, const char *strRemoteApartmentUrl,
                                      const unsigned int uintFrameType, void *pRawBuffer, unsigned int &uintBufLen);
/** 在指定的Apartment中，指定一个Node,
 * 向另一个Apartment的Node发送一个反馈，strApartmentUrl为空时表示全局的Apartment，发送成功则返回>0，
 * uintResponseCode取值范围是APARTMENT_RESPONSE_CODE
 * 发送出错则返回<0，当发送成功时，uintRequestId标志了请求的Id，该Id在本地唯一，且应该大于0
 * */
APARTMENT_API int apartment_node_write_response(const char *strApartmentUrl, const char *strNodeUrl,
                                                const char *strRemoteApartmentUrl, const char *strRemoteNodeUrl,
                                                unsigned int uintRequestId, int intResponseCode, void *pResponseBuffer,
                                                unsigned int uintBufLen);
/** 在指定的Apartment中，指定一个Node,
 * 向另一个Apartment的Node发送一个请求，strApartmentUrl为空时表示全局的Apartment，发送成功则返回>0，
 * 发送出错则返回<0，当发送成功时，uintRequestId标志了请求的Id，该Id在本地唯一
 * */
APARTMENT_API int apartment_node_write_request(const char *strApartmentUrl, const char *strNodeUrl,
                                               const char *strRemoteApartmentUrl, const char *strRemoteNodeUrl,
                                               void *pRequestBuffer, unsigned int uintBufLen,
                                               unsigned int &uintRequestId);

/** 在指定的Apartment中，指定一个Node,
 * 向另一个Apartment的Node发送一个请求，strApartmentUrl为空时表示全局的Apartment，发送成功则返回>0，
 * 发送出错则返回<0，当发送成功时，uintRequestId标志了请求的Id，该Id在本地唯一
 * */
APARTMENT_API int apartment_node_write_notify(const char *strApartmentUrl, const char *strNodeUrl,
                                              const char *strRemoteApartmentUrl, const char *strRemoteNodeUrl,
                                              void *pRequestBuffer, unsigned int uintBufLen,
                                              unsigned int &uintRequestId);

/** 在指定的Apartment中，指定一个Node以及由该Node发出的请求Id,
 * 当超时时间uintTimeout为-1时，表示需要无限等待，单位ms
 * 获取针对该请求的响应，strApartmentUrl为空时表示全局的Apartment，接收成功则返回>0，
 * 接收超时则返回0，接收出错则返回<0， 当接收不成功时， pResponseBuffer里的内容非预期。
 * 当pResponseBuffer为空时，uintBufLen输出请求的长度
 * 读到的反馈包括: 指定Node对端的ApartmentUrl + 指定Node对端的NodeUrl + 应用层反馈数据
 * */
APARTMENT_API int apartment_node_read_response(const char *strApartmentUrl, const char *strNodeUrl,
                                               unsigned int uintRequestId, int &intResponseCode, void *pResponseBuffer,
                                               unsigned int &uintBufLen, int intTimeout);
/*标记当前Apartment的应用层处理是否允许重入 */
APARTMENT_API bool apartment_allow_reentry(const char *strApartmentUrl, bool bAllow);

/*为指定Apartment或者Node创建一个周期触发的定时器，返回值是定时器Id*/
APARTMENT_API unsigned int apartment_set_timer(const char *strApartmentUrl, const char *strNodeUrl,
                                               unsigned int uintLagTime = 10, unsigned int uintInterval = 100);

/*为指定Apartment或者Node创建一个单次触发的定时器，返回值是定时器Id*/
APARTMENT_API unsigned int apartment_set_single_timer(const char *strApartmentUrl, const char *strNodeUrl,
                                                      unsigned int uintLagTime = 10);

/*删除一个定时器*/
APARTMENT_API bool apartment_kill_timer(const char *strApartmentUrl, const char *strNodeUrl,
                                        unsigned int uintTimerId = -1);

/*设置与远端Apartment之间的心跳属性，默认无心跳 */
APARTMENT_API bool apartment_keepalive_with(const char *strApartmentUrl, const char *strRemoteApartmentUrl,
                                            unsigned int uintAliveCounter = 10, unsigned int uintAliveInterval = 100);
