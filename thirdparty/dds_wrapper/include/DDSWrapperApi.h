#pragma once

#include "Types.h"
#include "AbstractExecutor.h"
#include "AbstractTopic.h"

class CallbackGroup;
class CFastDDSDomainParticipant;

/**
 * @brief 创建执行器
 * @param executorParam 执行器参数，【in】
 * @return Executor对象指针
 */
DDS_WRAPPER_API CAbstractExecutor *createExecutor(ExecutorParam executorParam);

/**
 * @brief 创建组
 * @param executorType 组类型，【in】
 * @return Group对象指针
 */
DDS_WRAPPER_API CallbackGroup *createGroup(CallbackGroupType callbackType);

/**
 * @brief 注册组
 * @param executor Executor对象指针，【in】
 * @param group Group对象指针，【in】
 * @return 成功返回true，失败false
 */
DDS_WRAPPER_API bool registerGroup(CAbstractExecutor *executor, CallbackGroup *group);

/**
 * @brief 创建并注册服务
 * @param serviceName 服务名称，【in】
 * @param serviceInfo 包含创建service时需要的类型信息，requestTopicType: 请求类型对象指针，responseTopicType:
 * 回复类型对象指针，userData：用户自定义数据【in】
 * @param callback 自定义回调接口，【in】
 * 函数签名 void (*)(void* requestDataType, void* responseDataType, void* userData)
 * requestDataType/responseDataType：fastdds工具自动生成的topic data type对象指针，userData表示用户自定义数据
 * @param participant 域参与者对象指针，使用默认配置时传NULL【in】
 * @param group 组对象指针，使用默认组时传NULL【in】
 * @return 成功返回true，失败false
 */
DDS_WRAPPER_API bool createService(const char *serviceName, const ServiceInfo &serviceInfo, void *callback,
                                   CFastDDSDomainParticipant *participant, CallbackGroup *group);

/**
 * @brief 卸载服务
 * @param serviceName 服务名称，【in】
 * @param group 组对象指针，使用默认组时传NULL【in】
 * @return 成功返回true，失败false
 */
DDS_WRAPPER_API bool removeService(const char *serviceName, CallbackGroup *group);

/**
 * @brief 创建并注册客户端
 * @param strServiceName 服务名称，【in】
 * @param stClientInfo 包含创建 Client 时需要的类型信息【in】
 * @param participant 域参与者对象指针，使用默认配置时传NULL【in】
 * @param group 组对象指针，使用默认组时传NULL【in】
 * @return 成功返回有效的客户端句柄, 失败返回 INVALID_CLIENT
 */
DDS_WRAPPER_API DdsClientHandle createClient(const char *strServiceName, const ClientInfo &stClientInfo,
                                             CFastDDSDomainParticipant *pParticipant, CallbackGroup *pGroup);

/**
 * @brief 卸载客户端
 * @param hClient 客户端句柄，【in】
 * @param group 组对象指针，使用默认组时传NULL【in】
 * @return 成功返回true，失败false
 */
DDS_WRAPPER_API bool removeClient(DdsClientHandle hClient, CallbackGroup *pGroup);

/**
 * @brief 客户端调用服务
 * @param hClient 客户端句柄，【in】
 * @param pReqData 调用服务请求的数据【in】
 * @return 成功返回true，失败false
 */
DDS_WRAPPER_API bool clientInvokeService(DdsClientHandle hClient, void *pReqData);

/**
 * @brief 创建域参与者
 * @param xmlProfilePath 配置文件路径，[in]
 * @return 创建的域参与者对象指针，失败返回空指针
 */
DDS_WRAPPER_API CFastDDSDomainParticipant *createDomainParticipant(const char *xmlProfilePath);

/**
 * @brief 创建topic
 * @param topicName topic名称，[in]
 * @param topicType topic类型 TOPIC_PUBLISHER or TOPIC_SUBSCRIBER，[in]
 * @param topicTypeObj topic类型对象指针，[in]
 * @param participant 域参与者对象指针，使用默认配置时传NULL【in】
 * @return 创建的topic实例指针, 创建失败时返回空指针
 */
DDS_WRAPPER_API CAbstractTopic *createTopic(const char *topicName, DDSTopicType topicType, void *topicTypeObj,
                                            CFastDDSDomainParticipant *participant);

/**
 * @brief 初始化接口，负责创建默认participant，创建默认Executor并启动
 * @param xmlProfilePath qos配置文件路径，[in]
 * @return Executor对象指针
 */
DDS_WRAPPER_API bool ddsWrapperInit(const char *xmlProfilePath);

/**
 * @brief 反初始化接口
 */
DDS_WRAPPER_API void ddsWrapperUninit();