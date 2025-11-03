#pragma once

#include "Types.h"

class CAbstractTopic {
public:
    CAbstractTopic() = default;

    virtual ~CAbstractTopic() = default;

    /**
     * @brief 初始化 Topic
     * @return true 初始化成功
     * @return false 初始化失败
     */
    virtual bool initTopic() = 0;

    /**
     * @brief 反初始化 Topic
     * @return true 反初始化成功
     * @return false 反初始化失败
     */
    virtual bool uninitTopic() = 0;

    /**
     * @brief Topic 是否已初始化
     * @return true Topic 已初始化
     * @return false Topic 未初始化
     */
    virtual bool isInited() const = 0;

    /**
     * @brief 接收 Topic 订阅消息
     * @param pData [in] Topic 订阅的消息结构的指针, 用于保存接收到的 Topic 数据
     * @param pReturnCode [out] 当不为nullptr时, 返回接收topic订阅的错误码,
     * @return true 接收 Topic 订阅消息成功
     * @return false 接收 Topic 订阅消息失败
     */
    virtual bool recvSubscription(void* pData, ReturnCode* pReturnCode = nullptr) = 0;

    /**
     * @brief 发送 Topic 发布消息
     * @param pData [in] Topic 发布的消息数据结构的指针, 用于保存待发送的 Topic 数据
     * @return true 发送 Topic 发布消息成功发
     * @return false 发送 Topic 发布消息失败
     */
    virtual bool sendPublication(const void* pData, ReturnCode* pReturnCode = nullptr) = 0;

    virtual void* getDataWriter() = 0;

    virtual void* getDataReader() = 0;
};