#pragma once

class CAbstractExecutor {
public:
    CAbstractExecutor();

    virtual ~CAbstractExecutor() = default;

    /**
     * @brief 主循环
     * @return 调用cancel后退出
     */
    virtual void spin() = 0;

    /**
     * @brief 退出主循环
     */
    void cancel();
};