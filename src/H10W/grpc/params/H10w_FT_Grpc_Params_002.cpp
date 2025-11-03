#include <signal.h>
#include "H10wGrpcMove.h"
#include "main.h"

static H10wGrpcMove *g_pTester = nullptr;

// 信号处理函数（保持原有逻辑）
static void consoleHandler(int intSigNum)
{
    if ((SIGINT == intSigNum) || (SIGTERM == intSigNum))
    {
        if (nullptr != g_pTester)
        {
            g_pTester->stopTest();
        }
    }
}

// 设置控制台信号处理（保持原有逻辑）
static void setConsoleHandler()
{
    struct sigaction stSigAction;
    stSigAction.sa_handler = &consoleHandler;
    sigemptyset(&stSigAction.sa_mask);
    stSigAction.sa_flags = 0;
    if (sigaction(SIGINT, &stSigAction, nullptr) != 0 ||
        sigaction(SIGTERM, &stSigAction, nullptr) != 0)
    {
        printf("Fail to set callback function for console application!\n");
        fflush(stdout);
    }
}

// 定义测试用例的描述, 方便用户了解测试内容
GTEST_CASE(auto_Grpc_Params, H10w_FT_Grpc_Params_002, "验证关节软限位修改后，获取关节软限位结果正确性")
{
    setConsoleHandler();

    auto test_context = std::make_shared<rclcpp::Context>();
    test_context->init(0, nullptr);

    auto node = std::make_shared<H10wGrpcMove>(IpPort, test_context);
    g_pTester = node.get(); // 绑定全局指针用于信号处理

    // 启动spin循环（单独线程，避免阻塞主逻辑）
    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });

    int count = 0;
    // 测试任务1：获取所有关节软限位
    std::cout << "Get Joint Soft Limits: " << std::endl;
    auto soft_limits = node->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : soft_limits)
    {
        std::cout << i << " = [max:" << max << "; min:" << min << ";]" << std::endl;
    }

    // 测试任务2：修改关节软限位
    std::cout << "Modify Joint Soft Limits: " << std::endl;
    std::vector<uint32_t> joint_index = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    std::vector<double> min_pos = {-0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.2, -0.2, 0.1, -0.001, -0.001};
    std::vector<double> max_pos = {0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 0.2, 0.2, 0.6, 0.001, 0.001};

    node->m_pControllerClient->setJointSoftLimit( joint_index, min_pos, max_pos);

    auto new_soft_limits = node->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : new_soft_limits)
    {

        if (max_pos[i] == max && min_pos[i] == min)
        {
            count++;
        }
    }
    EXPECT_EQ(count, joint_index.size());

    // 测试任务3：恢复所有关节软限位
    count = 0;
    for (auto &[i, max, min] : soft_limits)
    {

        max_pos[i] = max;
        min_pos[i] = min;
    }

    node->m_pControllerClient->setJointSoftLimit( joint_index, min_pos, max_pos);

    new_soft_limits = node->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : new_soft_limits)
    {

        if (max_pos[i] == max && min_pos[i] == min)
        {
            count++;
        }
    }
    EXPECT_EQ(count, joint_index.size());

    node->stopTest();
    // 清理资源
    node->stopTest();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    node.reset();

    g_pTester = nullptr;
    sleepMilliseconds(1000);
}