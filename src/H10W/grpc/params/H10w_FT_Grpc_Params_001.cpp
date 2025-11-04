#include <signal.h>
#include "H10wGrpcMove.h"
#include "main.h"

static H10wGrpcMove *g_pTester = nullptr;

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

GTEST_CASE(Grpc_Params, H10w_FT_Grpc_Params_001, "验证获取关节软限位函数有效性")
{
    setConsoleHandler();
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<H10wGrpcMove>(IpPort);
    g_pTester = node.get();

    // 启动spin循环（单独线程，避免阻塞主逻辑）
    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });

    // 测试任务1：获取所有关节软限位
    std::cout << "Get Joint Soft Limits: " << std::endl;
    auto soft_limits = node->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : soft_limits)
    {
        std::cout << i << " = [max:" << max << "; min:" << min << ";]" << std::endl;
    }

    char ret = read_input("将上述结果与配置文件中的软限位对比，是否一致？(y/n)\n");
    EXPECT_EQ(ret, 'y') << "用户确认结果不一致，获取关节软限位失败";

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
