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
    if (sigaction(SIGINT, &stSigAction, nullptr) != 0 || sigaction(SIGTERM, &stSigAction, nullptr) != 0)
    {
        printf("Fail to set callback function for console application!\n");
        fflush(stdout);
    }
}

GTEST_CASE(Grpc_Params, H10w_FT_Grpc_Params_004, "验证设置关节软限位函数的立即生效性")
{
    setConsoleHandler();
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<H10wGrpcMove>(IpPort);
    g_pTester = node.get();

    // 启动spin循环（单独线程，避免阻塞主逻辑）
    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });

    // 获取所有关节软限位
    std::cout << "Get Joint Soft Limits: " << std::endl;
    auto soft_limits = node->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : soft_limits)
    {
        std::cout << i << " = [max:" << max << "; min:" << min << ";]" << std::endl;
    }

    // 测试任务1：修改所有关节软限位
    std::cout << "Modify Joint Soft Limits: " << std::endl;
    std::vector<uint32_t> joint_index = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    std::vector<double> min_pos = {-0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.2, -0.2, 0.1, -0.001, -0.001};
    std::vector<double> max_pos = {0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 0.2, 0.2, 0.6, 0.001, 0.001};

    // node->m_pControllerClient->setJointSoftLimit(joint_index, min_pos, max_pos);

    char ret = read_input("查看配置文件中的软限位，是否与设置的一致？(y/n)\n");
    EXPECT_EQ(ret, 'y') << "用户确认结果不一致，获取关节软限位失败";
    // // 测试任务2：恢复关节软限位
    // for (auto &[i, max, min] : soft_limits)
    // {

    //     max_pos[i] = max;
    //     min_pos[i] = min;
    // }

    // node->m_pControllerClient->setJointSoftLimit(joint_index, min_pos, max_pos);
    // ret = read_input("查看配置文件中的软限位，是否与设置的一致？(y/n)\n");
    // EXPECT_EQ(ret, 'y') << "用户确认结果不一致，设置关节软限位失败";

    node->stopTest();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    node.reset();

    g_pTester = nullptr;
    sleepMilliseconds(1000);
}
