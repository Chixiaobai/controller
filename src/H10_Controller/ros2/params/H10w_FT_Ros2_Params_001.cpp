#include <signal.h>
#include "H10wRosClient.h"
#include "main.h"
static H10wRosClient *g_pTester = nullptr;

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

GTEST_CASE(auto_Ros_Params, H10w_FT_Ros_Params_001, "测试获取软限位")
{
    setConsoleHandler();
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<H10wRosClient>(IpPort);
    g_pTester = node.get();

    // 启动spin循环（单独线程，避免阻塞主逻辑）
    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });

    while (rclcpp::ok() && !node->has_move_msg())
    {
        std::this_thread::sleep_for(10ms);
    }

    std::vector<controller::msg::JointParams> joint_params;
    if (node->get_joint_soft_limit(joint_params))
    {
        std::cout << "get_joint_soft_limit succeed!" << std::endl;
        for (const auto &param : joint_params)
        {
            std::cout << "joint index: " << param.joint_index;
            std::cout << "max limit: " << param.max_pos;
            std::cout << "min limit: " << param.min_pos << std::endl;
        }
    }
    else
    {
        std::cout << "get_joint_soft_limit failed!" << std::endl;
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
