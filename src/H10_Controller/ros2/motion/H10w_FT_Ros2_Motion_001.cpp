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

GTEST_CASE(auto_Ros_Motions, H10w_FT_Ros_Motion_001, "测试单关节移动")
{

    setConsoleHandler();

    rclcpp::init(0, nullptr);

    auto node = std::make_shared<H10wRosClient>(IpPort);
    g_pTester = node.get();

    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });
    while (rclcpp::ok() && !node->has_move_msg())
    {
        std::this_thread::sleep_for(10ms);
    }

    node->ros_singlemove(7, 1.5, 0.1);

    char ret = read_input("是否正常结束运动(y/n)\n");
    EXPECT_EQ(ret, 'y') << "用户确认结果不一致，运动异常";
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