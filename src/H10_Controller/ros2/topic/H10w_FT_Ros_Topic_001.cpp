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

GTEST_CASE(auto_Ros_Topic, H10w_FT_Ros_Topic_001, "servoj")
{

    setConsoleHandler();
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<H10wRosClient>(IpPort);
    g_pTester = node.get();

    // 启动spin循环（单独线程，避免阻塞主逻辑）
    std::thread spin_thread([&node]()
                            { rclcpp::spin(node); });

     while (rclcpp::ok() && !node->has_move_msg()) {
        std::this_thread::sleep_for(10ms);
    }

    // power && brake on
    node->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
    node->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);

    // 测试任务2
    node->enable_realtime_cmd(true);
    controller::msg::RealTimeBodyJoints msg{};
    msg.left_arm = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    msg.right_arm = {0.5,0,0,1.57,0,1.57,0};
    // msg.right_arm = {0,0,0,1.57,0,1.57,0};
    // msg.right_arm = {0,0.2,0.2,2,0,1.0,0.5};
    msg.torso = {-0.2,0.2,0.3};
    msg.left_arm_valid = false;
    msg.right_arm_valid = true;
    msg.torso_valid = false;
    msg.time = 0.001;
    double step = 0.0001;
    node->servoj(msg, step);
    sleepMilliseconds(500);

    node->enable_realtime_cmd(false);

    char ret = read_input("是否正常运动(y/n)\n");
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