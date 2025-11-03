#include <signal.h>
#include "H10wRosServojSaftMonitor.h"
#include "h1_sdk_base.h"

static H10wRosServojSaftMonitor *g_pTester = nullptr;

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

// 定义测试用例的描述, 方便用户了解测试内容
void H10w_FT_SpeedMonitor_Elevator_001() { printf("测试超速阈值为0时，100%速度移动不会触发超速停机\n"); }

// 定义测试实体，多个用例可以关联同一个实体
void h10w_ft_speedmonitor_elevator_001()
{

    setConsoleHandler();

    std::vector<std::string> context; // 测试任务内容
    std::vector<bool> num;            // 测试任务结果
    std::vector<std::string> vec;     // 存储错误信息容器

    auto node = std::make_shared<H10wRosServojSaftMonitor>(IpPort);
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
    context.emplace_back("移动伸缩电机");
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

    // msg.torso = {0.0,0.0,0.4};
    // msg.right_arm = {0.5,0,0,1.57,0,1.57,0};
    // node->servoj(msg, step);

    node->enable_realtime_cmd(false);
    node->stopTest();
    // 等待spin线程结束
    spin_thread.join();

    // 解析测试结果
    Analysis_Test_Task_Result(num, "H10w_FT_SpeedMonitor_Elevator_001");
    if (num.at(num.size() - 1))
    {
        setCaseSucceed("H10w_FT_SpeedMonitor_Elevator_001");
    }
    else
    {
        setCaseFailed("H10w_FT_SpeedMonitor_Elevator_001");
    }

    sleepMilliseconds(1000);
}

// 注册测试用例及测试方法
REGIST_CASE_FUNCTION(h10w_ft_speedmonitor_elevator_001)
REGIST_CASE(H10w_FT_SpeedMonitor_Elevator_001, h10w_ft_speedmonitor_elevator_001, H10w_FT_SpeedMonitor_Elevator_001);