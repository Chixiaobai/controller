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

GTEST_CASE(Grpc_motions, H10w_FT_Grpc_Motion_001, "测试单关节移动")
{
    setConsoleHandler();

    auto node = std::make_shared<H10wGrpcMove>(IpPort);
    g_pTester = node.get();

    while (rclcpp::ok() && !node->has_move_msg())
    {
        std::this_thread::sleep_for(10ms);
    }

    float pos = 0;
    float vel = 0.1;
    uint32_t token = 1234;
    node->grpc_singlemove(7, pos, vel, token);
    sleepMilliseconds(1000);

    char ret = read_input("是否正常结束运动(y/n)\n");
    EXPECT_EQ(ret, 'y') << "用户确认结果不一致，运动异常";
    node->stopTest();

    sleepMilliseconds(1000);
}