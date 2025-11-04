#include <signal.h>
#include "Test.h"
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

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_001)
{
    setConsoleHandler();
    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";
    ASSERT_NE(grpc_params_client_->m_pControllerClient, nullptr) << "控制器客户端为空";

    // 测试任务1：获取所有关节软限位
    std::cout << "Get Joint Soft Limits: " << std::endl;
    auto soft_limits = grpc_params_client_->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : soft_limits)
    {
        std::cout << i << " = [max:" << max << "; min:" << min << ";]" << std::endl;
    }

    EXPECT_EQ(1, 2);
    
    sleepMilliseconds(1000);
}
