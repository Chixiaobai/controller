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

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_002)
{
    setConsoleHandler();

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";
    ASSERT_NE(grpc_params_client_->m_pControllerClient, nullptr) << "控制器客户端为空";

    int count = 0;
    // 测试任务1：获取所有关节软限位
    std::cout << "Get Joint Soft Limits: " << std::endl;
    auto soft_limits = grpc_params_client_->m_pControllerClient->getJointSoftLimit();
    for (auto &[i, max, min] : soft_limits)
    {
        std::cout << i << " = [max:" << max << "; min:" << min << ";]" << std::endl;
    }

    // // 测试任务2：修改关节软限位
    // std::cout << "Modify Joint Soft Limits: " << std::endl;
    // std::vector<uint32_t> joint_index = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
    // std::vector<double> min_pos = {-1.5, -1.5, -1.5, 0.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, 0.5, -1.5, -1.5, -1.5, -0.2, -0.2, 0.01};
    // std::vector<double> max_pos = {1.5, 1.5, 1.5, 1.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.0, 1.5, 1.5, 1.5, 0.2, 0.1, 0.6};

    // node->m_pControllerClient->setJointSoftLimit(joint_index, min_pos, max_pos);

    // auto new_soft_limits = node->m_pControllerClient->getJointSoftLimit();
    // for (auto &[i, max, min] : new_soft_limits)
    // {

    //     if (max_pos[i] == max && min_pos[i] == min)
    //     {
    //         count++;
    //     }
    // }
    // EXPECT_EQ(count, joint_index.size());

    // // 测试任务3：恢复所有关节软限位
    // count = 0;
    // for (auto &[i, max, min] : soft_limits)
    // {

    //     max_pos[i] = max;
    //     min_pos[i] = min;
    // }

    // node->m_pControllerClient->setJointSoftLimit(joint_index, min_pos, max_pos);

    // new_soft_limits = node->m_pControllerClient->getJointSoftLimit();
    // for (auto &[i, max, min] : new_soft_limits)
    // {

    //     if (max_pos[i] == max && min_pos[i] == min)
    //     {
    //         count++;
    //     }
    // }
    // EXPECT_EQ(count, joint_index.size());

    EXPECT_EQ(1, 2);

    sleepMilliseconds(1000);
}