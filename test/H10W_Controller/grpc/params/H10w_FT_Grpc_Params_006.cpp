#include "Test _Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_006)
{
    std::cout << "验证设置关节软限位函数对范围极限值的兼容性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：设置关节软限位
    std::vector<JointSoftLimitParams> set_limits;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_limits.emplace_back(robotParameters["joint_index"][i], robotParameters["max_limit"][i], robotParameters["min_limit"][i]);
    }
    bool ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(set_limits);
    ASSERT_TRUE(ret) << "设置关节软限位失败！";

    // 测试任务2：获取所有关节软限位
    std::vector<JointSoftLimitParams> soft_limits;
    ret = grpc_params_client_->m_pControllerClient->getJointSoftLimit(soft_limits);
    ASSERT_TRUE(ret) << "获取关节软限位失败！";

    // 测试任务3：验证获取到的关节软限位是否与预期一致
    bool same = (set_limits == soft_limits);
    ASSERT_TRUE(same) << "获取关节软限位结果与预期不符";

    sleepMilliseconds(1000);
}