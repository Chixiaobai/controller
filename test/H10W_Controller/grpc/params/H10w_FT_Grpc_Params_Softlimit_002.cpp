#include "Test_Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_Softlimit_002)
{
    std::cout << "验证设置关节软限位函数的立即生效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";
    std::vector<JointSoftLimitParams> get_limits;
    bool ret = grpc_params_client_->m_pControllerClient->getJointSoftLimit(get_limits);
    ASSERT_TRUE(ret) << "获取关节软限位失败！";


    uint32_t token = 0;
    // 测试任务1：控制各个关节运动到0.7rad
    std::vector<MoveParams> params;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        params.emplace_back(robotParameters["joint_index"][i], robotParameters["position1"][i], robotParameters["velocity"][i]);
    }
    ret = grpc_params_client_->grpc_multimove(params, token);
    ASSERT_TRUE(ret) << "控制所有关节运动到0.7rad失败！";

    // 测试任务2：修改各个关节的最大软限位为0.5rad
    std::vector<JointSoftLimitParams> set_limits;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_limits.emplace_back(robotParameters["joint_index"][i], robotParameters["max_limit"][i], robotParameters["min_limit_default"][i]);
    }
    ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(set_limits);
    ASSERT_TRUE(ret) << "设置关节软限位失败！";

    // 测试任务3：控制各个关节运动到0.0rad
    std::vector<MoveParams> params1;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        params1.emplace_back(robotParameters["joint_index"][i], robotParameters["position2"][i], robotParameters["velocity"][i]);
    }
    ret = grpc_params_client_->grpc_multimove(params1, token);
    ASSERT_TRUE(!ret) << "控制所有关节运动到0.0rad成功，测试失败！";

    // 测试任务4：恢复默认关节软限位
    ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(get_limits);
    ASSERT_TRUE(ret) << "恢复默认关节软限位失败！";

    // 测试任务5：控制各个关节运动到0.0rad
    ret = grpc_params_client_->grpc_multimove(params1, token);
    ASSERT_TRUE(ret) << "控制所有关节运动到0.0rad失败！";

    // 测试任务6：控制各个关节运动到初始位置
    params.clear();
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        params.emplace_back(robotParameters["joint_index"][i], robotParameters["position_home"][i], robotParameters["velocity"][i]);
    }
    ret = grpc_params_client_->grpc_multimove(params, token);
    ASSERT_TRUE(ret) << "控制所有关节运动到初始位置失败！";

    sleepMilliseconds(1000);
}