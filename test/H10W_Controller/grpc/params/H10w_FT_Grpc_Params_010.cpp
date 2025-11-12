#include "Test_Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_010)
{
    std::cout << "验证设置关节最大速度函数的立即生效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：获取关节最大速度
    std::vector<JointMaxParams> get_vel;
    bool ret = grpc_params_client_->m_pControllerClient->getJointMaxVel(get_vel);
    ASSERT_TRUE(ret) << "获取关节最大速度失败！";

    // 测试任务2：修改关节最大速度
    std::vector<JointMaxParams> set_vel;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel1"][i]);
    }
    ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
    ASSERT_TRUE(ret) << "设置关节最大速度失败！";

    uint32_t token = 1000;
    // 测试任务3：控制各个关节以最大速度运动至0.7rad 再回原
    std::vector<MoveParams> params;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        params.emplace_back(robotParameters["joint_index"][i], robotParameters["position1"][i], robotParameters["velocity"][i]);
    }
    ret = grpc_params_client_->grpc_multimove(params, token);
    ASSERT_TRUE(ret) << "控制所有关节运动到0.7rad失败！";

    std::vector<MoveParams> params_home;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        params_home.emplace_back(robotParameters["joint_index"][i], robotParameters["position_home"][i], robotParameters["velocity"][i]);
    }
    ret = grpc_params_client_->grpc_multimove(params_home, token);
    ASSERT_TRUE(ret) << "控制所有关节回原失败！";

    // 测试任务4：再次修改关节最大速度0.1
    set_vel.clear();
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel2"][i]);
    }
    ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
    ASSERT_TRUE(ret) << "设置关节最大速度失败！";

    // 测试任务5：控制各个关节以最大速度运动至0.7rad 再回原
    ret = grpc_params_client_->grpc_multimove(params, token);
    ASSERT_TRUE(ret) << "控制所有关节运动到0.7rad失败！";

    ret = grpc_params_client_->grpc_multimove(params_home, token);
    ASSERT_TRUE(ret) << "控制所有关节回原失败！";

    // 测试任务6：恢复关节最大速度
    ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(get_vel);
    ASSERT_TRUE(ret) << "恢复默认速度失败";

    sleepMilliseconds(1000);
}