#include "Test_Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_009)
{
    std::cout << "验证设置关节最大速度函数的有效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：设置关节最大速度
    std::vector<JointMaxParams> set_vel;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel"][i]);
    }
    bool ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
    ASSERT_TRUE(ret) << "设置关节最大速度失败！";

    // 测试任务2：获取关节最大速度
    std::vector<JointMaxParams> get_vel;
    ret = grpc_params_client_->m_pControllerClient->getJointMaxVel(get_vel);
    ASSERT_TRUE(ret) << "获取关节最大速度失败！";

    // 测试任务3：验证获取到的关节最大速度是否与设置一致
    bool same = (get_vel == set_vel);
    ASSERT_TRUE(same) << "获取关节最大速度结果与预期不符";

    // 测试任务4：恢复关节最大速度
    set_vel.clear();
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel_default"][i]);
    }
    ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
    ASSERT_TRUE(ret) << "恢复关节最大速度失败！";

    // 测试任务5：验证关节最大速度是否恢复为默认值
    get_vel.clear();
    ret = grpc_params_client_->m_pControllerClient->getJointMaxVel(get_vel);
    ASSERT_TRUE(ret) << "获取关节最大速度失败！";

    same = (get_vel == set_vel);
    ASSERT_TRUE(same) << "获取关节最大速度结果与预期不符";

    sleepMilliseconds(1000);
}