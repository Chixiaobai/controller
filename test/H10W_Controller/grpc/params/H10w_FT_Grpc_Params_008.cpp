#include "Test_Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_008)
{
    std::cout << "验证关节最大速度修改后，获取关节最大速度结果正确性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：获取关节最大速度
    std::vector<JointMaxParams> get_vel;
    bool ret = grpc_params_client_->m_pControllerClient->getJointMaxVel(get_vel);
    ASSERT_TRUE(ret) << "获取关节最大速度失败！";

    //测试任务2：修改关节最大速度
    std::vector<JointMaxParams> set_vel;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel"][i]);
    }
    ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
    ASSERT_TRUE(ret) << "设置关节最大速度失败！";

    // 测试任务3：验证获取到的关节最大速度是否与设置一致
    std::vector<JointMaxParams> expect_vel;
    ret = grpc_params_client_->m_pControllerClient->getJointMaxVel(expect_vel);
    ASSERT_TRUE(ret) << "获取关节最大速度失败！";

    bool same = (expect_vel == set_vel);
    ASSERT_TRUE(same) << "获取关节最大速度结果与预期不符";

    // 测试任务4：恢复关节最大速度
    ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(get_vel);
    ASSERT_TRUE(ret) << "恢复关节最大速度失败！";

    sleepMilliseconds(1000);
}