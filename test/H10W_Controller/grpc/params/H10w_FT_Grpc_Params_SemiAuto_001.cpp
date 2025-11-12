#include "Test_Grpc_Params_Fixture.h"
#include "main.h"
class SemiGrpcParamsTest : public GrpcParamsTest
{
protected:
    static void SetUpTestSuite()
    {
        GrpcParamsTest::SetUpTestSuite();
    }
    static void TearDownTestSuite()
    {
        GrpcParamsTest::TearDownTestSuite();
    }
    void SetUp()
    {
        GrpcParamsTest::SetUp();
    }
    void TearDown()
    {
        GrpcParamsTest::TearDown();
    }
};

TEST_F(SemiGrpcParamsTest, H10w_FT_Grpc_Params_SemiAuto_001)
{
    std::cout << "验证设置关节软限位函数的结果的永久有效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";
    std::vector<JointSoftLimitParams> set_limits;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_limits.emplace_back(robotParameters["joint_index"][i], robotParameters["max_limit"][i], robotParameters["min_limit"][i]);
    }
    if (read_input("是否为第一次执行该程序(第二次执行前需要重启机器人并输入n继续执行)?(y/n)") == 'y')
    {
        // 测试任务1：修改关节软限位
        bool ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(set_limits);
        ASSERT_TRUE(ret) << "设置关节软限位失败！";
    }
    else
    {
        // 测试任务2：重启后获取关节软限位
        std::vector<JointSoftLimitParams> soft_limits;
        bool ret = grpc_params_client_->m_pControllerClient->getJointSoftLimit(soft_limits);
        ASSERT_TRUE(ret) << "获取关节软限位失败";

        bool same = (set_limits == soft_limits);
        ASSERT_TRUE(same) << "获取关节软限位结果与预期不符";

        // 测试任务3：恢复所有关节软限位
        set_limits.clear();
        for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
        {
            set_limits.emplace_back(robotParameters["joint_index"][i], robotParameters["max_limit_default"][i], robotParameters["min_limit_default"][i]);
        }
        ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(set_limits);
        ASSERT_TRUE(ret) << "恢复默认限位失败";
    }
    sleepMilliseconds(1000);
}

TEST_F(SemiGrpcParamsTest, H10w_FT_Grpc_Params_SemiAuto_002)
{
    std::cout << "验证设置关节最大速度函数的结果的永久有效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";
    std::vector<JointMaxParams> set_vel;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel"][i]);
    }
    if (read_input("是否为第一次执行该程序(第二次执行前需要重启机器人并输入n继续执行)?(y/n)") == 'y')
    {
        // 测试任务1：修改关节最大速度
        bool ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
        ASSERT_TRUE(ret) << "设置关节最大速度失败！";
    }
    else
    {
        // 测试任务2：重启后获取关节最大速度
        std::vector<JointMaxParams> get_vel;
        bool ret = grpc_params_client_->m_pControllerClient->getJointMaxVel(get_vel);
        ASSERT_TRUE(ret) << "获取关节最大速度失败";

        bool same = (get_vel == set_vel);
        ASSERT_TRUE(same) << "获取关节最大速度结果与预期不符";

        // 测试任务3：恢复所有关节最大速度
        set_vel.clear();
        for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
        {
            set_vel.emplace_back(robotParameters["joint_index"][i], robotParameters["max_vel_default"][i]);
        }
        ret = grpc_params_client_->m_pControllerClient->setJointMaxVel(set_vel);
        ASSERT_TRUE(ret) << "恢复默认速度失败";
    }
    sleepMilliseconds(1000);
}
