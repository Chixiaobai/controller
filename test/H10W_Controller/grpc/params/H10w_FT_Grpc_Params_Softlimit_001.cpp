#include "Test_Grpc_Params_Fixture.h" 
#include "main.h"
class GrpcSoftLimitParamsTest : public GrpcParamsTest, 
                                public testing::TestWithParam<std::tuple<std::vector<double>, std::vector<double>>> {
public:
    // 显式指定使用父类的静态函数（解决潜在的继承歧义）
    static void SetUpTestSuite() { GrpcParamsTest::SetUpTestSuite(); }
    static void TearDownTestSuite() { GrpcParamsTest::TearDownTestSuite(); }
};

TEST_P(GrpcSoftLimitParamsTest, H10w_FT_Grpc_Params_Softlimit_001)
{
    std::cout << "验证设置/获取关节软限位函数" << std::endl;
    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    std::vector<JointSoftLimitParams> set_limits;
    const auto &max_limit = std::get<0>(GetParam());
    const auto &min_limit = std::get<1>(GetParam());
    for (size_t i = 0; i < max_limit.size(); ++i)
    {
        set_limits.emplace_back(robotParameters["joint_index"][i], max_limit[i], min_limit[i]);
    }
    // 测试任务1：设置关节软限位
    auto ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(set_limits);
    ASSERT_TRUE(ret) << "设置关节软限位失败";
    // 测试任务2：获取关节软限位
    std::vector<JointSoftLimitParams> soft_limits;
    ret = grpc_params_client_->m_pControllerClient->getJointSoftLimit(soft_limits);
    ASSERT_TRUE(ret) << "获取关节软限位失败";

    bool same = (set_limits == soft_limits);
    ASSERT_TRUE(same) << "获取关节软限位结果与预期不符";

    sleepMilliseconds(1000);
}

INSTANTIATE_TEST_SUITE_P(
    H10w_FT_Grpc_Params_Softlimit_001,
    GrpcSoftLimitParamsTest,
    testing::Values(
        std::make_tuple(robotParameters["max_limit"], robotParameters["min_limit"]),
        std::make_tuple(robotParameters["max_limit"], robotParameters["min_limit_default"]),
        std::make_tuple(robotParameters["max_limit_default"], robotParameters["min_limit"]),
        std::make_tuple(robotParameters["abnormal_max_limit"], robotParameters["abnormal_min_limit"]),
        std::make_tuple(robotParameters["max_limit_default"], robotParameters["min_limit_default"])));