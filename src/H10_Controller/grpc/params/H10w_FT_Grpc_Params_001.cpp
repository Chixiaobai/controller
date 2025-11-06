
#include "Test _Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_001)
{
    std::cout << "验证获取关节软限位函数有效性" << std::endl;
    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：获取所有关节软限位
    auto soft_limits = grpc_params_client_->m_pControllerClient->getJointSoftLimit();

    // std::vector<std::tuple<uint32_t, double, double>> soft_limits = {
    //     {1, 3.1230, -3.1230},
    //     {2, 1.5708, -1.5708},
    //     {3, 3.1230, -3.1230},
    //     {4, 3.0543, 2},
    //     {5, 3.1230, -3.1230},
    //     {6, 3.1230, -3.1230},
    //     {7, 3.1230, -3.1230},
    //     {8, 3.1230, -3.1230},
    //     {9, 1.5708, -1.5708},
    //     {10, 3.1230, -3.1230},
    //     {11, 3.0543, 0},
    //     {12, 3.1230, -3.1230},
    //     {13, 3.1230, -3.1230},
    //     {14, 3.1230, -3.1230},
    //     {15, 0.122, -0.785},
    //     {16, 1.571, -1.571},
    //     {17, 0.75, 0.01},
    //     {18, 1.5, -1.5},
    //     {19, 1.5, -1.5}};

    ASSERT_FALSE(soft_limits.empty()) << "获取关节软限位失败：未获取到任何关节数据！";

    std::map<uint32_t, std::pair<float, float>> expect_limit;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        uint32_t index = static_cast<int32_t>(robotParameters["joint_index"][i]);
        expect_limit[index] = {static_cast<float>(robotParameters["max_limit"][i]),
                               static_cast<float>(robotParameters["min_limit"][i])};
    }

    ASSERT_TRUE(check_soft_limits_match(soft_limits, expect_limit));
    sleepMilliseconds(1000);
}
