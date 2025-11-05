
#include "Test _Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_001)
{
    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";
    ASSERT_NE(grpc_params_client_->m_pControllerClient, nullptr) << "控制器客户端为空";

    // 测试任务1：获取所有关节软限位
    std::cout << "Get Joint Soft Limits: " << std::endl;
    // auto soft_limits = grpc_params_client_->m_pControllerClient->getJointSoftLimit();
    // for (auto &[i, max, min] : soft_limits)
    // {
    //     std::cout << i << " = [max:" << max << "; min:" << min << ";]" << std::endl;
    // }
    std::vector<std::tuple<uint32_t, double, double>> soft_limits = {
        {1, 3.1230, -3.1230},
        {2, 3.1230, -3.1230},
        {3, 3.1230, -3.1230},
        {4, 3.0543, 0},
        {5, 3.1230, -3.1230},
        {6, 1.5708, -1.5708},
        {7, 3.1230, -3.1230},
        {8, 3.1230, -3.1230},
        {9, 3.1230, -3.123},
        {10, 3.1230, -3.1230},
        {11, 3.0543, 0},
        {12, 3.1230, -3.1230},
        {13, 1.5708, -1.5708},
        {14, 3.1230, -3.1230},
        {15, 0.122, -0.785},
        {16, 1.571, -1.571},
        {17, 0.78, 0.01}};

    std::map<uint32_t, std::pair<double, double>> expected_limits = {
        {1, {3.1230, -3.1230}},
        {2, {3.1230, -3.1230}},
        {3, {3.1230, -3.1230}},
        {4, {3.0543, 0}},
        {5, {3.1230, -3.1230}},
        {6, {1.5708, -1.5708}},
        {7, {3.1230, -3.1230}},
        {8, {3.1230, -3.1230}},
        {9, {3.1230, -3.1230}},
        {10, {3.1230, -3.1230}},
        {11, {3.0543, 0}},
        {12, {3.1230, -3.1230}},
        {13, {1.5708, -1.5708}},
        {14, {3.1230, -3.1230}},
        {15, {0.122, -0.785}},
        {16, {1.571, -1.571}},
        {17, {0.75, 0.01}}};
    ASSERT_TRUE(std::all_of(soft_limits.begin(), soft_limits.end(), [&](const auto &t)
                            {
    auto [idx, act_max, act_min] = t;  
    auto [exp_max, exp_min] = expected_limits[idx]; 
    bool max_ok = std::abs(act_max - exp_max) < 1e-4;
    bool min_ok = std::abs(act_min - exp_min) < 1e-4;

    if (!max_ok || !min_ok) {
        std::cout << "\nIndex " << idx << " 不匹配：实际(max:" << act_max << ", min:" << act_min << ")，预期(max:" << exp_max << ", min:" << exp_min << ")" << std::flush;
    }
    return max_ok && min_ok; }))
        << "\n获取关节软限位失败";

    sleepMilliseconds(1000);
}
