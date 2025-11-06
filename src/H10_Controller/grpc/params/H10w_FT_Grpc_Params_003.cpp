#include "Test _Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_003)
{
    std::cout << "验证设置关节软限位函数的有效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：修改关节软限位
    bool ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["max_limit"], robotParameters["min_limit"]);
    ASSERT_TRUE(ret) << "设置关节软限位失败！";

    // 测试任务2：获取所有关节软限位并与设置值比较
    auto soft_limits = grpc_params_client_->m_pControllerClient->getJointSoftLimit();
    ASSERT_FALSE(soft_limits.empty()) << "获取关节软限位失败：未获取到任何关节数据！";

    std::map<uint32_t, std::pair<float, float>> expect_limit;
    for (size_t i = 0; i < robotParameters["joint_index"].size(); ++i)
    {
        uint32_t index = static_cast<uint32_t>(robotParameters["joint_index"][i]);
        expect_limit[index] = {static_cast<float>(robotParameters["max_limit"][i]),
                               static_cast<float>(robotParameters["min_limit"][i])};
    }
    ASSERT_TRUE(check_soft_limits_match(soft_limits, expect_limit));

    // 测试任务3：恢复所有关节软限位
    std::vector<uint32_t> jointdex;
    std::vector<float> max_limit, min_limit;
    for (auto &[id, max_val, min_val] : soft_limits)
        jointdex.push_back(id), max_limit.push_back(max_val), min_limit.push_back(min_val);

    ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(jointdex, max_limit, min_limit);
    ASSERT_TRUE(ret) << "恢复默认限位失败";

    sleepMilliseconds(1000);
}