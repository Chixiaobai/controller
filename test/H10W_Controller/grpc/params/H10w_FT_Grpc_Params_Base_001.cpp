#include "Test_Grpc_Params_Fixture.h" 
#include "main.h"
struct BaseParams
{
    double linear_vel;
    double angular_vel;
};
class GrpcBaseParamsTest : public GrpcParamsTest, public testing::TestWithParam<BaseParams> {
public:
    // 显式重写静态函数，指定调用夹具的实现（关键！）
    static void SetUpTestSuite() { GrpcParamsTest::SetUpTestSuite(); }
    static void TearDownTestSuite() { GrpcParamsTest::TearDownTestSuite(); }
};

TEST_P(GrpcBaseParamsTest, H10w_FT_Grpc_Params_Base_001)
{
    const BaseParams &params = GetParam();
    std::cout << "验证获取、设置底盘最大移动速度​函数" << std::endl;

    ASSERT_NE(GrpcParamsTest::grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    // 测试任务1：设置底盘最大移动速度
    bool ret = GrpcParamsTest::grpc_params_client_->m_pControllerClient->setBaseMaxVel(params.linear_vel, params.angular_vel);
    ASSERT_TRUE(ret) << "设置底盘最大移动速度失败！";

    // 测试任务2：获取底盘最大移动速度
    double linear_vel, angular_vel;
    ret = GrpcParamsTest::grpc_params_client_->m_pControllerClient->getBaseMaxVel(linear_vel, angular_vel);
    ASSERT_TRUE(ret) << "获取底盘最大移动速度失败！";

    ASSERT_NEAR(linear_vel, params.linear_vel, 1e-6) << "获取的底盘最大线速度与设置的线速度不一致";
    ASSERT_NEAR(angular_vel, params.angular_vel, 1e-6) << "获取的底盘最大角速度与设置的角速度不一致";
    sleepMilliseconds(1000);
}
INSTANTIATE_TEST_SUITE_P(
    H10w_FT_Grpc_Params_Base_001,
    GrpcBaseParamsTest,
    testing::Values(
        BaseParams{0.5, 0.5},       // 第一组参数 任意设置线速度/角速度
        BaseParams{1.0, 1.0},       // 第二组参数 修改线速度/角速度
        BaseParams{0.0, 0.0},       // 第三组参数 设置最小值线速度/角速度
        BaseParams{1.667, 3.1416},  // 第四组参数 设置+极限值线速度/角速度
        BaseParams{-1.667, -3.1416} // 第五组参数 设置-极限值线速度/角速度
        ));