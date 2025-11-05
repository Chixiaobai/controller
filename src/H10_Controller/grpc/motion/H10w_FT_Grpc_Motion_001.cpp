
#include "Test _Grpc_Motion_Fixture.h"
#include "main.h"

TEST_F(GrpcMotionTest, H10w_FT_Grpc_Motion_001)
{
    ASSERT_NE(grpc_motion_client_, nullptr) << "Grpc客户端初始化失败";
    ASSERT_NE(grpc_motion_client_->m_pControllerClient, nullptr) << "控制器客户端为空";
    uint32_t token = 1234;
    bool ret = grpc_motion_client_->grpc_singlemove(robotParameters["joint_index"][0], robotParameters["position"][0], robotParameters["vel"][0], token);
    ASSERT_TRUE(ret) << "单关节运动失败";
    sleepMilliseconds(1000);
}