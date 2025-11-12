
#include "Test _Grpc_Motion_Fixture.h"
#include "main.h"

TEST_F(GrpcMotionTest, H10w_FT_Grpc_Motion_001)
{
    ASSERT_NE(grpc_motion_client_, nullptr) << "Grpc客户端初始化失败";

    SingleMoveParams params;
    params.joint_index = robotParameters["joint_index"][0];
    params.target_position = robotParameters["position"][0];
    params.velocity = robotParameters["vel"][0];
    params.token = 1234;
    bool ret = grpc_motion_client_->grpc_singlemove(params);
    ASSERT_TRUE(ret) << "单关节运动失败";
    sleepMilliseconds(1000);
    
    // ret = grpc_motion_client_->grpc_singlemove(robotParameters["joint_index"][0], robotParameters["position_home"][0], robotParameters["vel"][0], token);
    // ASSERT_TRUE(ret) << "单关节运动失败";
    sleepMilliseconds(1000);
}