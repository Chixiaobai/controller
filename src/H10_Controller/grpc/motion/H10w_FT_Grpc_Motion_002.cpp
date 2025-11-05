
#include "Test _Grpc_Motion_Fixture.h"
#include "main.h"

TEST_F(GrpcMotionTest, H10w_FT_Grpc_Motion_002)
{
    ASSERT_NE(grpc_motion_client_, nullptr) << "Grpc客户端初始化失败";

    uint32_t token = 1234;
    std::cout << robotParameters["joint_index"][1] << std::endl;
    std::cout << robotParameters["position"][1] << std::endl;
    std::cout << robotParameters["vel"][1] << std::endl;

    bool ret = grpc_motion_client_->grpc_singlemove(robotParameters["joint_index"][1], robotParameters["position"][1], robotParameters["vel"][1], token);
    ASSERT_TRUE(ret) << "单关节运动失败";
    sleepMilliseconds(1000);
    
    ret = grpc_motion_client_->grpc_singlemove(robotParameters["joint_index"][1], robotParameters["position_home"][1], robotParameters["vel"][1], token);
    ASSERT_TRUE(ret) << "单关节运动失败";
    sleepMilliseconds(1000);
}