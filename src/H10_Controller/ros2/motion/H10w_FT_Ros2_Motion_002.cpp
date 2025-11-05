#include "Test _Ros_Motion_Fixture.h"
#include "main.h"

TEST_F(Ros2MotionTest, H10w_FT_Ros2_Motion_002)
{

    ASSERT_NE(ros_motion_client_, nullptr) << "Ros客户端初始化失败";


    bool ret = ros_motion_client_->ros_singlemove(robotParameters["joint_index"][1], robotParameters["position"][0], robotParameters["vel"][0]);
    ASSERT_TRUE(ret) << "单关节运动失败";

    sleepMilliseconds(1000);

    ret = ros_motion_client_->ros_singlemove(robotParameters["joint_index"][1], robotParameters["position_home"][0], robotParameters["vel"][0]);
    ASSERT_TRUE(ret) << "单关节运动失败";
}