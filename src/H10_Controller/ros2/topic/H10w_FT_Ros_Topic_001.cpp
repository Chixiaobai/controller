
#include "Test _Ros_Topic_Fixture.h"
#include "main.h"
TEST_F(Ros2TopicTest, H10w_FT_Ros2_Topic_001)
{

     ASSERT_NE(ros_topic_client_, nullptr) << "Grpc客户端初始化失败";
    ASSERT_NE(ros_topic_client_->m_pControllerClient, nullptr) << "控制器客户端为空";

    controller::msg::RealTimeBodyJoints msg{};
    msg.left_arm = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    msg.right_arm = {0.5,0,0,1.57,0,1.57,0};
    // msg.right_arm = {0,0,0,1.57,0,1.57,0};
    // msg.right_arm = {0,0.2,0.2,2,0,1.0,0.5};
    msg.torso = {-0.2,0.2,0.3};
    msg.left_arm_valid = false;
    msg.right_arm_valid = true;
    msg.torso_valid = false;
    msg.time = 0.001;
    double step = 0.0001;
    ros_topic_client_->servoj(msg, step);

    sleepMilliseconds(1000);
}