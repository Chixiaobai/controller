
#include "Test _Ros_Params_Fixture.h"
#include "main.h"
TEST_F(Ros2ParamsTest, H10w_FT_Ros2_Params_001)
{
    ASSERT_NE(ros_params_client_, nullptr) << "ros客户端初始化失败";

    std::vector<controller::msg::JointParams> joint_params;
    bool ret = ros_params_client_->get_joint_soft_limit(joint_params);
    ASSERT_TRUE(ret) << "获取关节软限位失败";
    sleepMilliseconds(1000);
}
