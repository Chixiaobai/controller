#include "Test _Grpc_Params_Fixture.h"
#include "main.h"

TEST_F(GrpcParamsTest, H10w_FT_Grpc_Params_004)
{
    std::cout << "验证设置关节软限位函数的立即生效性" << std::endl;

    ASSERT_NE(grpc_params_client_, nullptr) << "Grpc客户端初始化失败";

    uint32_t token = 0;
    // 测试任务1：控制各个关节运动到0.7rad
    bool ret = grpc_params_client_->grpc_multimove(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["position1"], robotParameters["velocity"], token);
    ASSERT_TRUE(ret) << "控制所有关节运动到0.7rad失败！";

    // 测试任务2：修改各个关节的最大软限位为0.5rad
    ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["max_limit"], robotParameters["min_limit_default"]);
    ASSERT_TRUE(ret) << "设置关节软限位失败！";

    // 测试任务3：控制各个关节运动到0.0rad
    ret = grpc_params_client_->grpc_multimove(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["position2"], robotParameters["velocity"], token);
    ASSERT_TRUE(!ret) << "控制所有关节运动到0.0rad成功，测试失败！";

    // 测试任务4：恢复默认关节软限位
    ret = grpc_params_client_->m_pControllerClient->setJointSoftLimit(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["max_limit_default"], robotParameters["min_limit_default"]);
    ASSERT_TRUE(ret) << "恢复默认限位失败";

    //测试任务5：控制各个关节运动到0.0rad
    ret = grpc_params_client_->grpc_multimove(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["position2"], robotParameters["velocity"], token);
    ASSERT_TRUE(ret) << "控制所有关节运动到0.0rad失败！";

    //测试任务6：控制各个关节运动到初始位置
    ret = grpc_params_client_->grpc_multimove(float_to_uint32_vec(robotParameters["joint_index"]), robotParameters["position_home"], robotParameters["velocity"], token);
    ASSERT_TRUE(ret) << "控制所有关节运动到初始位置失败！";
    
    sleepMilliseconds(1000);
}