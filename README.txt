使用方法:

重新生成一下grpc的文件
cd include_H10w/grpc_msg/grpc_ws
./generate_grpc_pb_cpp.sh

cd include_H10w/rpc_service
./generate_grpc_pb_cpp.sh


1. 编译controller
cd fastdds_ws
colcon build --packages-select controller

2.source controller的环境
source install/setup.bash

3.编译测试程序
在CmakeLists.txt同级目录下编译即可
colcon build --merge-install --packages-select h10w_controller_test

4.source 环境
source install/setup.bash
export LD_LIBRARY_PATH=thirdparty/grpc/lib/:$LD_LIBRARY_PATH

每新开一个终端，都需要source一次环境

5.运行测试程序
(1) 测试单个用例，假定用例ID为H10w_FT_Grpc_Params_001:
ros2 run h10w_controller_test h10w_controller_test  --gtest_filter=GrpcParamsTest.H10w_FT_Grpc_Params_001

(2) 测试每个模块的所有用例，假定模块为GrpcParamsTest，并将测试报告保存为report.html:
ros2 run h10w_controller_test h10w_controller_test  --gtest_filter=GrpcParamsTest.* -report report.html

(3) 执行所有用例，并将测试报告保存为report.html:
ros2 run h10w_controller_test h10w_controller_test  --gtest_filter=*.* -report report.html
