使用方法:
1. 测试单个用例，假定用例ID为H10w_FT_Grpc_Params_001:
ros2 run h10w_controller_test h10w_controller_test  --gtest_filter=GrpcParamsTest.H10w_FT_Grpc_Params_001

2. 测试每个模块的所有用例，假定模块为GrpcParamsTest，并将测试报告保存为report.html:
ros2 run h10w_controller_test h10w_controller_test  --gtest_filter=GrpcParamsTest.* -report report.html

3. 执行所有用例，并将测试报告保存为report.html:

ros2 run h10w_controller_test h10w_controller_test  --gtest_filter=*.* -report report.html
