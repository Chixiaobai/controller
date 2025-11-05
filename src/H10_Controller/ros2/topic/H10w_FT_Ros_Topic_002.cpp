// #include <signal.h>
// #include "H10wRosClient.h"
// #include "main.h"

// static H10wRosServolSaftMonitor *g_pTester = nullptr;

// static void consoleHandler(int intSigNum)
// {
//     if ((SIGINT == intSigNum) || (SIGTERM == intSigNum))
//     {

//         if (nullptr != g_pTester)
//         {
//             g_pTester->stopTest();
//         }
//     }
// }

// static void setConsoleHandler()
// {
//     struct sigaction stSigAction;
//     stSigAction.sa_handler = &consoleHandler;
//     sigemptyset(&stSigAction.sa_mask);
//     stSigAction.sa_flags = 0;
//     if (sigaction(SIGINT, &stSigAction, nullptr) != 0 || sigaction(SIGTERM, &stSigAction, nullptr) != 0)
//     {
//         printf("Fail to set callback function for console application!\n");
//         fflush(stdout);
//     }
// }

// // 定义测试用例的描述, 方便用户了解测试内容
// void H10w_FT_SpeedMonitor_Elevator_002() { printf("测试超速阈值为0时，100%速度移动不会触发超速停机\n"); }

// // 定义测试实体，多个用例可以关联同一个实体
// void h10w_ft_speedmonitor_elevator_002()
// {

//     setConsoleHandler();

//     std::vector<std::string> context; // 测试任务内容
//     std::vector<bool> num;            // 测试任务结果
//     std::vector<std::string> vec;     // 存储错误信息容器

//     auto node = std::make_shared<H10wRosServolSaftMonitor>(IpPort);
//     g_pTester = node.get();

//     // 启动spin循环（单独线程，避免阻塞主逻辑）
//     std::thread spin_thread([&node]()
//                             { rclcpp::spin(node); });

//      while (rclcpp::ok() && !node->has_move_msg()) {
//         std::this_thread::sleep_for(10ms);
//     }
//     // int index = static_cast<int>(h10_w::JOINT_INDEX::kElevatorMotor);
//     // float max_speed = 0.3;
//     // float thr_speed = 0;

//     // // 测试任务1
//     // context.emplace_back("设置关节最大速度为最大值，超速阈值等于0");
//     // int ret = node->m_pMonitorClient->SetJointSpeedCheckConfig(index, max_speed, thr_speed);
//     // Test_Task_Result(num, ret, 1);

//     // power && brake on
//     node->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
//     node->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);

//     // 测试任务2
//     context.emplace_back("移动伸缩电机");
//     node->enable_realtime_cmd(true);
//     controller::msg::RealTimeBodyTcpCartesian msg{};
//     for (int i = 0; i < 6; i++)
//     {
//         msg.left_arm[i] = node->get_move_msg_->tcp_pose[0].pose[i];
//         msg.right_arm[i] = node->get_move_msg_->tcp_pose[1].pose[i];
//         // std::cout<<msg.right_arm[i]<<"\n";
//         msg.torso[i] = node->get_move_msg_->tcp_pose[2].pose[i];
//     }
//     // msg.right_arm ={0.73,node->get_move_msg_->tcp_pose[1].pose[1],node->get_move_msg_->tcp_pose[1].pose[2],node->get_move_msg_->tcp_pose[1].pose[3],node->get_move_msg_->tcp_pose[1].pose[4],node->get_move_msg_->tcp_pose[1].pose[5]};
//     // msg.left_arm = {0.0,0.0,0.0,0.0,0.0,0.0};
//     // msg.right_arm[0] -= 0.1;
//     msg.right_arm[1] += 0.05;
//     // msg.right_arm[2] += 0.05;

//     // msg.torso = {0.0,0.0,0.0,0.0,0.0,0.0};
//     // sleepMilliseconds(5000);
//     // std::cout<<node->get_move_msg_->tcp_pose[1].pose[0]<<"\n";

//     msg.left_arm_valid =false;
//     msg.right_arm_valid =true;
//     msg.torso_valid =false;
//     msg.time = 0.001;
//     double step = 0.00003;
//     node->servol(msg, step);
    
//     msg.right_arm[0] += 0.1;
//     msg.right_arm[1] -= 0.05;
//     // msg.right_arm[2] -= 0.05;

//     node->servol(msg, step);

//     node->enable_realtime_cmd(false);
//     node->stopTest();
//     // 等待spin线程结束
//     spin_thread.join();
//     // char value = read_input("是否触发了超速停机[是-y,否-n]\n");
//     // Test_Task_Result(num, value, 'n');

//     // 解析测试结果
//     Analysis_Test_Task_Result(num, "H10w_FT_SpeedMonitor_Elevator_002");
//     if (num.at(num.size() - 1))
//     {
//         setCaseSucceed("H10w_FT_SpeedMonitor_Elevator_002");
//     }
//     else
//     {
//         setCaseFailed("H10w_FT_SpeedMonitor_Elevator_002");
//     }

//     sleepMilliseconds(1000);
// }

// // 注册测试用例及测试方法
// REGIST_CASE_FUNCTION(h10w_ft_speedmonitor_elevator_002)
// REGIST_CASE(H10w_FT_SpeedMonitor_Elevator_002, h10w_ft_speedmonitor_elevator_002, H10w_FT_SpeedMonitor_Elevator_002);