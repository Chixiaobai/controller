#include <cassert>
#include <cmath>
#include <iostream>
#include <array>
#include <chrono>
#include <thread>

#include "H10wRosClient.h"

H10wRosClient::H10wRosClient(const std::string &strIpPort)
    : Node("H10wRosClient"),
      m_pDevCtrlSvrClient(std::make_unique<CDeviceControlServiceClient>(
          grpc::CreateChannel(strIpPort + ":8686", grpc::InsecureChannelCredentials()))),
      m_pControllerClient(std::make_unique<H10WGrpcParam>(
          grpc::CreateChannel(strIpPort + ":8585", grpc::InsecureChannelCredentials())))

{
    rclcpp::QoS qos =
        rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .history(rclcpp::HistoryPolicy::KeepLast);

    move_subscriber_ = this->create_subscription<controller::msg::MoveMessage>(
        "move_message", qos,
        std::bind(&H10wRosClient::move_callback, this,
                  std::placeholders::_1));

    error_subscriber_ = this->create_subscription<controller::msg::ErrorMessage>(
        "error_message", qos,
        std::bind(&H10wRosClient::error_callback, this,
                  std::placeholders::_1));
    m_enable_realtime_cmd =
        this->create_client<controller::srv::EnableRealtimeCmd>(
            "/h10w/controller/enable_realtime_cmd");

    m_enable_controller_client =
        this->create_client<controller::srv::EnableController>(
            "/h10w/controller/enable_controller");
    m_get_chassis_max_vel_client =
        this->create_client<controller::srv::GetChassisMaxVel>(
            "/h10w/controller/get_chassis_max_vel");
    m_get_control_policy_client =
        this->create_client<controller::srv::GetControlPolicy>(
            "/h10w/controller/get_control_policy");
    m_get_safe_mode_client =
        this->create_client<controller::srv::GetSafeMode>(
            "/h10w/controller/get_safe_mode");
    m_is_enable_controller_client =
        this->create_client<controller::srv::IsEnabledController>(
            "/h10w/controller/is_enable_controller");
    m_set_chassis_max_vel_client =
        this->create_client<controller::srv::SetChassisMaxVel>(
            "/h10w/controller/set_chassis_max_vel");
    m_set_control_policy_client =
        this->create_client<controller::srv::SetControlPolicy>(
            "/h10w/controller/set_control_policy");
    m_set_safe_mode_client =
        this->create_client<controller::srv::SetSafeMode>(
            "/h10w/controller/set_safe_mode");

    m_single_move_client =
        this->create_client<controller::srv::SingleJointMove>(
            "/h10w/controller/single_move");
    m_multi_move_client = this->create_client<controller::srv::MultiJointMove>(
        "/h10w/controller/multi_move");
    m_linear_move_client = this->create_client<controller::srv::LinearMove>(
        "/h10w/controller/linear_move");
    m_servo_body_j_publisher = this->create_publisher<controller::msg::RealTimeBodyJoints>(
        "controller/servo_body_j", qos);
    m_servo_body_l_publisher =
        this->create_publisher<controller::msg::RealTimeBodyTcpCartesian>(
            "controller/servo_body_l", qos);
    m_speed_body_j_publisher =
        this->create_publisher<controller::msg::RealTimeBodyJoints>(
            "controller/speed_body_j", qos);
    m_speed_body_l_publisher =
        this->create_publisher<controller::msg::RealTimeBodyTcpCartesian>(
            "controller/speed_body_l", qos);
    m_clear_error_client =
        this->create_client<controller::srv::ErrorClear>("/controller/clear_error");

    m_get_version_client =
        this->create_client<controller::srv::GetVersion>("/controller/get_version");
    m_get_soft_limit_client =
        this->create_client<controller::srv::GetJointSoftLimit>(
            "/h10w/controller/get_joint_soft_limit");
    m_get_joint_max_vel_client =
        this->create_client<controller::srv::GetJointMaxVel>(
            "/h10w/controller/get_joint_max_vel");
    m_get_joint_max_acc_client =
        this->create_client<controller::srv::GetJointMaxAcc>(
            "/h10w/controller/get_joint_max_acc");
    m_get_cart_trans_max_vel_client =
        this->create_client<controller::srv::GetCartesianTranslationMaxVel>(
            "/h10w/controller/get_cart_trans_max_vel");
    m_get_cart_trans_max_acc_client =
        this->create_client<controller::srv::GetCartesianTranslationMaxAcc>(
            "/h10w/controller/get_cart_trans_max_acc");
    m_get_cart_rota_max_vel_client =
        this->create_client<controller::srv::GetCartesianRotationMaxVel>(
            "/h10w/controller/get_cart_rota_max_vel");
    m_get_cart_rota_max_acc_client =
        this->create_client<controller::srv::GetCartesianRotationMaxAcc>(
            "/h10w/controller/get_cart_rota_max_acc");
    m_get_mech_limit_client =
        this->create_client<controller::srv::GetJointMechanicalLimit>(
            "/h10w/controller/get_joint_mech_limit");
    m_get_mech_max_vel_client =
        this->create_client<controller::srv::GetJointMechanicalMaxVel>(
            "/h10w/controller/get_joint_mech_max_vel");
    m_get_mech_max_acc_client =
        this->create_client<controller::srv::GetJointMechanicalMaxAcc>(
            "/h10w/controller/get_joint_mech_max_acc");
    m_get_cart_mech_trans_max_vel_client = this->create_client<
        controller::srv::GetCartesianMechanicalTranslationMaxVel>(
        "/h10w/controller/get_cart_mech_trans_max_vel");
    m_get_cart_mech_trans_max_acc_client = this->create_client<
        controller::srv::GetCartesianMechanicalTranslationMaxAcc>(
        "/h10w/controller/get_cart_mech_trans_max_acc");
    m_get_cart_mech_rota_max_vel_client = this->create_client<
        controller::srv::GetCartesianMechanicalRotationMaxVel>(
        "/h10w/controller/get_cart_mech_rota_max_vel");
    m_get_cart_mech_rota_max_acc_client = this->create_client<
        controller::srv::GetCartesianMechanicalRotationMaxAcc>(
        "/h10w/controller/get_cart_mech_rota_max_acc");
    m_get_tcp_offset_client =
        this->create_client<controller::srv::GetTcpOffset>(
            "/h10w/controller/get_tcp_offset");
    m_get_tcp_payload_client =
        this->create_client<controller::srv::GetTcpPayload>(
            "/h10w/controller/get_tcp_payload");

    m_set_soft_limit_client =
        this->create_client<controller::srv::SetJointSoftLimit>(
            "/h10w/controller/set_joint_soft_limit");
    m_set_joint_max_acc_client =
        this->create_client<controller::srv::SetJointMaxAcc>(
            "/h10w/controller/set_joint_max_acc");
    m_set_joint_max_vel_client =
        this->create_client<controller::srv::SetJointMaxVel>(
            "/h10w/controller/set_joint_max_vel");
    m_set_cart_trans_max_vel_client =
        this->create_client<controller::srv::SetCartesianTranslationMaxVel>(
            "/h10w/controller/set_cart_trans_max_vel");
    m_set_cart_trans_max_acc_client =
        this->create_client<controller::srv::SetCartesianTranslationMaxAcc>(
            "/h10w/controller/set_cart_trans_max_acc");
    m_set_cart_rota_max_vel_client =
        this->create_client<controller::srv::SetCartesianRotationMaxVel>(
            "/h10w/controller/set_cart_rota_max_vel");
    m_set_cart_rota_max_acc_client =
        this->create_client<controller::srv::SetCartesianRotationMaxAcc>(
            "/h10w/controller/set_cart_rota_max_acc");
    m_set_tcp_offset_client =
        this->create_client<controller::srv::SetTcpOffset>(
            "/h10w/controller/set_tcp_offset");
    m_set_tcp_payload_client =
        this->create_client<controller::srv::SetTcpPayload>(
            "/h10w/controller/set_tcp_paylaod");
    m_forward_client =
        this->create_client<controller::srv::ForwardKinematics>(
            "/h10w/controller/forward_kinematics");
    m_inverse_client =
        this->create_client<controller::srv::InverseKinematics>(
            "/h10w/controller/inverse_kinematics");
    RCLCPP_INFO(this->get_logger(), "client 节点已启动，等待匹配...");
}

void H10wRosClient::move_callback(const controller::msg::MoveMessage::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(msg_mutex_);
    get_move_msg_ = msg;
}

void H10wRosClient::error_callback(const controller::msg::ErrorMessage::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(msg_mutex_);
    get_error_msg_ = msg;
}

bool H10wRosClient::enable_realtime_cmd(bool m_enable)
{
    /*等待服务端上线*/
    while (!m_enable_realtime_cmd->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request =
        std::make_shared<controller::srv::EnableRealtimeCmd::Request>();
    request->enable = m_enable;
    // 发送异步请求，然后等待返回，返回时调用回调函数
    auto result_future = m_enable_realtime_cmd->async_send_request(request);
    auto response = result_future.get(); // 阻塞直到结果返回
    RCLCPP_INFO(this->get_logger(), "收到实时指令使能结果：%d", response->success);
    return response->success;
}

bool H10wRosClient::ros_singlemove(const MoveParams params, uint32_t &token)
{
    /*等待服务端上线*/
    while (!m_single_move_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 构造请求
    auto request =
        std::make_shared<controller::srv::SingleJointMove::Request>();
    request->joint_index = params.joint_index;
    request->target_position = params.target_position;
    request->velocity = params.velocity;

    auto result_future = m_single_move_client->async_send_request(request);
    // 等待服务响应
    auto response = result_future.get();
    if (!response->success)
    {
        std::cerr << "ros_singlemove RPC failed " << '\n';
        return false;
    }
    token = response->token;

    // 定义超时时间为5秒
    const auto timeout = std::chrono::seconds(5);
    // 记录开始时间
    const auto start_time = std::chrono::steady_clock::now();
    // 循环检查间隔（100毫秒）
    const auto check_interval = std::chrono::milliseconds(100);

    // 循环检查直到满足条件或超时
    while (true)
    {
        // 获取当前时间
        const auto now = std::chrono::steady_clock::now();
        // 检查是否超时
        if (now - start_time >= timeout)
        {

            RCLCPP_ERROR(this->get_logger(),
                         "ros_singlemove() 超时（%d秒），操作未完成。当前状态: token=%u, state=%d, 目标token=%u",
                         static_cast<int>(timeout.count()),
                         get_move_msg_->token,
                         get_move_msg_->state,
                         token);
            return false;
        }

        // 检查是否满足完成条件
        if ((get_move_msg_->state == 0 && get_move_msg_->token == token))
        {
            RCLCPP_INFO(this->get_logger(),
                        "ros_singlemove() 完成。token=%u, state=%d",
                        token, get_move_msg_->state);
            return true;
        }
        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }
        // 检查系统状态是否正常
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "系统状态异常，退出运动检查");
            return false;
        }

        // 等待一段时间后再次检查（避免CPU占用过高）
        std::this_thread::sleep_for(check_interval);
    }
}

bool H10wRosClient::ros_multimove(const std::vector<MoveParams> params, uint32_t &token)
{
    /*等待服务端上线*/
    while (!m_multi_move_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 检查输入参数有效性
    if (params.empty())
    {
        RCLCPP_ERROR(this->get_logger(), ");关节参数为空");
        return false;
    }

    std::vector<controller::msg::JointAngle> joint_angles;
    // 遍历参数构造关节角度信息
    for (auto &joint_params : params)
    {
        controller::msg::JointAngle joint_angle;
        joint_angle.joint_index = joint_params.joint_index;
        joint_angle.target_position = joint_params.target_position;
        joint_angle.velocity = joint_params.velocity;
        joint_angles.emplace_back(joint_angle);
    }

    // 构造请求
    auto request = std::make_shared<controller::srv::MultiJointMove::Request>();
    request->joint_angles = joint_angles;

    auto result_future = m_multi_move_client->async_send_request(request);
    auto response = result_future.get();

    if (!response->success)
    {
        std::cerr << "ros_multimove RPC failed " << '\n';
        return false;
    }
    token = response->token;

    // 定义超时时间为5秒
    const auto timeout = std::chrono::seconds(5);
    // 记录开始时间
    const auto start_time = std::chrono::steady_clock::now();
    // 循环检查间隔（100毫秒）
    const auto check_interval = std::chrono::milliseconds(100);

    // 循环检查直到满足条件或超时
    while (true)
    {
        // 获取当前时间
        const auto now = std::chrono::steady_clock::now();
        // 检查是否超时
        if (now - start_time >= timeout)
        {

            RCLCPP_ERROR(this->get_logger(),
                         "ros_multimove() 超时（%d秒），操作未完成。当前状态: token=%u, state=%d, 目标token=%u",
                         static_cast<int>(timeout.count()),
                         get_move_msg_->token,
                         get_move_msg_->state,
                         token);
            return false;
        }

        // 检查是否满足完成条件
        if ((get_move_msg_->state == 0 && get_move_msg_->token == token))
        {
            RCLCPP_INFO(this->get_logger(),
                        "ros_multimove() 完成。token=%u, state=%d",
                        token, get_move_msg_->state);
            return true;
        }
        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }
        // 检查系统状态是否正常
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "系统状态异常，退出运动检查");
            return false;
        }

        // 等待一段时间后再次检查（避免CPU占用过高）
        std::this_thread::sleep_for(check_interval);
    }
}

bool H10wRosClient::ros_linearmove(const std::vector<LinearMoveParams> params, uint32_t &token)
{
    while (!m_linear_move_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    std::vector<controller::msg::LinearMoveParams> linear_params;
    // 遍历参数构造关节角度信息
    for (auto &linear : params)
    {
        controller::msg::LinearMoveParams linear_param;
        linear_param.type = linear.type;
        for (auto &pose : linear.pose)
            linear_param.pose.push_back(pose);
        linear_param.velocity_percent = linear.velocity_percent;
        linear_param.acceleration_percent = linear.acceleration_percent;
        linear_params.emplace_back(linear_param);
    }

    // 构造请求
    auto request =
        std::make_shared<controller::srv::LinearMove::Request>();
    request->linear_move = linear_params;
    auto result_future = m_linear_move_client->async_send_request(request);

    auto response = result_future.get();
    if (!response->success)
    {
        std::cerr << "ros_linearmove RPC failed " << '\n';
        return false;
    }
    token = response->token;

    // 定义超时时间为5秒
    const auto timeout = std::chrono::seconds(5);
    // 记录开始时间
    const auto start_time = std::chrono::steady_clock::now();
    // 循环检查间隔（100毫秒）
    const auto check_interval = std::chrono::milliseconds(100);

    // 循环检查直到满足条件或超时
    while (true)
    {
        // 获取当前时间
        const auto now = std::chrono::steady_clock::now();
        // 检查是否超时
        if (now - start_time >= timeout)
        {

            RCLCPP_ERROR(this->get_logger(),
                         "ros_linearmove() 超时（%d秒），操作未完成。当前状态: token=%u, state=%d, 目标token=%u",
                         static_cast<int>(timeout.count()),
                         get_move_msg_->token,
                         get_move_msg_->state,
                         token);
            return false;
        }

        // 检查是否满足完成条件
        if ((get_move_msg_->state == 0 && get_move_msg_->token == token))
        {
            RCLCPP_INFO(this->get_logger(),
                        "ros_linearmove() 完成。token=%u, state=%d",
                        token, get_move_msg_->state);
            return true;
        }
        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }
        // 检查系统状态是否正常
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "系统状态异常，退出运动检查");
            return false;
        }

        // 等待一段时间后再次检查（避免CPU占用过高）
        std::this_thread::sleep_for(check_interval);
    }
}

bool H10wRosClient::clear_error()
{
    /*等待服务端上线*/
    while (!m_clear_error_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    auto request = std::make_shared<controller::srv::ErrorClear::Request>();
    // 发送异步请求，然后等待返回
    auto result_future = m_clear_error_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_clear_error_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    return result->success;
}

bool H10wRosClient::servoj(controller::msg::RealTimeBodyJoints msg, double m_step)
{
    if (!has_move_msg())
        return false;

    move_done_ = false; // 重置状态
    auto message = controller::msg::RealTimeBodyJoints();
    message.left_arm_valid = msg.left_arm_valid;
    message.right_arm_valid = msg.right_arm_valid;
    message.torso_valid = msg.torso_valid;
    message.time = msg.time;

    // 复制当前位置
    for (int i = 0; i < 7; ++i)
    {
        message.left_arm[i] = get_move_msg_->position[i];
        message.right_arm[i] = get_move_msg_->position[i + 7];
    }
    for (int i = 0; i < 3; ++i)
    {
        message.torso[i] = get_move_msg_->position[i + 14];
    }

    while (rclcpp::ok() && !move_done_.load())
    {
        bool all_reached = true;

        // 左臂
        if (message.left_arm_valid)
        {
            for (int i = 0; i < 7; ++i)
            {
                double &cur = message.left_arm[i];
                double tar = msg.left_arm[i];
                double delta = tar - cur;
                if (std::fabs(delta) > m_step)
                {
                    double inc = (delta > 0 ? 1.0 : -1.0) * m_step;
                    cur += inc;
                    all_reached = false;
                }
                else
                {
                    cur = tar;
                }
                message.left_arm[i] = cur;
            }
        }

        // 右臂
        if (message.right_arm_valid)
        {
            for (int i = 0; i < 7; ++i)
            {
                double &cur = message.right_arm[i];
                double tar = msg.right_arm[i];
                double delta = tar - cur;
                if (std::fabs(delta) > m_step)
                {
                    double inc = (delta > 0 ? 1.0 : -1.0) * m_step;
                    cur += inc;
                    all_reached = false;
                }
                else
                {
                    cur = tar;
                }
                message.right_arm[i] = cur;
            }
        }

        // torso
        if (message.torso_valid)
        {
            for (int i = 0; i < 3; ++i)
            {
                double &cur = message.torso[i];
                double tar = msg.torso[i];
                double delta = tar - cur;
                if (std::fabs(delta) > m_step)
                {
                    double inc = (delta > 0 ? 1.0 : -1.0) * m_step;
                    cur += inc;
                    all_reached = false;
                }
                else
                {
                    cur = tar;
                }
                message.torso[i] = cur;
            }
        }

        m_servo_body_j_publisher->publish(message);

        // 检查是否到达目标
        if (all_reached)
        {
            move_done_ = true;
            RCLCPP_INFO(this->get_logger(), "servoj reached target, exit loop");
            return true;
        }

        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(message.time));
    }
    RCLCPP_ERROR(this->get_logger(), "servoj loop exited abnormally (rclcpp not ok)");
    return false;
}

bool H10wRosClient::servol(controller::msg::RealTimeBodyTcpCartesian msg, double m_step)
{
    if (!has_move_msg())
        return false;
    move_done_ = false;
    auto message = controller::msg::RealTimeBodyTcpCartesian();
    message.left_arm_valid = msg.left_arm_valid;
    message.right_arm_valid = msg.right_arm_valid;
    message.torso_valid = msg.torso_valid;
    message.time = msg.time;

    for (int i = 0; i < 6; i++)
    {
        message.left_arm[i] = get_move_msg_->tcp_pose[0].pose[i];
        message.right_arm[i] = get_move_msg_->tcp_pose[1].pose[i];
        message.torso[i] = get_move_msg_->tcp_pose[2].pose[i];
    }

    while (rclcpp::ok() && !move_done_.load())
    {
        bool all_reached = true;
        // 左臂
        if (message.left_arm_valid)
        {
            for (int i = 0; i < 6; ++i)
            {
                double &cur = message.left_arm[i];
                double tar = msg.left_arm[i];
                double delta = tar - cur;
                if (std::fabs(delta) > m_step)
                { // 未到目标
                    double inc = (delta > 0 ? 1.0 : -1.0) * m_step;
                    cur += inc;
                    all_reached = false;
                }
                else
                {
                    cur = tar;
                }
                message.left_arm[i] = cur;
            }
        }

        // 右臂
        if (message.right_arm_valid)
        {
            for (int i = 0; i < 6; ++i)
            {
                double &cur = message.right_arm[i];
                double tar = msg.right_arm[i];
                double delta = tar - cur;
                if (std::fabs(delta) > m_step)
                { // 未到目标
                    double inc = (delta > 0 ? 1.0 : -1.0) * m_step;
                    cur += inc;
                    all_reached = false;
                }
                else
                {
                    cur = tar;
                }
                message.right_arm[i] = cur;
            }
        }

        // 躯干
        if (message.torso_valid)
        {
            for (int i = 0; i < 6; ++i)
            {
                double &cur = message.torso[i];
                double tar = msg.torso[i];
                double delta = tar - cur;
                if (std::fabs(delta) > m_step)
                { // 未到目标
                    double inc = (delta > 0 ? 1.0 : -1.0) * m_step;
                    cur += inc;
                    all_reached = false;
                }
                else
                {
                    cur = tar;
                }
                message.torso[i] = cur;
            }
        }
        m_servo_body_l_publisher->publish(message);

        if (all_reached)
        {
            move_done_ = true;
            RCLCPP_INFO(this->get_logger(), "servol reached target, exit loop");
            return true;
        }

        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(message.time));
    }
    RCLCPP_ERROR(this->get_logger(), "servol loop exited abnormally (rclcpp not ok)");
    return false;
}

bool H10wRosClient::speedj(controller::msg::RealTimeBodyJoints msg, int32_t t)
{
    move_done_ = false;
    auto message = controller::msg::RealTimeBodyJoints();
    message.left_arm_valid = msg.left_arm_valid;
    message.right_arm_valid = msg.right_arm_valid;
    message.torso_valid = msg.torso_valid;
    message.time = msg.time;
    int a = 0;
    while (rclcpp::ok() && !move_done_.load())
    {
        for (int i = 0; i < 7; i++)
        {
            message.left_arm[i] = msg.left_arm[i];
            message.right_arm[i] = msg.right_arm[i];
        }
        for (int i = 0; i < 3; i++)
        {
            message.torso[i] = msg.torso[i];
        }

        m_speed_body_j_publisher->publish(message);
        std::this_thread::sleep_for(std::chrono::duration<double>(message.time));
        a++;
        if (a == t)
        {
            for (int i = 0; i < 7; i++)
            {
                message.left_arm[i] = 0;
                message.right_arm[i] = 0;
            }
            for (int i = 0; i < 3; i++)
            {
                message.torso[i] = 0;
            }
            m_speed_body_j_publisher->publish(message);
            move_done_ = true;
            RCLCPP_INFO(this->get_logger(), "speedj reached target, exit loop");
            return true;
        }
        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }
    }
    RCLCPP_ERROR(this->get_logger(), "speedj loop exited abnormally (rclcpp not ok)");
    return false;
}

bool H10wRosClient::speedl(controller::msg::RealTimeBodyTcpCartesian msg, int32_t t)
{
    move_done_ = false;
    auto message = controller::msg::RealTimeBodyTcpCartesian();

    message.left_arm_valid = msg.left_arm_valid;
    message.right_arm_valid = msg.right_arm_valid;
    message.torso_valid = msg.torso_valid;
    message.time = msg.time;
    int a = 0;

    while (rclcpp::ok() && !move_done_.load())
    {
        for (int i = 0; i < 6; i++)
        {
            message.left_arm[i] = msg.left_arm[i];
            message.right_arm[i] = msg.right_arm[i];
            message.torso[i] = msg.torso[i];
        }
        m_speed_body_l_publisher->publish(message);
        std::this_thread::sleep_for(std::chrono::duration<double>(message.time));
        a++;
        if (a == t)
        {
            for (int i = 0; i < 6; i++)
            {
                message.left_arm[i] = 0;
                message.right_arm[i] = 0;
                message.torso[i] = 0;
            }
            m_speed_body_l_publisher->publish(message);
            move_done_ = true;
            RCLCPP_INFO(this->get_logger(), "speedl reached target, exit loop");
            return true;
        }
        // error
        if (get_error_msg_->error_code != 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "message:module=%u, error_code=%d, msg=%s",
                         get_error_msg_->module,
                         get_error_msg_->error_code,
                         get_error_msg_->msg.c_str());
            return false;
        }
    }
    RCLCPP_ERROR(this->get_logger(), "speedl loop exited abnormally (rclcpp not ok)");
    return false;
}

bool H10wRosClient::get_version(ControllerVersion &version)
{
    /*等待服务端上线*/
    while (!m_get_version_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    auto request = std::make_shared<controller::srv::GetVersion::Request>();
    // 发送异步请求，然后等待返回
    auto result_future = m_get_version_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_version_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    version.main = result->main;
    for (size_t i = 0; i < result->plugin_names.size(); ++i)
    {
        version.plugins[result->plugin_names[i]] = result->plugin_versions[i];
    }
    return true;
}

bool H10wRosClient::get_joint_soft_limit(std::vector<controller::msg::JointParams> &joint_params)
{
    /*等待服务端上线*/
    while (!m_get_soft_limit_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    auto request =
        std::make_shared<controller::srv::GetJointSoftLimit::Request>();
    // 发送异步请求，然后等待返回
    auto result_future = m_get_soft_limit_client->async_send_request(request);
    auto result = result_future.get();

    joint_params = result->joint_params;
    return true;
}

bool H10wRosClient::get_joint_max_vel(std::vector<controller::msg::JointParams> &joint_params)
{
    /*等待服务端上线*/
    while (!m_get_joint_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    auto request = std::make_shared<controller::srv::GetJointMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_joint_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_joint_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    joint_params = result->joint_params;
    return true;
}

bool H10wRosClient::get_joint_max_acc(std::vector<controller::msg::JointParams> &joint_params)
{
    /*等待服务端上线*/
    while (!m_get_joint_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    auto request = std::make_shared<controller::srv::GetJointMaxAcc::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_joint_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_joint_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    joint_params = result->joint_params;
    return true;
}

bool H10wRosClient::get_cart_trans_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_trans_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianTranslationMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_trans_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_trans_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_cart_trans_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_trans_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianTranslationMaxAcc::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_trans_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_trans_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_cart_rota_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_rota_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianRotationMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_rota_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_rota_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_cart_rota_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_rota_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianRotationMaxAcc::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_rota_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_rota_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_joint_mech_limit(std::vector<controller::msg::JointParams> &joint_params)
{
    /*等待服务端上线*/
    while (
        !m_get_mech_limit_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    auto request =
        std::make_shared<controller::srv::GetJointMechanicalLimit::Request>();
    // 发送异步请求，然后等待返回
    auto result_future = m_get_mech_limit_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_mech_limit_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    joint_params = result->joint_params;
    return true;
}

bool H10wRosClient::get_joint_mech_max_vel(std::vector<controller::msg::JointParams> &joint_params)
{
    /*等待服务端上线*/
    while (
        !m_get_mech_max_vel_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request =
        std::make_shared<controller::srv::GetJointMechanicalMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future = m_get_mech_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_mech_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    joint_params = result->joint_params;
    return true;
}

bool H10wRosClient::get_joint_mech_max_acc(std::vector<controller::msg::JointParams> &joint_params)
{
    /*等待服务端上线*/
    while (
        !m_get_mech_max_acc_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request =
        std::make_shared<controller::srv::GetJointMechanicalMaxAcc::Request>();
    // 发送异步请求，然后等待返回
    auto result_future = m_get_mech_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_mech_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto result = result_future.get();
    joint_params = result->joint_params;
    return true;
}

bool H10wRosClient::get_cart_mech_trans_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_mech_trans_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianMechanicalTranslationMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_mech_trans_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_mech_trans_max_vel_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_cart_mech_trans_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_mech_trans_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianMechanicalTranslationMaxAcc::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_mech_trans_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_mech_trans_max_acc_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_cart_mech_rota_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_mech_rota_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianMechanicalRotationMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_mech_rota_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_mech_rota_max_vel_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_cart_mech_rota_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params)
{
    /*等待服务端上线*/
    while (!m_get_cart_mech_rota_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetCartesianMechanicalRotationMaxAcc::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_cart_mech_rota_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_cart_mech_rota_max_acc_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    cartesian_params = result->cartesian_params;
    return true;
}

bool H10wRosClient::get_tcp_offset(std::vector<int32_t> type, std::vector<TcpOffsetParams> &tcp_offset_params)
{
    /*等待服务端上线*/
    while (!m_get_tcp_offset_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetTcpOffset::Request>();
    request->type = type;

    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_tcp_offset_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_tcp_offset_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    tcp_offset_params = result->tcp_offset_params;
    return true;
}

bool H10wRosClient::get_tcp_payload(std::vector<int32_t> type, std::vector<TcpPayloadParams> &tcp_payload_params)
{
    /*等待服务端上线*/
    while (!m_get_tcp_payload_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetTcpPayload::Request>();
    request->type = type;

    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_tcp_payload_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_tcp_payload_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    tcp_payload_params = result->tcp_payload_params;
    return true;
}

bool H10wRosClient::set_joint_soft_limit(const std::vector<JointSoftLimitParams> params)
{
    /*等待服务端上线*/
    while (
        !m_set_soft_limit_client->wait_for_service(std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<SetJointSoftLimit::Request>();
    if (params.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &joint_param : params)
    {
        controller::msg::JointParams ddsParam;
        ddsParam.joint_index = joint_param.joint_index;
        ddsParam.max_pos = joint_param.max_pos;
        ddsParam.min_pos = joint_param.min_pos;
        request->joint_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future = m_set_soft_limit_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_soft_limit_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_joint_max_vel(const std::vector<JointMaxParams> max_vel)
{
    /*等待服务端上线*/
    while (!m_set_joint_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<SetJointMaxVel::Request>();
    if (max_vel.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &joint_param : max_vel)
    {
        controller::msg::JointParams ddsParam;
        ddsParam.joint_index = joint_param.joint_index;
        ddsParam.max_vel = joint_param.value;
        request->joint_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_joint_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_joint_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_joint_max_acc(const std::vector<JointMaxParams> max_acc)
{
    /*等待服务端上线*/
    while (!m_set_joint_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<controller::srv::SetJointMaxAcc::Request>();
    if (max_acc.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &joint_param : max_acc)
    {
        controller::msg::JointParams ddsParam;
        ddsParam.joint_index = joint_param.joint_index;
        ddsParam.max_acc = joint_param.value;
        request->joint_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_joint_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_joint_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_cart_trans_max_vel(const std::vector<CartMaxParams> max_vel)
{
    /*等待服务端上线*/
    while (!m_set_cart_trans_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetCartesianTranslationMaxVel::Request>();
    if (max_vel.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &cart_param : max_vel)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cart_param.cartesian_index;
        ddsParam.trans_max_vel = cart_param.value;
        request->cartesian_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_cart_trans_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_cart_trans_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_cart_trans_max_acc(const std::vector<CartMaxParams> max_acc)
{
    /*等待服务端上线*/
    while (!m_set_cart_trans_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetCartesianTranslationMaxAcc::Request>();
    if (max_acc.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &cart_param : max_acc)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cart_param.cartesian_index;
        ddsParam.trans_max_acc = cart_param.value;
        request->cartesian_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_cart_trans_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_cart_trans_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_cart_rota_max_vel(const std::vector<CartMaxParams> max_vel)
{
    /*等待服务端上线*/
    while (!m_set_cart_rota_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetCartesianRotationMaxVel::Request>();
    if (max_vel.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &cart_param : max_vel)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cart_param.cartesian_index;
        ddsParam.rota_max_vel = cart_param.value;
        request->cartesian_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_cart_rota_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_cart_rota_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_cart_rota_max_acc(const std::vector<CartMaxParams> max_acc)
{
    /*等待服务端上线*/
    while (!m_set_cart_rota_max_acc_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetCartesianRotationMaxAcc::Request>();
    if (max_acc.empty())
    {
        std::cout << "Invalid joint count: " << std::endl;
        return false;
    }
    for (const auto &cart_param : max_acc)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cart_param.cartesian_index;
        ddsParam.rota_max_acc = cart_param.value;
        request->cartesian_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_cart_rota_max_acc_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_cart_rota_max_acc_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_tcp_offset(const std::vector<TcpParam> tcp_offset_param)
{
    /*等待服务端上线*/
    while (!m_set_tcp_offset_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetTcpOffset::Request>();
    if (tcp_offset_param.empty())
    {
        std::cout << "Invalid joint count " << std::endl;
        return false;
    }

    for (const auto &offset : tcp_offset_param)
    {
        controller::msg::TcpOffsetParams ddsParam;
        ddsParam.type = offset.type;
        for (double val : offset.data)
        {
            ddsParam.offset.push_back(val);
        }
        request->tcp_offset_params.push_back(ddsParam);
    }

    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_tcp_offset_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_tcp_offset_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_tcp_payload(const std::vector<TcpParam> tcp_payload_param)
{
    /*等待服务端上线*/
    while (!m_set_tcp_payload_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetTcpPayload::Request>();
    if (tcp_payload_param.empty())
    {
        std::cout << "Invalid joint count " << std::endl;
        return false;
    }
    for (const auto &payload : tcp_payload_param)
    {
        controller::msg::TcpPayloadParams ddsParam;
        ddsParam.type = payload.type;
        for (double val : payload.data)
        {
            ddsParam.parameters.push_back(val);
        }
        request->tcp_payload_params.push_back(ddsParam);
    }
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_tcp_payload_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_tcp_payload_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::forward(const ForwardRequest joint, std::vector<TcpParam> &pose)
{
    /*等待服务端上线*/
    while (!m_forward_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::ForwardKinematics::Request>();

    for (int t : joint.type)
        request->type.push_back(t); // 添加元素
    for (double angle : joint.joint_angles)
        request->joint_angles.push_back(angle);

    // 发送异步请求，然后等待返回
    auto result_future =
        m_forward_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_forward_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();

    auto &tcpPoseParams = response->pose;

    for (const auto &tcpPoseParam : tcpPoseParams)
    {
        std::vector<double> poseVec(
            tcpPoseParam.pose.begin(),
            tcpPoseParam.pose.end());
        TcpParam rosParam(tcpPoseParam.type, poseVec);
        pose.push_back(rosParam);
    }
    return true;
}

bool H10wRosClient::inverse(const KinematicsParams params, std::vector<double> &joint)
{
    /*等待服务端上线*/
    while (!m_inverse_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    if (params.pose.empty() || params.type.empty())
    {
        std::cerr << "Empty pose input" << std::endl;
        return false;
    }
    // 构造请求
    auto request = std::make_shared<
        controller::srv::InverseKinematics::Request>();

    for (int i = 0; i < params.type.size(); i++)
    {
        controller::msg::TcpPoseParams ddsParam;
        ddsParam.type = params.type[i];
        for (double val : params.pose[i])
        {
            ddsParam.pose.push_back(val);
        };
        request->pose.push_back(ddsParam);
    }
    for (double angle : params.joint_angle)
        request->reference_joint_angles.push_back(angle);
    request->if_use_whole_body = params.if_use_whole_body;

    // 发送异步请求，然后等待返回
    auto result_future =
        m_inverse_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_inverse_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    auto jointAngles = response->joint_angles;
    for (int i = 0; i < jointAngles.size(); i++)
    {
        joint.emplace_back(jointAngles[i]);
    }
    return true;
}

bool H10wRosClient::enable_controller(bool m_enable)
{
    /*等待服务端上线*/
    while (!m_enable_controller_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::EnableController::Request>();
    request->enable = m_enable;

    // 发送异步请求，然后等待返回
    auto result_future =
        m_enable_controller_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_enable_controller_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::set_chassis_max_vel(double linear_vel, double angular_vel)
{
    /*等待服务端上线*/
    while (!m_set_chassis_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetChassisMaxVel::Request>();
    request->linear_velocity = linear_vel;
    request->angular_velocity = angular_vel;
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_chassis_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_chassis_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::get_chassis_max_vel(double &linear_vel, double &angular_vel)
{
    /*等待服务端上线*/
    while (!m_get_chassis_max_vel_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetChassisMaxVel::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_chassis_max_vel_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_chassis_max_vel_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    linear_vel = response->linear_velocity;
    angular_vel = response->angular_velocity;
    return true;
}

bool H10wRosClient::set_control_policy(int32_t policy)
{
    /*等待服务端上线*/
    while (!m_set_control_policy_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetControlPolicy::Request>();
    request->policy = policy;
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_control_policy_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_control_policy_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::get_control_policy(int32_t &policy)
{
    /*等待服务端上线*/
    while (!m_get_control_policy_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetControlPolicy::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_control_policy_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_control_policy_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    policy = response->policy;
    return true;
}

bool H10wRosClient::set_safe_mode(bool safe_mode)
{
    /*等待服务端上线*/
    while (!m_set_safe_mode_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = std::make_shared<
        controller::srv::SetSafeMode::Request>();
    request->enable = safe_mode;
    // 发送异步请求，然后等待返回
    auto result_future =
        m_set_safe_mode_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_set_safe_mode_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    return response->success;
}

bool H10wRosClient::get_safe_mode(bool &safe_mode)
{
    /*等待服务端上线*/
    while (!m_get_safe_mode_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 构造请求
    auto request = std::make_shared<
        controller::srv::GetSafeMode::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_get_safe_mode_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_safe_mode_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    safe_mode = response->enable;
    return true;
}

bool H10wRosClient::is_enabled_controller(bool &is_enabled)
{
    /*等待服务端上线*/
    while (!m_is_enable_controller_client->wait_for_service(
        std::chrono::seconds(1)))
    {
        // 等待时检测rclcpp的状态
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 构造请求
    auto request = std::make_shared<
        controller::srv::IsEnabledController::Request>();
    // 发送异步请求，然后等待返回
    auto result_future =
        m_is_enable_controller_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_is_enable_controller_client->remove_pending_request(result_future);
        return false;
    }
    auto response = result_future.get();
    is_enabled = response->enable;
    return response->success;
}
