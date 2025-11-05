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
      m_pControllerClient(std::make_unique<HumanoidControllerClient>(
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
    m_get_tcp_paylaod_client =
        this->create_client<controller::srv::GetTcpPayload>(
            "/h10w/controller/get_tcp_paylaod");

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
bool H10wRosClient::ros_singlemove(const uint32_t joint_index, const float target_position, const float velocity)
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
    request->joint_index = joint_index;
    request->target_position = target_position;
    request->velocity = velocity;

    auto result_future = m_single_move_client->async_send_request(request);
    // 等待服务响应
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "收到移动结果：%d", response->success);

    // 设置超时时间（例如5秒，可根据实际情况调整）
    const int timeout_ms = 5000;
    const int check_interval_ms = 10;
    int elapsed_ms = 0;

    while (true)
    {
        // 检查是否达到目标位置
        if (isFloatValueEqual(get_move_msg_->position[joint_index - 1], target_position, 0.001) && get_move_msg_->state == 0)
        {
            std::cout << "运动完成" << "\n";
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

        // 检查是否超时
        if (elapsed_ms >= timeout_ms)
        {
            RCLCPP_ERROR(this->get_logger(), "运动超时！目标位置: %.3f, 当前位置: %.3f",
                         target_position, get_move_msg_->position[joint_index - 1]);
            // 可以根据需要添加超时处理逻辑，如强制停止等
            return false; // 超时退出
        }

        // 休眠一小段时间再检查
        sleepMilliseconds(check_interval_ms);
        elapsed_ms += check_interval_ms;

        // 检查系统状态是否正常
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "系统状态异常，退出运动检查");
            return false;
        }
    }
}

bool H10wRosClient::ros_multimove(const std::vector<int32_t> &joint_indices, const std::vector<float> &target_positions, const std::vector<float> &velocities)
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
    if (joint_indices.size() != target_positions.size() ||
        joint_indices.size() != velocities.size())
    {
        RCLCPP_ERROR(this->get_logger(), "关节参数向量长度不匹配");
        return false;
    }
    if (joint_indices.empty())
    {
        RCLCPP_WARN(this->get_logger(), "未提供任何关节运动参数");
        return false;
    }

    std::vector<controller::msg::JointAngle> joint_angles;
    // 遍历参数构造关节角度信息
    for (size_t i = 0; i < joint_indices.size(); ++i)
    {
        controller::msg::JointAngle joint_angle;
        joint_angle.joint_index = joint_indices[i];
        joint_angle.target_position = target_positions[i];
        joint_angle.velocity = velocities[i];
        joint_angles.emplace_back(joint_angle);
    }

    // 构造请求
    auto request = std::make_shared<controller::srv::MultiJointMove::Request>();
    request->joint_angles = joint_angles;

    auto result_future = m_multi_move_client->async_send_request(request);
    auto response = result_future.get(); // 阻塞直到结果返回

    // 设置超时时间（例如8秒，多关节运动可适当延长）
    const int timeout_ms = 8000;
    const int check_interval_ms = 10;
    int elapsed_ms = 0;

    while (true)
    {
        bool all_joints_reached = true;

        // 检查所有关节是否都达到目标位置且状态正常
        for (size_t i = 0; i < joint_indices.size(); ++i)
        {
            int32_t joint_idx = joint_indices[i];
            double target_pos = target_positions[i];

            // 检查单个关节是否到位
            if (!isFloatValueEqual(get_move_msg_->position[joint_idx - 1], target_pos, 0.001) || get_move_msg_->state != 0)
            {
                all_joints_reached = false;
                break; // 只要有一个关节未到位，就跳出检查循环
            }
        }

        // 如果所有关节都到位，退出等待循环
        if (all_joints_reached)
        {
            std::cout << "所有关节运动完成" << "\n";
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

        // 检查是否超时
        if (elapsed_ms >= timeout_ms)
        {
            RCLCPP_ERROR(this->get_logger(), "运动超时！部分关节未达到目标位置");
            // 输出未到位的关节信息，便于调试
            for (size_t i = 0; i < joint_indices.size(); ++i)
            {
                int32_t joint_idx = joint_indices[i];
                double target_pos = target_positions[i];
                if (!isFloatValueEqual(get_move_msg_->position[joint_idx - 1], target_pos, 0.001))
                {
                    RCLCPP_ERROR(this->get_logger(), "关节 %d: 目标位置 %.3f, 当前位置 %.3f",
                                 joint_idx, target_pos, get_move_msg_->position[joint_idx - 1]);
                }
            }
            return false; // 超时退出
        }

        // 休眠一小段时间再检查
        sleepMilliseconds(check_interval_ms);
        elapsed_ms += check_interval_ms;

        // 检查系统状态是否正常
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "系统状态异常，退出运动检查");
            return false;
        }
    }
}

bool H10wRosClient::ros_linearmove(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, const std::vector<float> velocity_percent, std::vector<float> acceleration_percent)
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
    for (size_t i = 0; i < type.size(); ++i)
    {
        controller::msg::LinearMoveParams linear_param;
        linear_param.type = type[i];
        linear_param.pose = pose[i];
        linear_param.velocity_percent = velocity_percent[i];
        linear_param.acceleration_percent = acceleration_percent[i];
        linear_params.emplace_back(linear_param);
    }
    // 构造请求
    auto request =
        std::make_shared<controller::srv::LinearMove::Request>();
    request->linear_move = linear_params;
    auto result_future = m_linear_move_client->async_send_request(request);
    // 等待服务响应
    auto response = result_future.get();

    // 设置超时时间（例如5秒，可根据实际情况调整）
    const int timeout_ms = 5000;
    const int check_interval_ms = 10;
    int elapsed_ms = 0;

    while (true)
    {
        bool all_reached = true;
        // 检查是否满足完成条件
        for (int i = 0; i < type.size(); i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (!isFloatValueEqual(get_move_msg_->tcp_pose[type[i] - 1].pose[j], pose[i][j], 0.001))
                {
                    all_reached = false;
                    break;
                }
            }
        }
        if (all_reached && get_move_msg_->state == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "grpc_linearmove() 完成");
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

        // 检查是否超时
        if (elapsed_ms >= timeout_ms)
        {
            RCLCPP_ERROR(this->get_logger(), "运动超时！");
            // 可以根据需要添加超时处理逻辑，如强制停止等
            return false; // 超时退出
        }

        // 休眠一小段时间再检查
        sleepMilliseconds(check_interval_ms);
        elapsed_ms += check_interval_ms;

        // 检查系统状态是否正常
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "系统状态异常，退出运动检查");
            return false;
        }
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
    std::cout << "1111111111\n";
    auto result = result_future.get();
    std::cout << "2222222222222\n";
    std::cout << result << "\n";

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

bool H10wRosClient::get_tcp_offset(std::vector<int32_t> &type, std::vector<controller::msg::TcpOffsetParams> &tcp_offset_params)
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

bool H10wRosClient::get_tcp_payload(std::vector<int32_t> &type, std::vector<controller::msg::TcpPayloadParams> &tcp_payload_params)
{
    /*等待服务端上线*/
    while (!m_get_tcp_paylaod_client->wait_for_service(
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
        m_get_tcp_paylaod_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_get_tcp_paylaod_client->remove_pending_request(
            result_future);
        return false;
    }
    auto result = result_future.get();
    tcp_payload_params = result->tcp_payload_params;
    return true;
}

bool H10wRosClient::set_joint_soft_limit(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_pos, const std::vector<double> &min_pos)
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
    auto request = std::make_shared<controller::srv::SetJointSoftLimit::Request>();
    if (joint_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << joint_index.size() << std::endl;
        return false;
    }
    if (max_pos.size() != joint_index.size() ||
        min_pos.size() != joint_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < joint_index.size(); i++)
    {
        controller::msg::JointParams ddsParam;
        ddsParam.joint_index = joint_index[i];
        ddsParam.max_pos = max_pos[i];
        ddsParam.min_pos = min_pos[i];
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

bool H10wRosClient::set_joint_max_vel(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_vel)
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
    auto request = std::make_shared<controller::srv::SetJointMaxVel::Request>();
    if (joint_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << joint_index.size() << std::endl;
        return false;
    }
    if (max_vel.size() != joint_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < joint_index.size(); i++)
    {
        controller::msg::JointParams ddsParam;
        ddsParam.joint_index = joint_index[i];
        ddsParam.max_vel = max_vel[i];
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

bool H10wRosClient::set_joint_max_acc(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_acc)
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
    if (joint_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << joint_index.size() << std::endl;
        return false;
    }
    if (max_acc.size() != joint_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < joint_index.size(); i++)
    {
        controller::msg::JointParams ddsParam;
        ddsParam.joint_index = joint_index[i];
        ddsParam.max_acc = max_acc[i];
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

bool H10wRosClient::set_cart_trans_max_vel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_vel)
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
    if (cartesian_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << cartesian_index.size() << std::endl;
        return false;
    }
    if (trans_max_vel.size() != cartesian_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < cartesian_index.size(); i++)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cartesian_index[i];
        ddsParam.trans_max_vel = trans_max_vel[i];
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

bool H10wRosClient::set_cart_trans_max_acc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_acc)
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
    if (cartesian_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << cartesian_index.size() << std::endl;
        return false;
    }
    if (trans_max_acc.size() != cartesian_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < cartesian_index.size(); i++)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cartesian_index[i];
        ddsParam.trans_max_acc = trans_max_acc[i];
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

bool H10wRosClient::set_cart_rota_max_vel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_vel)
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
    if (cartesian_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << cartesian_index.size() << std::endl;
        return false;
    }
    if (rota_max_vel.size() != cartesian_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < cartesian_index.size(); i++)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cartesian_index[i];
        ddsParam.rota_max_vel = rota_max_vel[i];
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

bool H10wRosClient::set_cart_rota_max_acc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_acc)
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
    if (cartesian_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << cartesian_index.size() << std::endl;
        return false;
    }
    if (rota_max_acc.size() != cartesian_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < cartesian_index.size(); i++)
    {
        controller::msg::CartesianParams ddsParam;
        ddsParam.cartesian_index = cartesian_index[i];
        ddsParam.rota_max_acc = rota_max_acc[i];
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

bool H10wRosClient::set_tcp_offset(const std::vector<int32_t> &type, std::vector<std::vector<double>> &offset)
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
    if (type.size() <= 0)
    {
        std::cout << "Invalid joint count: " << type.size() << std::endl;
        return false;
    }
    if (offset.size() != type.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < type.size(); i++)
    {
        controller::msg::TcpOffsetParams ddsParam;
        ddsParam.type = type[i];
        ddsParam.offset = offset[i];
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

bool H10wRosClient::set_tcp_payload(const std::vector<int32_t> &type, std::vector<std::vector<double>> &payload)
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
    if (type.size() <= 0)
    {
        std::cout << "Invalid joint count: " << type.size() << std::endl;
        return false;
    }
    if (payload.size() != type.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    for (int i = 0; i < type.size(); i++)
    {
        controller::msg::TcpPayloadParams ddsParam;
        ddsParam.type = type[i];
        ddsParam.parameters = payload[i];
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

std::vector<std::pair<int32_t, std::vector<double>>> H10wRosClient::forward(const std::vector<int32_t> &type, std::vector<double> &joint_angles)
{
    std::vector<std::pair<int32_t, std::vector<double>>> result;
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
    request->type = type;
    request->joint_angles = joint_angles;

    // 发送异步请求，然后等待返回
    auto result_future =
        m_forward_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_forward_client->remove_pending_request(result_future);
    }
    auto response = result_future.get();
    auto tcpPoseParams = response->pose;
    for (auto tcpPoseParam : tcpPoseParams)
    {
        std::vector<double> poseVec(
            tcpPoseParam.pose.begin(),
            tcpPoseParam.pose.end());
        result.emplace_back(tcpPoseParam.type, poseVec);
    }
    return result;
}

std::vector<double> H10wRosClient::inverse(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, bool if_use_whole_body)
{
    std::vector<double> result;
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

    // 构造请求
    auto request = std::make_shared<
        controller::srv::InverseKinematics::Request>();
    if (type.size() <= 0)
    {
        std::cout << "Invalid joint count: " << type.size() << std::endl;
    }
    if (pose.size() != type.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
    }
    for (int i = 0; i < type.size(); i++)
    {
        controller::msg::TcpPoseParams ddsParam;
        ddsParam.type = type[i];
        ddsParam.pose = pose[i];
        request->pose.push_back(ddsParam);
    }
    request->if_use_whole_body = if_use_whole_body;

    // 发送异步请求，然后等待返回
    auto result_future =
        m_inverse_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败 :(");
        m_inverse_client->remove_pending_request(result_future);
    }
    auto response = result_future.get();
    auto jointAngles = response->joint_angles;
    for (int i = 0; i < jointAngles.size(); i++)
    {
        result.emplace_back(jointAngles[i]);
    }
    return result;
}