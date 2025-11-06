#include <cassert>
#include <cmath>
#include <iostream>
#include <array>
#include <chrono>
#include <thread>

#include "H10wGrpcMove.h"

H10wGrpcMove::H10wGrpcMove(const std::string &strIpPort)
    : Node("H10wGrpcMove"),
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
        std::bind(&H10wGrpcMove::move_callback, this,
                  std::placeholders::_1));

    error_subscriber_ = this->create_subscription<controller::msg::ErrorMessage>(
        "error_message", qos,
        std::bind(&H10wGrpcMove::error_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveSubscriber 节点已启动，等待消息...");
}
void H10wGrpcMove::move_callback(const controller::msg::MoveMessage::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(msg_mutex_);
    get_move_msg_ = msg;
}
void H10wGrpcMove::error_callback(const controller::msg::ErrorMessage::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(msg_mutex_);
    get_error_msg_ = msg;
}
bool H10wGrpcMove::grpc_singlemove(int32_t index, float &position, float &velocity_percent, uint32_t &token)
{
    m_pControllerClient->SingleJointMove(index, position, velocity_percent, token);
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
                         "grpc_singlemove() 超时（%d秒），操作未完成。当前状态: token=%u, state=%d, 目标token=%u",
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
                        "grpc_singlemove() 完成。token=%u, state=%d",
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

        // 等待一段时间后再次检查（避免CPU占用过高）
        std::this_thread::sleep_for(check_interval);
    }
}
bool H10wGrpcMove::grpc_multimove(const std::vector<uint32_t> &joint_index, const std::vector<float> &position, const std::vector<float> &velocity_percent, uint32_t &token)
{
    m_pControllerClient->MultiJointsMove(joint_index, position, velocity_percent, token);

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
                         "grpc_multimove() 超时（%d秒），操作未完成。当前状态: token=%u, state=%d, 目标token=%u",
                         static_cast<int>(timeout.count()),
                         get_move_msg_->token,
                         get_move_msg_->state,
                         token);
            return false;
        }

        // 检查是否满足完成条件
        if (get_move_msg_->state == 0 && get_move_msg_->token == token)
        {
            RCLCPP_INFO(this->get_logger(),
                        "grpc_multimove() 完成。token=%u, state=%d",
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

        // 等待一段时间后再次检查（避免CPU占用过高）
        std::this_thread::sleep_for(check_interval);
    }
}
bool H10wGrpcMove::grpc_linearmove(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, const std::vector<float> velocity_percent, std::vector<float> acceleration_percent, int32_t &task_id)
{
    m_pControllerClient->LinearMovel(type, pose, velocity_percent, acceleration_percent, task_id);
    // 定义超时时间为5秒
    const auto timeout = std::chrono::seconds(5);
    // 记录开始时间
    const auto start_time = std::chrono::steady_clock::now();
    // 循环检查间隔（100毫秒）
    const auto check_interval = std::chrono::milliseconds(100);
    // 循环检查直到满足条件或超时
    while (true)
    {
        bool all_reached = true;
        // 获取当前时间
        const auto now = std::chrono::steady_clock::now();
        // 检查是否超时
        if (now - start_time >= timeout)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "grpc_linearmove() 超时（%d秒），操作未完成。当前状态: state=%d,", static_cast<int>(timeout.count()),
                         get_move_msg_->state);
            return false;
        }

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

        // 等待一段时间后再次检查（避免CPU占用过高）
        std::this_thread::sleep_for(check_interval);
    }
}
