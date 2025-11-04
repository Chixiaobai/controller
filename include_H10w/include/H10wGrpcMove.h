#pragma once

#include <rclcpp/rclcpp.hpp>
#include "controller/msg/move_message.hpp"
#include "controller/msg/error_message.hpp"
#include "controller/srv/multi_joint_move.hpp"
#include "controller/srv/single_joint_move.hpp"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
#include "H10Wglobalconstants.h"

using namespace GlobalConstants::H10W;

class H10wGrpcMove : public rclcpp::Node
{
public:
    explicit H10wGrpcMove(const std::string &strIpPort = "localhost");
    ~H10wGrpcMove() = default;

    bool has_move_msg() const { return get_move_msg_ != nullptr; }

private:
    void move_callback(const controller::msg::MoveMessage::SharedPtr msg);
    void error_callback(const controller::msg::ErrorMessage::SharedPtr msg);

public:
    void grpc_singlemove(int32_t index, float &position, float &velocity_percent, uint32_t &token);
    void grpc_multimove(const std::vector<int32_t> &joint_index, const std::vector<float> &position, const std::vector<float> &velocity_percent, uint32_t &token);
    void grpc_linearmove(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, const std::vector<float> velocity_percent, std::vector<float> acceleration_percent, int32_t &task_id);

public:
    std::shared_ptr<grpc::Channel> m_channel;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::unique_ptr<HumanoidControllerClient> m_pControllerClient;
    controller::msg::MoveMessage::SharedPtr get_move_msg_ = nullptr;
    controller::msg::ErrorMessage::SharedPtr get_error_msg_ = nullptr;

protected:
    rclcpp::Subscription<controller::msg::MoveMessage>::SharedPtr
        move_subscriber_;
    rclcpp::Subscription<controller::msg::ErrorMessage>::SharedPtr
        error_subscriber_;
    std::mutex msg_mutex_;
};