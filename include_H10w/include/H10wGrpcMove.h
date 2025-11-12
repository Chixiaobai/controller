#pragma once

#include <rclcpp/rclcpp.hpp>
#include "controller/msg/move_message.hpp"
#include "controller/msg/error_message.hpp"
#include "controller/srv/multi_joint_move.hpp"
#include "controller/srv/single_joint_move.hpp"

#include "DeviceControlServiceClient.h"
#include "H10Wglobalconstants.h"
#include "H10wGrpcParam.h"

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
    bool grpc_singlemove(const MoveParams params, uint32_t &token );

    bool grpc_linearmove(const std::vector<LinearMoveParams> params, uint32_t &token );

    bool grpc_multimove(const std::vector<MoveParams> params, uint32_t &token );

public:
    std::shared_ptr<grpc::Channel> m_channel;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::unique_ptr<H10WGrpcParam> m_pControllerClient;
    controller::msg::MoveMessage::SharedPtr get_move_msg_ = nullptr;
    controller::msg::ErrorMessage::SharedPtr get_error_msg_ = nullptr;

protected:
    rclcpp::Subscription<controller::msg::MoveMessage>::SharedPtr
        move_subscriber_;
    rclcpp::Subscription<controller::msg::ErrorMessage>::SharedPtr
        error_subscriber_;
    std::mutex msg_mutex_;
};
