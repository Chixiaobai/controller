#pragma once

#include <rclcpp/rclcpp.hpp>
#include "controller/srv/error_clear.hpp"
#include "controller/msg/move_message.hpp"
#include "controller/msg/error_message.hpp"
#include "H10Wglobalconstants.h"
#include "controller/srv/multi_joint_move.hpp"
#include "controller/srv/single_joint_move.hpp"
#include "controller/srv/linear_move.hpp"
#include "controller/srv/enable_realtime_cmd.hpp"
#include "controller/msg/real_time_body_joints.hpp"
#include "controller/msg/real_time_body_tcp_cartesian.hpp"
#include "controller/srv/get_version.hpp"
#include "controller/msg/joint_params.hpp"
#include "controller/msg/cartesian_params.hpp"
#include "controller/srv/get_joint_max_vel.hpp"
#include "controller/srv/get_joint_max_acc.hpp"
#include "controller/srv/get_joint_soft_limit.hpp"
#include "controller/srv/get_cartesian_translation_max_vel.hpp"
#include "controller/srv/get_cartesian_translation_max_acc.hpp"
#include "controller/srv/get_cartesian_rotation_max_vel.hpp"
#include "controller/srv/get_cartesian_rotation_max_acc.hpp"
#include "controller/srv/get_joint_mechanical_limit.hpp"
#include "controller/srv/set_joint_max_vel.hpp"
#include "controller/srv/set_joint_max_acc.hpp"
#include "controller/srv/set_joint_soft_limit.hpp"
#include "controller/srv/set_cartesian_translation_max_vel.hpp"
#include "controller/srv/set_cartesian_translation_max_acc.hpp"
#include "controller/srv/set_cartesian_rotation_max_vel.hpp"
#include "controller/srv/set_cartesian_rotation_max_acc.hpp"
#include "controller/srv/set_tcp_offset.hpp"
#include "controller/srv/set_tcp_payload.hpp"
#include "controller/srv/forward_kinematics.hpp"
#include "controller/srv/inverse_kinematics.hpp"
#include "controller/srv/get_joint_mechanical_max_vel.hpp"
#include "controller/srv/get_joint_mechanical_max_acc.hpp"
#include "controller/srv/get_cartesian_mechanical_translation_max_vel.hpp"
#include "controller/srv/get_cartesian_mechanical_translation_max_acc.hpp"
#include "controller/srv/get_cartesian_mechanical_rotation_max_vel.hpp"
#include "controller/srv/get_cartesian_mechanical_rotation_max_acc.hpp"
#include "controller/srv/get_tcp_offset.hpp"
#include "controller/srv/get_tcp_payload.hpp"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"

using namespace GlobalConstants::H10W;
// joint_index, target_position, velocity
using JointTarget = std::tuple<uint32_t, double, double>;
// joint_index, max_pos, min_pos
using JointLimit = std::vector<std::tuple<uint32_t, double, double>>;
// index, max_value
using JointMaxAcc = std::vector<std::tuple<uint32_t, double>>;
using JointMaxVel = std::vector<std::tuple<uint32_t, double>>;
using CartRotaAcc = std::vector<std::tuple<uint32_t, double>>;
using CartRotaVel = std::vector<std::tuple<uint32_t, double>>;
using CartTransAcc = std::vector<std::tuple<uint32_t, double>>;
using CartTransVel = std::vector<std::tuple<uint32_t, double>>;

struct ControllerVersion
{
    std::string main;
    std::map<std::string, std::string> plugins;
};

class H10wRosClient : public rclcpp::Node
{
public:
    explicit H10wRosClient(const std::string &strIpPort = "localhost");
    ~H10wRosClient() = default;

    bool has_move_msg() const { return get_move_msg_ != nullptr; }

private:
    void move_callback(const controller::msg::MoveMessage::SharedPtr msg);

    void error_callback(const controller::msg::ErrorMessage::SharedPtr msg);

public:
    bool ros_singlemove(const uint32_t joint_index, const float target_position, const float velocity);

    bool ros_multimove(const std::vector<int32_t> &joint_indices, const std::vector<float> &target_positions, const std::vector<float> &velocities);

    bool ros_linearmove(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, const std::vector<float> velocity_percent, std::vector<float> acceleration_percent);

    bool enable_realtime_cmd(bool m_enable);

    bool clear_error();

    bool servoj(controller::msg::RealTimeBodyJoints msg, double m_step);

    bool servol(controller::msg::RealTimeBodyTcpCartesian msg, double m_step);

    bool speedj(controller::msg::RealTimeBodyJoints msg, int32_t t);

    bool speedl(controller::msg::RealTimeBodyTcpCartesian msg, int32_t t);

    bool get_version(ControllerVersion &version);

    bool get_joint_soft_limit(std::vector<controller::msg::JointParams> &joint_params);

    bool get_joint_max_vel(std::vector<controller::msg::JointParams> &joint_params);

    bool get_joint_max_acc(std::vector<controller::msg::JointParams> &joint_params);

    bool get_cart_trans_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_cart_trans_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_cart_rota_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_cart_rota_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_joint_mech_limit(std::vector<controller::msg::JointParams> &joint_params);

    bool get_joint_mech_max_vel(std::vector<controller::msg::JointParams> &joint_params);

    bool get_joint_mech_max_acc(std::vector<controller::msg::JointParams> &joint_params);

    bool get_cart_mech_trans_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_cart_mech_trans_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_cart_mech_rota_max_vel(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_cart_mech_rota_max_acc(std::vector<controller::msg::CartesianParams> &cartesian_params);

    bool get_tcp_offset(std::vector<int32_t> &type, std::vector<controller::msg::TcpOffsetParams> &tcp_offset_params);

    bool get_tcp_payload(std::vector<int32_t> &type, std::vector<controller::msg::TcpPayloadParams> &tcp_payload_params);

    bool set_joint_soft_limit(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_pos, const std::vector<double> &min_pos);

    bool set_joint_max_vel(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_vel);

    bool set_joint_max_acc(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_acc);

    bool set_cart_trans_max_vel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_vel);

    bool set_cart_trans_max_acc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_acc);

    bool set_cart_rota_max_vel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_vel);

    bool set_cart_rota_max_acc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_acc);

    bool set_tcp_offset(const std::vector<int32_t> &type, std::vector<std::vector<double>> &offset);

    bool set_tcp_payload(const std::vector<int32_t> &type, std::vector<std::vector<double>> &payload);

    std::vector<std::pair<int32_t, std::vector<double>>> forward(const std::vector<int32_t> &type, std::vector<double> &joint_angles);

    std::vector<double> inverse(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, bool if_use_whole_body);

public:
    std::shared_ptr<grpc::Channel> m_channel;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::unique_ptr<HumanoidControllerClient> m_pControllerClient;
    controller::msg::MoveMessage::SharedPtr get_move_msg_ = nullptr;
    controller::msg::ErrorMessage::SharedPtr get_error_msg_ = nullptr;

protected:
    rclcpp::Client<controller::srv::ErrorClear>::SharedPtr m_clear_error_client;
    rclcpp::Subscription<controller::msg::MoveMessage>::SharedPtr
        move_subscriber_;
    rclcpp::Subscription<controller::msg::ErrorMessage>::SharedPtr
        error_subscriber_;
    rclcpp::Client<controller::srv::MultiJointMove>::SharedPtr
        m_multi_move_client;
    rclcpp::Client<controller::srv::SingleJointMove>::SharedPtr
        m_single_move_client;
    rclcpp::Client<controller::srv::LinearMove>::SharedPtr
        m_linear_move_client;
    rclcpp::Client<controller::srv::EnableRealtimeCmd>::SharedPtr
        m_enable_realtime_cmd;
    rclcpp::Publisher<controller::msg::RealTimeBodyJoints>::SharedPtr
        m_servo_body_j_publisher;
    rclcpp::Publisher<controller::msg::RealTimeBodyTcpCartesian>::SharedPtr
        m_servo_body_l_publisher;
    rclcpp::Publisher<controller::msg::RealTimeBodyJoints>::SharedPtr
        m_speed_body_j_publisher;
    rclcpp::Publisher<controller::msg::RealTimeBodyTcpCartesian>::SharedPtr
        m_speed_body_l_publisher;
    std::mutex msg_mutex_;
    std::atomic<bool> move_done_{false};

    rclcpp::Client<controller::srv::GetVersion>::SharedPtr m_get_version_client;
    rclcpp::Client<controller::srv::GetJointSoftLimit>::SharedPtr
        m_get_soft_limit_client;
    rclcpp::Client<controller::srv::GetJointMaxVel>::SharedPtr
        m_get_joint_max_vel_client;
    rclcpp::Client<controller::srv::GetJointMaxAcc>::SharedPtr
        m_get_joint_max_acc_client;
    rclcpp::Client<controller::srv::GetCartesianTranslationMaxVel>::SharedPtr
        m_get_cart_trans_max_vel_client;
    rclcpp::Client<controller::srv::GetCartesianTranslationMaxAcc>::SharedPtr
        m_get_cart_trans_max_acc_client;
    rclcpp::Client<controller::srv::GetCartesianRotationMaxVel>::SharedPtr
        m_get_cart_rota_max_vel_client;
    rclcpp::Client<controller::srv::GetCartesianRotationMaxAcc>::SharedPtr
        m_get_cart_rota_max_acc_client;
    rclcpp::Client<controller::srv::GetJointMechanicalLimit>::SharedPtr
        m_get_mech_limit_client;
    rclcpp::Client<controller::srv::GetJointMechanicalMaxVel>::SharedPtr
        m_get_mech_max_vel_client;
    rclcpp::Client<controller::srv::GetJointMechanicalMaxAcc>::SharedPtr
        m_get_mech_max_acc_client;
    rclcpp::Client<controller::srv::GetCartesianMechanicalTranslationMaxVel>::
        SharedPtr m_get_cart_mech_trans_max_vel_client;
    rclcpp::Client<controller::srv::GetCartesianMechanicalTranslationMaxAcc>::
        SharedPtr m_get_cart_mech_trans_max_acc_client;
    rclcpp::Client<controller::srv::GetCartesianMechanicalRotationMaxVel>::
        SharedPtr m_get_cart_mech_rota_max_vel_client;
    rclcpp::Client<controller::srv::GetCartesianMechanicalRotationMaxAcc>::
        SharedPtr m_get_cart_mech_rota_max_acc_client;
    rclcpp::Client<controller::srv::GetTcpOffset>::
        SharedPtr m_get_tcp_offset_client;
    rclcpp::Client<controller::srv::GetTcpPayload>::
        SharedPtr m_get_tcp_paylaod_client;

    rclcpp::Client<controller::srv::SetJointSoftLimit>::SharedPtr
        m_set_soft_limit_client;
    rclcpp::Client<controller::srv::SetJointMaxVel>::SharedPtr
        m_set_joint_max_vel_client;
    rclcpp::Client<controller::srv::SetJointMaxAcc>::SharedPtr
        m_set_joint_max_acc_client;
    rclcpp::Client<controller::srv::SetCartesianTranslationMaxVel>::SharedPtr
        m_set_cart_trans_max_vel_client;
    rclcpp::Client<controller::srv::SetCartesianTranslationMaxAcc>::SharedPtr
        m_set_cart_trans_max_acc_client;
    rclcpp::Client<controller::srv::SetCartesianRotationMaxVel>::SharedPtr
        m_set_cart_rota_max_vel_client;
    rclcpp::Client<controller::srv::SetCartesianRotationMaxAcc>::SharedPtr
        m_set_cart_rota_max_acc_client;
    rclcpp::Client<controller::srv::SetTcpOffset>::SharedPtr
        m_set_tcp_offset_client;
    rclcpp::Client<controller::srv::SetTcpPayload>::SharedPtr
        m_set_tcp_payload_client;
    rclcpp::Client<controller::srv::ForwardKinematics>::SharedPtr
        m_forward_client;
    rclcpp::Client<controller::srv::InverseKinematics>::SharedPtr
        m_inverse_client;
};