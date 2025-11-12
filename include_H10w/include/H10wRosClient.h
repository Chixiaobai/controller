#pragma once

#include <rclcpp/rclcpp.hpp>
#include "controller/srv/enable_realtime_cmd.hpp"
#include "controller/srv/error_clear.hpp"
#include "H10Wglobalconstants.h"
#include "controller/srv/forward_kinematics.hpp"
#include "controller/srv/inverse_kinematics.hpp"
#include "controller/srv/get_cartesian_mechanical_translation_max_vel.hpp"
#include "controller/srv/get_cartesian_mechanical_translation_max_acc.hpp"
#include "controller/srv/get_cartesian_mechanical_rotation_max_vel.hpp"
#include "controller/srv/get_cartesian_mechanical_rotation_max_acc.hpp"
#include "controller/srv/get_cartesian_translation_max_vel.hpp"
#include "controller/srv/get_cartesian_translation_max_acc.hpp"
#include "controller/srv/get_cartesian_rotation_max_vel.hpp"
#include "controller/srv/get_cartesian_rotation_max_acc.hpp"
#include "controller/srv/get_joint_max_vel.hpp"
#include "controller/srv/get_joint_max_acc.hpp"
#include "controller/srv/get_joint_mechanical_limit.hpp"
#include "controller/srv/get_joint_mechanical_max_vel.hpp"
#include "controller/srv/get_joint_mechanical_max_acc.hpp"
#include "controller/srv/get_joint_soft_limit.hpp"
#include "controller/srv/get_tcp_offset.hpp"
#include "controller/srv/get_tcp_payload.hpp"
#include "controller/srv/get_version.hpp"
#include "controller/srv/multi_joint_move.hpp"
#include "controller/srv/single_joint_move.hpp"
#include "controller/srv/linear_move.hpp"
#include "controller/srv/set_cartesian_translation_max_vel.hpp"
#include "controller/srv/set_cartesian_translation_max_acc.hpp"
#include "controller/srv/set_cartesian_rotation_max_vel.hpp"
#include "controller/srv/set_cartesian_rotation_max_acc.hpp"
#include "controller/srv/set_joint_max_vel.hpp"
#include "controller/srv/set_joint_max_acc.hpp"
#include "controller/srv/set_joint_soft_limit.hpp"
#include "controller/srv/set_tcp_offset.hpp"
#include "controller/srv/set_tcp_payload.hpp"

#include "controller/srv/enable_controller.hpp"
#include "controller/srv/get_chassis_max_vel.hpp"
#include "controller/srv/set_chassis_max_vel.hpp"
#include "controller/srv/get_control_policy.hpp"
#include "controller/srv/set_control_policy.hpp"
#include "controller/srv/get_safe_mode.hpp"
#include "controller/srv/set_safe_mode.hpp"
#include "controller/srv/is_enabled_controller.hpp"

#include "controller/msg/move_message.hpp"
#include "controller/msg/error_message.hpp"
#include "controller/msg/real_time_body_joints.hpp"
#include "controller/msg/real_time_body_tcp_cartesian.hpp"
#include "controller/msg/joint_params.hpp"
#include "controller/msg/cartesian_params.hpp"

// #include "controller/msg/joint_angle.hpp"
// #include "controller/msg/linear_move_params.hpp"
// #include "controller/msg/localization.hpp"
// #include "controller/msg/mobile_base_cmd.hpp"
// #include "controller/msg/mobile_base_state.hpp"
// #include "controller/msg/odometry_message.hpp"
#include "controller/msg/tcp_offset_params.hpp"
#include "controller/msg/tcp_payload_params.hpp"
#include "controller/msg/tcp_pose_params.hpp"

#include "DeviceControlServiceClient.h"
#include "H10wGrpcParam.h"


using namespace GlobalConstants::H10W;
using controller::msg::ErrorMessage;
using controller::msg::MoveMessage;
using controller::msg::RealTimeBodyJoints;
using controller::msg::RealTimeBodyTcpCartesian;

using controller::msg::CartesianParams;
using controller::msg::JointParams;
using controller::msg::TcpOffsetParams;
using controller::msg::TcpPayloadParams;

using controller::srv::SetJointMaxVel;
using controller::srv::SetJointSoftLimit;



class H10wRosClient : public rclcpp::Node
{
public:
    explicit H10wRosClient(const std::string &strIpPort = "localhost");
    ~H10wRosClient() = default;

    bool has_move_msg() const { return get_move_msg_ != nullptr; }

private:
    void move_callback(const MoveMessage::SharedPtr msg);

    void error_callback(const ErrorMessage::SharedPtr msg);

public:
    bool ros_singlemove(const MoveParams params,uint32_t &token);

    bool ros_multimove(const std::vector<MoveParams> params,uint32_t &token);

    bool ros_linearmove(const std::vector<LinearMoveParams> params,uint32_t &token);

    bool enable_realtime_cmd(bool m_enable);

    bool clear_error();

    bool servoj(RealTimeBodyJoints msg, double m_step);

    bool servol(RealTimeBodyTcpCartesian msg, double m_step);

    bool speedj(RealTimeBodyJoints msg, int32_t t);

    bool speedl(RealTimeBodyTcpCartesian msg, int32_t t);

    bool get_version(ControllerVersion &version);

    bool get_joint_soft_limit(std::vector<JointParams> &joint_params);

    bool get_joint_max_vel(std::vector<JointParams> &joint_params);

    bool get_joint_max_acc(std::vector<JointParams> &joint_params);

    bool get_cart_trans_max_vel(std::vector<CartesianParams> &cartesian_params);

    bool get_cart_trans_max_acc(std::vector<CartesianParams> &cartesian_params);

    bool get_cart_rota_max_vel(std::vector<CartesianParams> &cartesian_params);

    bool get_cart_rota_max_acc(std::vector<CartesianParams> &cartesian_params);

    bool get_joint_mech_limit(std::vector<JointParams> &joint_params);

    bool get_joint_mech_max_vel(std::vector<JointParams> &joint_params);

    bool get_joint_mech_max_acc(std::vector<JointParams> &joint_params);

    bool get_cart_mech_trans_max_vel(std::vector<CartesianParams> &cartesian_params);

    bool get_cart_mech_trans_max_acc(std::vector<CartesianParams> &cartesian_params);

    bool get_cart_mech_rota_max_vel(std::vector<CartesianParams> &cartesian_params);

    bool get_cart_mech_rota_max_acc(std::vector<CartesianParams> &cartesian_params);

    bool get_tcp_offset(std::vector<int32_t> type, std::vector<TcpOffsetParams> &tcp_offset_params);

    bool get_tcp_payload(std::vector<int32_t> type, std::vector<TcpPayloadParams> &tcp_payload_params);

    bool set_joint_soft_limit(const std::vector<JointSoftLimitParams> params);

    bool set_joint_max_vel(const std::vector<JointMaxParams> max_vel);

    bool set_joint_max_acc(const std::vector<JointMaxParams> max_acc);

    bool set_cart_trans_max_vel(const std::vector<CartMaxParams> max_vel);

    bool set_cart_trans_max_acc(const std::vector<CartMaxParams> max_acc);

    bool set_cart_rota_max_vel(const std::vector<CartMaxParams> max_vel);

    bool set_cart_rota_max_acc(const std::vector<CartMaxParams> max_acc);

    bool set_tcp_offset(const std::vector<TcpParam> tcp_offset_params);

    bool set_tcp_payload(const std::vector<TcpParam> tcp_payload_params);

    bool forward(const ForwardRequest joint, std::vector<TcpParam> &pose);

    bool inverse(const KinematicsParams params, std::vector<double> &joint);


    bool enable_controller(bool m_enable);

    bool get_chassis_max_vel(double &linear_vel, double &angular_vel);

    bool set_chassis_max_vel(double linear_vel, double angular_vel);

    bool get_control_policy(int32_t &policy);

    bool set_control_policy(int32_t policy);

    bool get_safe_mode(bool &safe_mode);

    bool set_safe_mode(bool safe_mode);

    bool is_enabled_controller(bool &is_enabled);



    // std::vector<std::pair<int32_t, std::vector<double>>> forward(const std::vector<int32_t> &type, std::vector<double> &joint_angles);

    // std::vector<double> inverse(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, bool if_use_whole_body);

public:
    std::shared_ptr<grpc::Channel> m_channel;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::unique_ptr<H10WGrpcParam> m_pControllerClient;
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
        SharedPtr m_get_tcp_payload_client;

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
    rclcpp::Client<controller::srv::EnableController>::SharedPtr
        m_enable_controller_client;
    rclcpp::Client<controller::srv::GetChassisMaxVel>::SharedPtr 
        m_get_chassis_max_vel_client;
    rclcpp::Client<controller::srv::GetControlPolicy>::SharedPtr
        m_get_control_policy_client;
    rclcpp::Client<controller::srv::GetSafeMode>::SharedPtr
        m_get_safe_mode_client;
    rclcpp::Client<controller::srv::IsEnabledController>::SharedPtr
        m_is_enable_controller_client;
    rclcpp::Client<controller::srv::SetChassisMaxVel>::SharedPtr
        m_set_chassis_max_vel_client;
    rclcpp::Client<controller::srv::SetControlPolicy>::SharedPtr
        m_set_control_policy_client;
    rclcpp::Client<controller::srv::SetSafeMode>::SharedPtr
        m_set_safe_mode_client;
};
