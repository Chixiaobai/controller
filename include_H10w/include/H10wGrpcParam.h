#pragma once
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/config_protobuf.h>
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/system_service.grpc.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/params_service.grpc.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/chassis_service.grpc.pb.h"
// #include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/system_service.pb.h"
// #include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/params_service.pb.h"
// #include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/chassis_service.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/motion_service.grpc.pb.h"

#include "controller/msg/tcp_offset_params.hpp"
#include "controller/msg/tcp_payload_params.hpp"
#include "controller/msg/tcp_pose_params.hpp"

#include "H10Wglobalconstants.h"
using namespace GlobalConstants::H10W;

using controller::ClassisService;
using controller::EnableControllerRequest ;
using controller::EnableControllerResponse;
using controller::IsEnabledControllerResponse;
using controller::SetMaxVelocityRequest;
using controller::SetMaxVelocityResponse;
using controller::GetMaxVelocityResponse;

using controller::CartesianParams;
using controller::ErrorClearResponse;
using controller::ForwardKinematicsRequest;
using controller::ForwardKinematicsResponse;
using controller::GetCartesianParamsResponse;
using controller::GetJointParamsResponse;
using controller::GetTcpOffsetRequest;
using controller::GetTcpOffsetResponse;
using controller::GetTcpPayloadRequest;
using controller::GetTcpPayloadResponse;
using controller::InverseKinematicsRequest;
using controller::InverseKinematicsResponse;
using controller::JointParams;
using controller::TcpOffsetParams;
using controller::TcpPayloadParams;
using controller::TcpPoseParams;
using controller::ParamsService;

using controller::SetCartesianParamsRequest;
using controller::SetJointParamsRequest;
using controller::SetParamResponse;
using controller::SetTcpOffsetParamsRequest;
using controller::SetTcpPayloadParamsRequest;
using controller::SystemService;
using google::protobuf::Empty;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using controller::JointAngle;
using controller::JointMoveRequest;
using controller::JointMoveResponse;

using controller::LinearMove;
using controller::LinearMoveRequest;
using controller::LinearMoveResponse;

using controller::MotionService;
using controller::MultiJointMoveRequest;
using controller::MultiJointMoveResponse;

using controller::StopResponse;

using controller::EnableRealtimeRequest;
using controller::EnableRealtimeResponse;

using controller::ControlPolicyRequest;
using controller::ControlPolicyResponse;
using controller::GetControlPolicyResponse;

using controller::SetSafeModeRequest;
using controller::SetSafeModeResponse;
using controller::GetSafeModeResponse;



class H10WGrpcParam final
{
public:
    H10WGrpcParam(std::shared_ptr<Channel> pChannel);
    ~H10WGrpcParam() = default;

    bool MultiJointMove(std::vector<MoveParams> params,uint32_t &token);

    bool SingleJointMove(MoveParams params,uint32_t &token);

    bool LinearMovel(std::vector<LinearMoveParams> params,uint32_t &token);
    
    bool GetVersion(std::vector<std::string> &version);
    bool ClearError();
    bool EnableController(bool m_enable);
    bool IsEnabledController(bool &m_enable);

    //底盘速度
    bool setBaseMaxVel(double linear_vel, double angular_vel);
    bool getBaseMaxVel(double &linear_vel, double &angular_vel);

    // 获取关节参数
    bool getJointSoftLimit(std::vector<JointSoftLimitParams> &limits);
    bool getJointMaxVel(std::vector<JointMaxParams> &max_vel);
    bool getJointMaxAcc(std::vector<JointMaxParams> &max_acc);
    bool getJointMechanicalLimit(std::vector<JointSoftLimitParams> &mechanical_limits);
    bool getJointMechanicalMaxVel(std::vector<JointMaxParams> &mechanical_max_vel);
    bool getJointMechanicalMaxAcc(std::vector<JointMaxParams> &mechanical_max_acc);

    // 获取笛卡尔坐标参数
    bool getCartesianTranslationMaxVel(std::vector<CartMaxParams> &cartrans_max_vel);
    bool getCartesianTranslationMaxAcc(std::vector<CartMaxParams> &cartrans_max_acc);
    bool getCartesianRotationMaxVel(std::vector<CartMaxParams> &cartrot_max_vel);
    bool getCartesianRotationMaxAcc(std::vector<CartMaxParams> &cartrot_max_acc);
    bool getCartesianMechanicalTranslationMaxVel(std::vector<CartMaxParams> &cartransmec_max_vel);
    bool getCartesianMechanicalTranslationMaxAcc(std::vector<CartMaxParams> &cartransmec_max_acc);
    bool getCartesianMechanicalRotationMaxVel(std::vector<CartMaxParams> &cartrotmec_max_vel);
    bool getCartesianMechanicalRotationMaxAcc(std::vector<CartMaxParams> &cartrotmec_max_acc);

    // 获取TCP参数
    bool get_tcp_offset(const std::vector<int32_t> type, std::vector<TcpParam> &tcp_offset);
    bool get_tcp_payload(const std::vector<int32_t> type, std::vector<TcpParam> &tcp_payload);

    // 设置参数
    bool setJointSoftLimit(const std::vector<JointSoftLimitParams> limits);
    bool setJointMaxVel(const std::vector<JointMaxParams> max_vel);
    bool setJointMaxAcc(const std::vector<JointMaxParams> max_acc);
    bool setCartesianTranslationMaxVel(const std::vector<CartMaxParams> cartrans_max_vel);
    bool setCartesianTranslationMaxAcc(const std::vector<CartMaxParams> cartrans_max_acc);
    bool setCartesianRotationMaxVel(const std::vector<CartMaxParams> cartrot_max_vel);
    bool setCartesianRotationMaxAcc(const std::vector<CartMaxParams> cartrot_max_acc);
    bool setTcpOffset(const std::vector<TcpParam> tcp_offset);
    bool setTcpPayload(const std::vector<TcpParam> tcp_payload);


    bool forward(const ForwardRequest joint, std::vector<TcpParam> &pose);
    bool inverse(const KinematicsParams pose, std::vector<double> &joint);


    bool grcp_realtime_cmd(bool m_enable);
    bool grpc_set_controlpolicy(int32_t m_policy);
    bool grpc_get_controlpolicy(int32_t &m_policy);
    bool grpc_set_safemode(bool m_mode);
    bool grpc_get_safemode(bool &m_mode);
    bool stop();

private:
    std::unique_ptr<SystemService::Stub> m_system_stub_;
    std::unique_ptr<ParamsService::Stub> m_params_stub_;
    std::unique_ptr<ClassisService::Stub> m_classis_stub_;
    std::unique_ptr<MotionService::Stub> m_motion_stub_;
};
