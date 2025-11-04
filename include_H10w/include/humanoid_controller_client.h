#pragma once
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/config_protobuf.h>
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/motion_service.grpc.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/motion_service.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/system_service.grpc.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/system_service.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/params_service.grpc.pb.h"
#include "../grpc_msg/grpc_ws/common/grpc_pb_cpp/params_service.pb.h"

using controller::CartesianParams;
using controller::GetCartesianParamsResponse;
using controller::GetJointParamsResponse;
using controller::GetTcpOffsetRequest;
using controller::GetTcpPayloadRequest;
using controller::GetTcpOffsetResponse;
using controller::GetTcpPayloadResponse;
using controller::JointParams;
using controller::ParamsService;
using controller::SetCartesianParamsRequest;
using controller::SetJointParamsRequest;
using controller::SetParamResponse;
using controller::SetTcpOffsetParamsRequest;
using controller::SetTcpPayloadParamsRequest;
using controller::ForwardKinematicsRequest;
using controller::ForwardKinematicsResponse;
using controller::InverseKinematicsRequest;
using controller::InverseKinematicsResponse;

using google::protobuf::Empty;
using controller::ErrorClearResponse;
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
using controller::SystemService;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

class HumanoidControllerClient final
{
public:
    HumanoidControllerClient(std::shared_ptr<Channel> pChannel);
    ~HumanoidControllerClient() = default;

    bool MultiJointsMove(const std::vector<int32_t> &joint_index, const std::vector<float> &position, const std::vector<float> &velocity_percent, uint32_t &token);

    bool SingleJointMove(int32_t index, float &position, float &velocity_percent, uint32_t &token);

    bool LinearMovel(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, const std::vector<float> velocity_percent, std::vector<float> acceleration_percent, int32_t &task_id);

    bool stop();

    bool GetVersion(std::vector<std::string> &version);

    bool ClearError();

    std::vector<std::tuple<uint32_t, double, double>> getJointSoftLimit();

    std::vector<std::pair<uint32_t, double>> getJointMaxVel();

    std::vector<std::pair<uint32_t, double>> getJointMaxAcc();

    std::vector<std::pair<uint32_t, double>> getCartesianTranslationMaxVel();

    std::vector<std::pair<uint32_t, double>> getCartesianTranslationMaxAcc();

    std::vector<std::pair<uint32_t, double>> getCartesianRotationMaxVel();

    std::vector<std::pair<uint32_t, double>> getCartesianRotationMaxAcc();

    std::vector<std::tuple<uint32_t, double, double>> getJointMechanicalLimit();

    std::vector<std::pair<uint32_t, double>> getJointMechanicalMaxVel();

    std::vector<std::pair<uint32_t, double>> getJointMechanicalMaxAcc();

    std::vector<std::pair<uint32_t, double>> getCartesianMechanicalTranslationMaxVel();

    std::vector<std::pair<uint32_t, double>> getCartesianMechanicalTranslationMaxAcc();

    std::vector<std::pair<uint32_t, double>> getCartesianMechanicalRotationMaxVel();

    std::vector<std::pair<uint32_t, double>> getCartesianMechanicalRotationMaxAcc();

    std::vector<std::pair<int32_t, std::vector<double>>> get_tcp_offset(std::vector<int32_t> &type);
    
    std::vector<std::pair<int32_t, std::vector<double>>> get_tcp_payload(std::vector<int32_t> &type);

    bool setJointSoftLimit(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_pos, const std::vector<double> &min_pos);

    bool setJointMaxVel(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_vel);

    bool setJointMaxAcc(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_acc);

    bool setCartesianTranslationMaxVel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_vel);

    bool setCartesianTranslationMaxAcc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_acc);

    bool setCartesianRotationMaxVel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_vel);

    bool setCartesianRotationMaxAcc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_acc);

    bool setTcpOffset(const std::vector<int32_t> &type, std::vector<std::vector<double>> &offset);
    
    bool setTcpPayload(const std::vector<int32_t> &type, std::vector<std::vector<double>> &payload);
    
    std::vector<std::pair<int32_t, std::vector<double>>> forward(const std::vector<int32_t> &type, std::vector<double> &joint_angles);
    
    std::vector<double> inverse(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, bool if_use_whole_body);

private:
    std::unique_ptr<MotionService::Stub> m_motion_stub_;
    std::unique_ptr<SystemService::Stub> m_system_stub_;
    std::unique_ptr<ParamsService::Stub> m_params_stub_;
};