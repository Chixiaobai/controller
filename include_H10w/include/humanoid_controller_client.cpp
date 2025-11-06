#include "humanoid_controller_client.h"

HumanoidControllerClient::HumanoidControllerClient(std::shared_ptr<grpc::Channel> pChannel)
    : m_motion_stub_(MotionService::NewStub(pChannel)),
      m_system_stub_(SystemService::NewStub(pChannel)),
      m_params_stub_(ParamsService::NewStub(pChannel)) {}

bool HumanoidControllerClient::SingleJointMove(int32_t index, float &position, float &velocity_percent, uint32_t &token)
{

    JointMoveRequest request;
    request.set_joint_index(index);
    request.set_target_position(position);
    request.set_velocity(velocity_percent);
    JointMoveResponse response;
    ClientContext ctx;
    Status status = m_motion_stub_->RequestSingleJointMove(&ctx, request, &response);
    if (status.ok())
    {
        token = response.token();
        std::cout << "SingleJointMove succeeded: " << response.success() << ", Token: " << token << std::endl;
        return response.success();
    }
    else
    {
        std::cerr << "SingleJointMove RPC failed: " << status.error_message() << '\n';
        return false;
    }
}

bool HumanoidControllerClient::MultiJointsMove(const std::vector<uint32_t> &joint_index, const std::vector<float> &position, const std::vector<float> &velocity_percent, uint32_t &token)
{
    if (joint_index.size() <= 0)
    {
        std::cout << "Invalid joint count: " << joint_index.size() << std::endl;
        return false;
    }
    if (position.size() != joint_index.size() ||
        velocity_percent.size() != joint_index.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
        return false;
    }
    MultiJointMoveRequest request;
    for (int i = 0; i < joint_index.size(); i++)
    {
        JointAngle *joint = request.add_joint_angles();
        joint->set_joint_index(joint_index[i]);
        joint->set_target_position(position[i]);
        joint->set_velocity(velocity_percent[i]);
    }

    MultiJointMoveResponse response;
    ClientContext ctx;
    Status status = m_motion_stub_->RequestMultiJointsMove(&ctx, request, &response);
    if (status.ok())
    {
        token = response.token();
        std::cout << "MultiJointsMove succeeded: " << response.success() << ", Token: " << token << std::endl;
        return response.success();
    }
    else
    {
        std::cerr << "MultiJointsMove RPC failed: " << status.error_message() << '\n';
        return false;
    }
}

bool HumanoidControllerClient::LinearMovel(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, const std::vector<float> velocity_percent, std::vector<float> acceleration_percent, int32_t &task_id)
{
    LinearMoveRequest request;
    for (int i = 0; i < type.size(); i++)
    {
        LinearMove *Linear = request.add_linear_move();
        Linear->set_type(type[i]);
        for (int j = 0; j < 6; ++j)
        {
            Linear->add_pose(pose[i][j]);
        }
        Linear->set_velocity_percent(velocity_percent[i]);
        Linear->set_acceleration_percent(acceleration_percent[i]);
    }
    LinearMoveResponse response;
    ClientContext ctx;
    Status status = m_motion_stub_->RequestLinearMove(&ctx, request, &response);
    if (status.ok())
    {
        task_id = response.task_id();
        std::cout << "LinearMove succeeded: " << response.result() << ", task_id: " << task_id << std::endl;
        return response.result();
    }
    else
    {
        std::cerr << "LinearMove RPC failed: " << status.error_message() << '\n';
        return false;
    }
}

bool HumanoidControllerClient::stop()
{
    ClientContext ctx;
    google::protobuf::Empty empty;
    StopResponse response;
    Status status = m_motion_stub_->RequestStop(&ctx, empty, &response);
    if (!status.ok())
    {
        std::cerr << "Stop RPC failed: " << status.error_message() << '\n';
        return false;
    }
    return true;

    std::cout << "Stop success = " << std::boolalpha << response.success() << '\n';
}

bool HumanoidControllerClient::GetVersion(std::vector<std::string> &version)
{
    ClientContext ctx;
    google::protobuf::Empty empty;
    controller::VersionResponse response;
    Status status = m_system_stub_->GetVersion(&ctx, empty, &response);
    if (!status.ok())
    {
        std::cerr << "GetVersion RPC failed: " << status.error_message() << '\n';
        return false;
    }
    version.clear();
    version.push_back(response.main());
    for (const auto &[k, v] : response.plugins())
    {
        version.push_back(v);
    }
    return true;
}

bool HumanoidControllerClient::ClearError()
{
    ClientContext ctx;
    google::protobuf::Empty request;

    ErrorClearResponse response;
    Status status = m_system_stub_->ClearError(&ctx, request, &response);

    if (!status.ok())
    {
        std::cerr << "ClearError RPC failed: " << status.error_message() << '\n';
        return false;
    }
    std::cout << "ClearError success = " << std::boolalpha << response.success()
              << '\n';
    return true;
}

std::vector<std::tuple<uint32_t, double, double>> HumanoidControllerClient::getJointSoftLimit()
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    std::vector<std::tuple<uint32_t, double, double>> result;
    Status status = m_params_stub_->GetJointSoftLimit(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointSoftLimit failed: " << status.error_message()
                  << std::endl;
    }
    else
    {
        auto jointParams = response.joint_params();
        for (auto jointParam : jointParams)
        {
            result.emplace_back(jointParam.joint_index(), jointParam.max_pos(), jointParam.min_pos());
            std::cout << "joint index:" << jointParam.joint_index()
                      << " max:" << jointParam.max_pos()
                      << " min:" << jointParam.min_pos() << std::endl;
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getJointMaxVel()
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status = m_params_stub_->GetJointMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMaxVel failed: " << status.error_message()
                  << std::endl;
    }
    else
    {
        auto jointParams = response.joint_params();
        for (auto jointParam : jointParams)
        {
            result.emplace_back(jointParam.joint_index(),
                                jointParam.max_vel());
            std::cout << "joint index:" << jointParam.joint_index()
                      << " max vel:" << jointParam.max_vel() << std::endl;
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getJointMaxAcc()
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status = m_params_stub_->GetJointMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMaxAcc failed: " << status.error_message()
                  << std::endl;
    }
    else
    {
        auto jointParams = response.joint_params();
        for (auto jointParam : jointParams)
        {
            result.emplace_back(jointParam.joint_index(),
                                jointParam.max_acc());
            std::cout << "joint index:" << jointParam.joint_index()
                      << " max acc:" << jointParam.max_acc() << std::endl;
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianTranslationMaxVel()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianTranslationMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianTranslationMaxVel failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.trans_max_vel());
            std::cout << "cart index:" << cartParam.cartesian_index()
                      << " trans max vel:" << cartParam.trans_max_vel()
                      << std::endl;
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianTranslationMaxAcc()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianTranslationMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianTranslationMaxAcc failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.trans_max_acc());
            std::cout << "cart index:" << cartParam.cartesian_index()
                      << " trans max acc" << cartParam.trans_max_acc() << std::endl;
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianRotationMaxVel()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianRotationMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianRotationMaxVel failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.rota_max_vel());
            std::cout << "cart index:" << cartParam.cartesian_index()
                      << " rota max vel:" << cartParam.rota_max_vel() << std::endl;
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianRotationMaxAcc()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianRotationMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianRotationMaxAcc failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.rota_max_acc());
            std::cout << "cart Index:" << cartParam.cartesian_index()
                      << " rota max acc:" << cartParam.rota_max_acc() << std::endl;
        }
    }
    return result;
}

std::vector<std::tuple<uint32_t, double, double>> HumanoidControllerClient::getJointMechanicalLimit()
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    std::vector<std::tuple<uint32_t, double, double>> result;
    Status status =
        m_params_stub_->GetJointMechanicalLimit(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMechanicalLimit failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto jointParams = response.joint_params();
        for (auto jointParam : jointParams)
        {
            result.emplace_back(jointParam.joint_index(), jointParam.max_pos(),
                                jointParam.min_pos());
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getJointMechanicalMaxVel()
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetJointMechanicalMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMechanicalMaxVel failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto jointParams = response.joint_params();
        for (auto jointParam : jointParams)
        {
            result.emplace_back(jointParam.joint_index(), jointParam.max_vel());
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getJointMechanicalMaxAcc()
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetJointMechanicalMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMechanicalMaxAcc failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto jointParams = response.joint_params();
        for (auto jointParam : jointParams)
        {
            result.emplace_back(jointParam.joint_index(), jointParam.max_acc());
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianMechanicalTranslationMaxVel()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianMechanicalTranslationMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalTranslationMaxVel failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.trans_max_vel());
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianMechanicalTranslationMaxAcc()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianMechanicalTranslationMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalTranslationMaxAcc failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.trans_max_acc());
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianMechanicalRotationMaxVel()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianMechanicalRotationMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalRotationMaxVel failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.rota_max_vel());
        }
    }
    return result;
}

std::vector<std::pair<uint32_t, double>> HumanoidControllerClient::getCartesianMechanicalRotationMaxAcc()
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    std::vector<std::pair<uint32_t, double>> result;
    Status status =
        m_params_stub_->GetCartesianMechanicalRotationMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalRotationMaxAcc failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto cartParams = response.cartesian_params();
        for (auto cartParam : cartParams)
        {
            result.emplace_back(cartParam.cartesian_index(),
                                cartParam.rota_max_acc());
        }
    }
    return result;
}

std::vector<std::pair<int32_t, std::vector<double>>> HumanoidControllerClient::get_tcp_offset(std::vector<int32_t> &type)
{
    ClientContext clientText;
    GetTcpOffsetRequest request;
    GetTcpOffsetResponse response;
    std::vector<std::pair<int32_t, std::vector<double>>> result;

    for (int i = 0; i < type.size(); i++)
    {
        request.add_type(type[i]);
    }
    Status status =
        m_params_stub_->GetTcpOffset(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetTcpOffset failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto tcp_offset_params = response.tcp_offset_params();
        for (auto tcp_offset_param : tcp_offset_params)
        {
            // 修复：显式转换 RepeatedField 到 vector
            std::vector<double> offsetVec(
                tcp_offset_param.offset().begin(),
                tcp_offset_param.offset().end());
            result.emplace_back(tcp_offset_param.type(), offsetVec);
        }
    }
    return result;
}

std::vector<std::pair<int32_t, std::vector<double>>> HumanoidControllerClient::get_tcp_payload(std::vector<int32_t> &type)
{
    ClientContext clientText;
    GetTcpPayloadRequest request;
    GetTcpPayloadResponse response;
    std::vector<std::pair<int32_t, std::vector<double>>> result;

    for (int i = 0; i < type.size(); i++)
    {
        request.add_type(type[i]);
    }
    Status status =
        m_params_stub_->GetTcpPayload(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetTcpPayload failed: "
                  << status.error_message() << std::endl;
    }
    else
    {
        auto tcp_payload_params = response.tcp_payload_params();
        for (auto tcp_payload_param : tcp_payload_params)
        {
            std::vector<double> payloadVec(
                tcp_payload_param.parameters().begin(),
                tcp_payload_param.parameters().end());
            result.emplace_back(tcp_payload_param.type(), payloadVec);
        }
    }
    return result;
}

bool HumanoidControllerClient::setJointSoftLimit(const std::vector<uint32_t> &joint_index, const std::vector<float> &max_pos, const std::vector<float> &min_pos)
{
    ClientContext context;
    SetJointParamsRequest request;
    SetParamResponse response;

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
        auto jointParam = request.add_joint_params();
        jointParam->set_joint_index(joint_index[i]);
        jointParam->set_max_pos(max_pos[i]);
        jointParam->set_min_pos(min_pos[i]);
    }

    Status status = m_params_stub_->SetJointSoftLimit(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetJointSoftLimit call succeeded!" << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetJointSoftLimit call failed: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setJointMaxVel(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_vel)
{
    ClientContext context;
    SetJointParamsRequest request;
    SetParamResponse response;

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
        auto jointParam = request.add_joint_params();
        jointParam->set_joint_index(joint_index[i]);
        jointParam->set_max_vel(max_vel[i]);
    }

    Status status = m_params_stub_->SetJointMaxVel(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetJointMaxVel call succeeded!" << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetJointMaxVel call failed: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setJointMaxAcc(const std::vector<uint32_t> &joint_index, const std::vector<double> &max_acc)
{
    ClientContext context;
    SetJointParamsRequest request;
    SetParamResponse response;

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
        auto jointParam = request.add_joint_params();
        jointParam->set_joint_index(joint_index[i]);
        jointParam->set_max_acc(max_acc[i]);
    }

    Status status = m_params_stub_->SetJointMaxAcc(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetJointMaxAcc call succeeded!" << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetJointMaxAcc call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setCartesianTranslationMaxVel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_vel)
{
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;

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
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(cartesian_index[i]);
        cartesianParam->set_trans_max_vel(trans_max_vel[i]);
    }

    Status status = m_params_stub_->SetCartesianTranslationMaxVel(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetCartesianTranslationMaxVel call succeeded!"
                  << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetCartesianTranslationMaxVel call failed!"
                  << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setCartesianTranslationMaxAcc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &trans_max_acc)
{
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;

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
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(cartesian_index[i]);
        cartesianParam->set_trans_max_acc(trans_max_acc[i]);
    }

    Status status = m_params_stub_->SetCartesianTranslationMaxAcc(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetCartesianTranslationMaxAcc call succeeded!"
                  << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetCartesianTranslationMaxAcc call failed!"
                  << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setCartesianRotationMaxVel(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_vel)
{
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;

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
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(cartesian_index[i]);
        cartesianParam->set_rota_max_vel(rota_max_vel[i]);
    }

    Status status = m_params_stub_->SetCartesianRotationMaxVel(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetCartesianRotationMaxVel call succeeded!"
                  << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetCartesianRotationMaxVel call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setCartesianRotationMaxAcc(const std::vector<uint32_t> &cartesian_index, const std::vector<double> &rota_max_acc)
{
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;

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
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(cartesian_index[i]);
        cartesianParam->set_rota_max_acc(rota_max_acc[i]);
    }

    Status status =
        m_params_stub_->SetCartesianRotationMaxAcc(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetCartesianRotationMaxAcc call succeeded!"
                  << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetCartesianRotationMaxAcc call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setTcpOffset(const std::vector<int32_t> &type, std::vector<std::vector<double>> &offset)
{
    ClientContext context;
    SetTcpOffsetParamsRequest request;
    SetParamResponse response;

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
        auto offsetParam = request.add_tcp_offset_params();
        offsetParam->set_type(type[i]);
        for (int j = 0; j < offset[0].size(); j++)
        {
            offsetParam->add_offset(offset[i][j]);
        }
    }

    Status status =
        m_params_stub_->SetTcpOffset(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetTcpOffset call succeeded!"
                  << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetTcpOffset call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool HumanoidControllerClient::setTcpPayload(const std::vector<int32_t> &type, std::vector<std::vector<double>> &payload)
{
    ClientContext context;
    SetTcpPayloadParamsRequest request;
    SetParamResponse response;

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
        auto payloadParam = request.add_tcp_payload_params();
        payloadParam->set_type(type[i]);
        for (int j = 0; j < payload[0].size(); j++)
        {
            payloadParam->add_parameters(payload[i][j]);
        }
    }

    Status status =
        m_params_stub_->SetTcpPayload(&context, request, &response);
    if (status.ok())
    {
        std::cout << "RPC: SetTcpPayload call succeeded!"
                  << std::endl;
        std::cout << "Response: " << response.success() << std::endl;
        return response.success();
    }
    else
    {
        std::cout << "RPC: SetTcpPayload call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
        return false;
    }
}

std::vector<std::pair<int32_t, std::vector<double>>> HumanoidControllerClient::forward(const std::vector<int32_t> &type, std::vector<double> &joint_angles)
{
    ClientContext context;
    ForwardKinematicsRequest request;
    ForwardKinematicsResponse response;
    std::vector<std::pair<int32_t, std::vector<double>>> result;

    if (type.size() <= 0)
    {
        std::cout << "Invalid joint count: " << type.size() << std::endl;
    }
    if (joint_angles.size() != type.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
    }
    for (int i = 0; i < type.size(); i++)
    {
        request.add_type(type[i]);
        request.add_joint_angles(joint_angles[i]);
    }

    Status status =
        m_params_stub_->ForwardKinematics(&context, request, &response);
    if (status.ok())
    {
        auto tcpPoseParams = response.tcp_pose_params();
        for (auto tcpPoseParam : tcpPoseParams)
        {
            std::vector<double> poseVec(
                tcpPoseParam.pose().begin(),
                tcpPoseParam.pose().end());
            result.emplace_back(tcpPoseParam.type(), poseVec);
        }
    }
    else
    {
        std::cout << "RPC: ForwardKinematics call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
    }
    return result;
}

std::vector<double> HumanoidControllerClient::inverse(const std::vector<int32_t> &type, std::vector<std::vector<double>> &pose, bool if_use_whole_body)
{
    ClientContext context;
    InverseKinematicsRequest request;
    InverseKinematicsResponse response;
    std::vector<double> result;

    if (type.size() <= 0)
    {
        std::cout << "Invalid joint count: " << type.size() << std::endl;
    }
    if (pose[0].size() != type.size())
    {
        std::cout << "Parameter array size mismatch with joint count" << std::endl;
    }
    for (int i = 0; i < type.size(); i++)
    {
        auto tcpPoseParams = request.add_tcp_pose_params();
        tcpPoseParams->set_type(type[i]);
        for (int j = 0; j < pose[0].size(); j++)
        {
            tcpPoseParams->add_pose(pose[i][j]);
        }
    }

    request.set_if_use_whole_body(if_use_whole_body);

    Status status =
        m_params_stub_->InverseKinematics(&context, request, &response);
    if (status.ok())
    {
        auto jointAngles = response.joint_angles();
        for (int i = 0; i < jointAngles.size(); i++)
        {
            result.emplace_back(jointAngles[i]);
        }
    }
    else
    {
        std::cout << "RPC: InverseKinematics call failed!" << std::endl;
        std::cout << "Error: " << status.error_message() << std::endl;
    }
    return result;
}