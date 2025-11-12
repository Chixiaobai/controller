#include "H10wGrpcParam.h"

H10WGrpcParam::H10WGrpcParam(std::shared_ptr<grpc::Channel> pChannel)
    : m_system_stub_(SystemService::NewStub(pChannel)),
      m_params_stub_(ParamsService::NewStub(pChannel)),
      m_classis_stub_(ClassisService::NewStub(pChannel)),
      m_motion_stub_(MotionService::NewStub(pChannel)) {}

bool H10WGrpcParam::GetVersion(std::vector<std::string> &version)
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

bool H10WGrpcParam::ClearError()
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

bool H10WGrpcParam::EnableController(bool m_enable)
{
    ClientContext ctx;
    EnableControllerRequest request;
    EnableControllerResponse response;
    request.set_enable(m_enable);
    Status status = m_system_stub_->EnableController(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "EnableController RPC failed: " << status.error_message() << '\n';
    }
    else
    {
        std::cout << "EnableController success = " << std::boolalpha << response.success()
                  << '\n';
    }
    return response.success();
}

bool H10WGrpcParam::IsEnabledController(bool &m_enable)
{
    ClientContext ctx;
    google::protobuf::Empty request;
    IsEnabledControllerResponse response;
    Status status = m_system_stub_->IsEnabledController(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "IsEnabledController RPC failed: " << status.error_message() << '\n';
        return false;
    }
    else
    {
        m_enable = response.enable();
        std::cout << "IsEnabledController success = " << std::boolalpha << response.enable()
                  << '\n';
        return true;
    }
}

bool H10WGrpcParam::setBaseMaxVel(double linear_vel, double angular_vel)
{
    ClientContext ctx;
    SetMaxVelocityRequest request;
    SetMaxVelocityResponse response;
    request.set_linear_velocity(linear_vel);
    request.set_angular_velocity(angular_vel);
    Status status = m_classis_stub_->RequestSetMaxVelocity(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RequestSetMaxVelocity RPC failed: " << status.error_message() << '\n';
    }
    else
    {
        std::cout << "RequestSetMaxVelocity success = " << std::boolalpha << response.success()
                  << '\n';
    }
    return response.success();
}

bool H10WGrpcParam::getBaseMaxVel(double &linear_vel, double &angular_vel)
{
    ClientContext ctx;
    google::protobuf::Empty request;
    GetMaxVelocityResponse response;
    Status status = m_classis_stub_->RequestGetMaxVelocity(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RequestGetMaxVelocity RPC failed: " << status.error_message() << '\n';
        return false;
    }
    else
    {
        linear_vel = response.linear_velocity();
        angular_vel = response.angular_velocity();
        std::cout << "RequestGetMaxVelocity success " << '\n';
        return true;
    }
}

bool H10WGrpcParam::getJointSoftLimit(std::vector<JointSoftLimitParams> &limits)
{
    ClientContext clientText;
    Empty request;
    GetJointParamsResponse response;
    Status status = m_params_stub_->GetJointSoftLimit(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointSoftLimit failed: " << status.error_message()
                  << std::endl;
        return false;
    }
    else
    {
        limits.reserve(response.joint_params_size());
        for (const auto &param : response.joint_params())
        {
            limits.emplace_back(param.joint_index(), param.max_pos(), param.min_pos());
        }
        return true;
    }
}

bool H10WGrpcParam::getJointMaxVel(std::vector<JointMaxParams> &max_vel)
{
    ClientContext ctx;
    Empty request;
    GetJointParamsResponse response;
    Status status = m_params_stub_->GetJointMaxVel(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMaxVel failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        max_vel.reserve(response.joint_params_size());
        for (const auto &param : response.joint_params())
        {
            max_vel.emplace_back(param.joint_index(), param.max_vel());
        }
        return true;
    }
}

bool H10WGrpcParam::getJointMaxAcc(std::vector<JointMaxParams> &max_acc)
{
    ClientContext ctx;
    Empty request;
    GetJointParamsResponse response;
    Status status = m_params_stub_->GetJointMaxAcc(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMaxAcc failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        max_acc.reserve(response.joint_params_size());
        for (const auto &param : response.joint_params())
        {
            max_acc.emplace_back(param.joint_index(), param.max_acc());
        }
        return true;
    }
}

bool H10WGrpcParam::getJointMechanicalLimit(std::vector<JointSoftLimitParams> &mechanical_limits)
{
    ClientContext ctx;
    Empty request;
    GetJointParamsResponse response;
    Status status = m_params_stub_->GetJointMechanicalLimit(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMechanicalLimit failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        mechanical_limits.reserve(response.joint_params_size());
        for (const auto &param : response.joint_params())
        {
            mechanical_limits.emplace_back(param.joint_index(), param.max_pos(), param.min_pos());
        }
        return true;
    }
}

bool H10WGrpcParam::getJointMechanicalMaxVel(std::vector<JointMaxParams> &mechanical_max_vel)
{
    ClientContext ctx;
    Empty request;
    GetJointParamsResponse response;
    Status status = m_params_stub_->GetJointMechanicalMaxVel(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMechanicalMaxVel failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        mechanical_max_vel.reserve(response.joint_params_size());
        for (const auto &param : response.joint_params())
        {
            mechanical_max_vel.emplace_back(param.joint_index(), param.max_vel());
        }
        return true;
    }
}

bool H10WGrpcParam::getJointMechanicalMaxAcc(std::vector<JointMaxParams> &mechanical_max_acc)
{
    ClientContext ctx;
    Empty request;
    GetJointParamsResponse response;
    Status status = m_params_stub_->GetJointMechanicalMaxAcc(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetJointMechanicalMaxAcc failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        mechanical_max_acc.reserve(response.joint_params_size());
        for (const auto &param : response.joint_params())
        {
            mechanical_max_acc.emplace_back(param.joint_index(), param.max_acc());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianTranslationMaxVel(std::vector<CartMaxParams> &cartrans_max_vel)
{
    ClientContext ctx;
    Empty request;
    GetCartesianParamsResponse response;
    Status status = m_params_stub_->GetCartesianTranslationMaxVel(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianTranslationMaxVel failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrans_max_vel.reserve(response.cartesian_params_size());
        for (const auto &param : response.cartesian_params())
        {
            cartrans_max_vel.emplace_back(param.cartesian_index(), param.trans_max_vel());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianTranslationMaxAcc(std::vector<CartMaxParams> &cartrans_max_acc)
{
    ClientContext ctx;
    Empty request;
    GetCartesianParamsResponse response;
    Status status = m_params_stub_->GetCartesianTranslationMaxAcc(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianTranslationMaxAcc failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrans_max_acc.reserve(response.cartesian_params_size());
        for (const auto &param : response.cartesian_params())
        {
            cartrans_max_acc.emplace_back(param.cartesian_index(), param.trans_max_acc());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianRotationMaxVel(std::vector<CartMaxParams> &cartrot_max_vel)
{
    ClientContext ctx;
    Empty request;
    GetCartesianParamsResponse response;
    Status status = m_params_stub_->GetCartesianRotationMaxVel(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianRotationMaxVel failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrot_max_vel.reserve(response.cartesian_params_size());
        for (const auto &param : response.cartesian_params())
        {
            cartrot_max_vel.emplace_back(param.cartesian_index(), param.rota_max_vel());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianRotationMaxAcc(std::vector<CartMaxParams> &cartrot_max_acc)
{
    ClientContext ctx;
    Empty request;
    GetCartesianParamsResponse response;
    Status status = m_params_stub_->GetCartesianRotationMaxAcc(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianRotationMaxAcc failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrot_max_acc.reserve(response.cartesian_params_size());
        for (const auto &param : response.cartesian_params())
        {
            cartrot_max_acc.emplace_back(param.cartesian_index(), param.rota_max_acc());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianMechanicalTranslationMaxVel(std::vector<CartMaxParams> &cartrans_mechanical_max_vel)
{
    ClientContext ctx;
    Empty request;
    GetCartesianParamsResponse response;
    Status status = m_params_stub_->GetCartesianMechanicalTranslationMaxVel(&ctx, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalTranslationMaxVel failed: " << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrans_mechanical_max_vel.reserve(response.cartesian_params_size());
        for (const auto &param : response.cartesian_params())
        {
            cartrans_mechanical_max_vel.emplace_back(param.cartesian_index(), param.trans_max_vel());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianMechanicalTranslationMaxAcc(std::vector<CartMaxParams> &cartrans_mechanical_max_acc)
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    Status status =
        m_params_stub_->GetCartesianMechanicalTranslationMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalTranslationMaxAcc failed: "
                  << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrans_mechanical_max_acc.reserve(response.cartesian_params_size());
        for (const auto &cartParam : response.cartesian_params())
        {
            cartrans_mechanical_max_acc.emplace_back(cartParam.cartesian_index(), cartParam.trans_max_acc());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianMechanicalRotationMaxVel(std::vector<CartMaxParams> &cartrotmec_max_vel)
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    Status status =
        m_params_stub_->GetCartesianMechanicalRotationMaxVel(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalRotationMaxVel failed: "
                  << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrotmec_max_vel.reserve(response.cartesian_params_size());
        for (const auto &cartParam : response.cartesian_params())
        {
            cartrotmec_max_vel.emplace_back(cartParam.cartesian_index(), cartParam.rota_max_vel());
        }
        return true;
    }
}

bool H10WGrpcParam::getCartesianMechanicalRotationMaxAcc(std::vector<CartMaxParams> &cartrotmec_max_acc)
{
    ClientContext clientText;
    Empty request;
    GetCartesianParamsResponse response;
    Status status =
        m_params_stub_->GetCartesianMechanicalRotationMaxAcc(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetCartesianMechanicalRotationMaxAcc failed: "
                  << status.error_message() << std::endl;
        return false;
    }
    else
    {
        cartrotmec_max_acc.reserve(response.cartesian_params_size());
        for (const auto &cartParam : response.cartesian_params())
        {
            cartrotmec_max_acc.emplace_back(cartParam.cartesian_index(), cartParam.rota_max_acc());
        }
        return true;
    }
}

bool H10WGrpcParam::get_tcp_offset(const std::vector<int32_t> type, std::vector<TcpParam> &tcp_offset)
{
    ClientContext clientText;
    GetTcpOffsetRequest request;
    GetTcpOffsetResponse response;
    for (int t : type)
    {
        request.add_type(t);
    }
    Status status =
        m_params_stub_->GetTcpOffset(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetTcpOffset failed: "
                  << status.error_message() << std::endl;
        return false;
    }
    else
    {
        tcp_offset.reserve(response.tcp_offset_params_size());
        for (const auto &tcp_offset_param : response.tcp_offset_params())
        {
            std::vector<double> offset_data(tcp_offset_param.offset().begin(), tcp_offset_param.offset().end());
            tcp_offset.emplace_back(tcp_offset_param.type(), std::move(offset_data));
        }
        return true;
    }
}

bool H10WGrpcParam::get_tcp_payload(const std::vector<int32_t> type, std::vector<TcpParam> &tcp_payload)
{
    ClientContext clientText;
    GetTcpPayloadRequest request;
    for (int t : type)
    {
        request.add_type(t);
    }
    GetTcpPayloadResponse response;
    Status status =
        m_params_stub_->GetTcpPayload(&clientText, request, &response);
    if (!status.ok())
    {
        std::cerr << "RPC: GetTcpPayload failed: "
                  << status.error_message() << std::endl;
        return false;
    }
    else
    {
        tcp_payload.reserve(response.tcp_payload_params_size());
        for (const auto &tcp_payload_param : response.tcp_payload_params())
        {
            std::vector<double> payload_data(tcp_payload_param.parameters().begin(), tcp_payload_param.parameters().end());
            tcp_payload.emplace_back(tcp_payload_param.type(), std::move(payload_data));
        }
        return true;
    }
}

bool H10WGrpcParam::setJointSoftLimit(const std::vector<JointSoftLimitParams> limits)
{
    if (limits.empty())
    {
        std::cerr << "Empty limits input" << std::endl;
        return false;
    }
    ClientContext context;
    SetJointParamsRequest request;
    SetParamResponse response;
    for (const auto &limit : limits)
    {
        auto param = request.add_joint_params();
        param->set_joint_index(limit.joint_index);
        param->set_max_pos(limit.max_pos);
        param->set_min_pos(limit.min_pos);
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

bool H10WGrpcParam::setJointMaxVel(const std::vector<JointMaxParams> max_vel)
{
    if (max_vel.empty())
    {
        std::cerr << "Empty max_vel input" << std::endl;
        return false;
    }
    ClientContext context;
    SetJointParamsRequest request;
    SetParamResponse response;
    for (const auto &vel : max_vel)
    {
        auto jointParam = request.add_joint_params();
        jointParam->set_joint_index(vel.joint_index);
        jointParam->set_max_vel(vel.value);
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

bool H10WGrpcParam::setJointMaxAcc(const std::vector<JointMaxParams> max_acc)
{
    if (max_acc.empty())
    {
        std::cerr << "Empty max_acc input" << std::endl;
        return false;
    }
    ClientContext context;
    SetJointParamsRequest request;
    SetParamResponse response;
    for (const auto &acc : max_acc)
    {
        auto jointParam = request.add_joint_params();
        jointParam->set_joint_index(acc.joint_index);
        jointParam->set_max_acc(acc.value);
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

bool H10WGrpcParam::setCartesianTranslationMaxVel(const std::vector<CartMaxParams> cartrans_max_vel)
{
    if (cartrans_max_vel.empty())
    {
        std::cerr << "Empty cartrans_max_vel input" << std::endl;
        return false;
    }
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;
    for (const auto &vel : cartrans_max_vel)
    {
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(vel.cartesian_index);
        cartesianParam->set_trans_max_vel(vel.value);
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

bool H10WGrpcParam::setCartesianTranslationMaxAcc(const std::vector<CartMaxParams> cartrans_max_acc)
{
    if (cartrans_max_acc.empty())
    {
        std::cerr << "Empty cartrans_max_acc input" << std::endl;
        return false;
    }
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;
    for (const auto &acc : cartrans_max_acc)
    {
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(acc.cartesian_index);
        cartesianParam->set_trans_max_acc(acc.value);
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

bool H10WGrpcParam::setCartesianRotationMaxVel(const std::vector<CartMaxParams> cartrot_max_vel)
{
    if (cartrot_max_vel.empty())
    {
        std::cerr << "Empty cartrans_max_vel input" << std::endl;
        return false;
    }
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;
    for (const auto &vel : cartrot_max_vel)
    {
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(vel.cartesian_index);
        cartesianParam->set_rota_max_vel(vel.value);
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

bool H10WGrpcParam::setCartesianRotationMaxAcc(const std::vector<CartMaxParams> cartrot_max_acc)
{
    if (cartrot_max_acc.empty())
    {
        std::cerr << "Empty cartrot_max_acc input" << std::endl;
        return false;
    }
    ClientContext context;
    SetCartesianParamsRequest request;
    SetParamResponse response;
    for (const auto &acc : cartrot_max_acc)
    {
        auto cartesianParam = request.add_cartesian_params();
        cartesianParam->set_cartesian_index(acc.cartesian_index);
        cartesianParam->set_rota_max_acc(acc.value);
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

bool H10WGrpcParam::setTcpOffset(const std::vector<TcpParam> tcp_offset)
{
    if (tcp_offset.empty())
    {
        std::cerr << "Empty tcp_offset input" << std::endl;
        return false;
    }
    ClientContext context;
    SetTcpOffsetParamsRequest request;
    SetParamResponse response;
    for (const auto &offset : tcp_offset)
    {
        auto offsetParam = request.add_tcp_offset_params();
        offsetParam->set_type(offset.type);
        for (double val : offset.data)
        {
            offsetParam->add_offset(val);
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

bool H10WGrpcParam::setTcpPayload(const std::vector<TcpParam> tcp_payload)
{
    if (tcp_payload.empty())
    {
        std::cerr << "Empty tcp_payload input" << std::endl;
        return false;
    }
    ClientContext context;
    SetTcpPayloadParamsRequest request;
    SetParamResponse response;
    for (const auto &payload : tcp_payload)
    {
        auto payloadParam = request.add_tcp_payload_params();
        payloadParam->set_type(payload.type);
        for (double val : payload.data)
        {
            payloadParam->add_parameters(val);
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

bool H10WGrpcParam::forward(const ForwardRequest joint, std::vector<TcpParam> &pose)
{
    if (joint.joint_angles.empty() || joint.type.empty())
    {
        std::cerr << "Empty joint input or type/angle size mismatch" << std::endl;
        return false;
    }

    ClientContext context;
    ForwardKinematicsRequest request;
    ForwardKinematicsResponse response;

    for (int32_t t : joint.type)
        request.add_type(t);
    for (double angle : joint.joint_angles)
        request.add_joint_angles(angle);

    Status status = m_params_stub_->ForwardKinematics(&context, request, &response);
    if (status.ok())
    {
        pose.reserve(response.tcp_pose_params_size());
        for (const auto &tcpPoseParam : response.tcp_pose_params())
        {
            pose.emplace_back(
                tcpPoseParam.type(),
                std::vector<double>(tcpPoseParam.pose().begin(), tcpPoseParam.pose().end()));
        }
        return true;
    }
    else
    {
        std::cerr << "RPC: ForwardKinematics call failed! Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool H10WGrpcParam::inverse(const KinematicsParams pose, std::vector<double> &joint)
{
    if (pose.pose.empty() || pose.type.empty())
    {
        std::cerr << "Empty pose input" << std::endl;
        return false;
    }

    ClientContext context;
    InverseKinematicsRequest request;
    InverseKinematicsResponse response;

    for (size_t i = 0; i < pose.type.size(); ++i)
    {
        auto *tcpPoseParams = request.add_tcp_pose_params();
        tcpPoseParams->set_type(pose.type[i]);
        for (double val : pose.pose[i])
        {
            tcpPoseParams->add_pose(val);
        }
    }
    for (double angle : pose.joint_angle)
        request.add_reference_joint_angles(angle);

    request.set_if_use_whole_body(pose.if_use_whole_body);

    Status status = m_params_stub_->InverseKinematics(&context, request, &response);
    if (status.ok())
    {
        auto jointAngles = response.joint_angles();
        for (int i = 0; i < jointAngles.size(); i++)
        {
            joint.emplace_back(jointAngles[i]);
        }
        return true;
    }
    else
    {
        std::cerr << "RPC: InverseKinematics call failed! Error: " << status.error_message() << std::endl;
        return false;
    }
}

bool H10WGrpcParam::grcp_realtime_cmd(bool m_enable)
{
    ClientContext ctx;
    EnableRealtimeRequest request;
    request.set_enable(m_enable);
    EnableRealtimeResponse response;
    Status status = m_motion_stub_->RequestEnableRealtimeCmd(&ctx, request, &response);
    if (status.ok())
    {
        std::cout << "enable_realtime_cmd succeeded, " << ", m_enable: " << m_enable << std::endl;
        return response.success();
    }
    else
    {
        std::cerr << "enable_realtime_cmd RPC failed: " << status.error_message() << '\n';
        return response.success();
    }
}

bool H10WGrpcParam::grpc_set_controlpolicy(int32_t m_policy)
{
    ClientContext ctx;
    ControlPolicyRequest request;
    request.set_policy(m_policy);
    ControlPolicyResponse response;
    Status status = m_motion_stub_->RequestSetControlPolicy(&ctx, request, &response);
    if (status.ok())
    {
        std::cout << "set_controller_policy succeeded" << std::endl;
    }
    else
    {
        std::cerr << "set_controller_policy RPC failed: " << status.error_message() << '\n';
    }
    return response.success();
}

bool H10WGrpcParam::grpc_get_controlpolicy(int32_t &m_policy)
{
    ClientContext ctx;
    Empty request;
    GetControlPolicyResponse response;
    Status status = m_motion_stub_->RequestGetControlPolicy(&ctx, request, &response);
    if (status.ok())
    {
        m_policy = response.policy();
        std::cout << "get_controller_policy succeeded" << std::endl;
        return true;
    }
    else
    {
        std::cerr << "get_controller_policy RPC failed: " << status.error_message() << '\n';
        return false;
    }
}

bool H10WGrpcParam::grpc_set_safemode(bool m_mode)
{
    ClientContext ctx;
    SetSafeModeRequest request;
    request.set_mode(m_mode);
    SetSafeModeResponse response;
    Status status = m_motion_stub_->RequestSetSafeMode(&ctx, request, &response);
    if (status.ok())
    {
        std::cout << "grpc_set_safemode succeeded" << std::endl;
    }
    else
    {
        std::cerr << "grpc_set_safemode RPC failed: " << status.error_message() << '\n';
    }
    return response.success();
}

bool H10WGrpcParam::grpc_get_safemode(bool &m_mode)
{
    ClientContext ctx;
    Empty request;
    GetSafeModeResponse response;
    Status status = m_motion_stub_->RequestGetSafeMode(&ctx, request, &response);
    if (status.ok())
    {
        m_mode = response.mode();
        std::cout << "grpc_get_safemode succeeded" << std::endl;
        return true;
    }
    else
    {
        std::cerr << "grpc_get_safemode RPC failed: " << status.error_message() << '\n';
        return false;
    }
}

bool H10WGrpcParam::stop()
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
    std::cout << "Stop success = " << std::boolalpha << response.success() << '\n';
    return true;
}

bool H10WGrpcParam::SingleJointMove(MoveParams params, uint32_t &token)
{
    JointMoveRequest request;
    request.set_joint_index(params.joint_index);
    request.set_target_position(params.target_position);
    request.set_velocity(params.velocity);
    JointMoveResponse response;
    ClientContext ctx;
    Status status = m_motion_stub_->RequestSingleJointMove(&ctx, request, &response);
    if (status.ok())
    {
        token = response.token();
        std::cout << "SingleJointMove succeeded: " << response.success() << ", Token: " << token << std::endl;
    }
    else
    {
        std::cerr << "SingleJointMove RPC failed: " << status.error_message() << '\n';
    }
    return response.success();
}

bool H10WGrpcParam::MultiJointMove(std::vector<MoveParams> params, uint32_t &token)
{
    if (params.empty())
    {
        std::cout << "Invalid joint " << std::endl;
        return false;
    }
    MultiJointMoveRequest request;

    for (const auto &joint_param : params)
    {
        JointAngle *joint = request.add_joint_angles();
        joint->set_joint_index(joint_param.joint_index);
        joint->set_target_position(joint_param.target_position);
        joint->set_velocity(joint_param.velocity);
    }
    MultiJointMoveResponse response;
    ClientContext ctx;
    Status status = m_motion_stub_->RequestMultiJointsMove(&ctx, request, &response);
    if (status.ok())
    {
        token = response.token();
        std::cout << "MultiJointsMove succeeded: " << response.success() << ", Token: " << token << std::endl;
    }
    else
    {
        std::cerr << "MultiJointsMove RPC failed: " << status.error_message() << '\n';
    }
    return response.success();
}

bool H10WGrpcParam::LinearMovel(std::vector<LinearMoveParams> params, uint32_t &token)
{
    LinearMoveRequest request;
    for (const auto &linear_param : params)
    {
        LinearMove *Linear = request.add_linear_move();
        Linear->set_type(linear_param.type);
        for (double val : linear_param.pose)
        {
            Linear->add_pose(val);
        }
        Linear->set_velocity_percent(linear_param.velocity_percent);
        Linear->set_acceleration_percent(linear_param.acceleration_percent);
    }
    LinearMoveResponse response;
    ClientContext ctx;
    Status status = m_motion_stub_->RequestLinearMove(&ctx, request, &response);
    if (status.ok())
    {
        token = response.token();
        std::cout << "LinearMove succeeded: " << response.success() << ", token: " << token << std::endl;
    }
    else
    {
        std::cerr << "LinearMove RPC failed: " << status.error_message() << '\n';
    }
    return response.success();
}
