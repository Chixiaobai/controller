#pragma once

#include <cstdint>
#include <memory>

#include <grpcpp/grpcpp.h>

#include "../rpc_service/service/Service.grpc.pb.h"

enum class BRAKE_STATUS : uint32_t { OFF = 0, ON = 1 };

enum class POWER_STATUS : uint32_t { ON = 0, OFF = 1 };

class CDeviceControlServiceClient final {
public:
    CDeviceControlServiceClient(std::shared_ptr<grpc::Channel> pChannel);
    ~CDeviceControlServiceClient() = default;

    // Switch on/off brake
    bool controlBrakeStatus(BRAKE_STATUS eBrakeStatus, bool bAutoClearError);

    // Switch on/off power
    bool controlPowerStatus(POWER_STATUS ePowerStatus);

    // GetError
    bool GetErrorCode(H10WService::PART_INDEX ePartIndex, std::string& errorCode);

    // ClearError
    bool ClearError(H10WService::JOINT_INDEX eJointIndex);

  private:
    std::unique_ptr<H10WService::DeviceControlService::Stub> m_pStub;
};
