#include <iostream>

#include "DeviceControlServiceClient.h"

CDeviceControlServiceClient::CDeviceControlServiceClient(std::shared_ptr<grpc::Channel> pChannel)
    : m_pStub(H10WService::DeviceControlService::NewStub(pChannel)) {}

bool CDeviceControlServiceClient::controlBrakeStatus(BRAKE_STATUS eBrakeStatus, bool bAutoClearError) {
    // Setup request
    auto pRequest = std::make_unique<H10WService::BrakeControlRequest>();
    // auto uintBrakeStatus = static_cast<uint32_t>(eBrakeStatus);
    pRequest->set_request_state(
        (::H10WService::BrakeControlRequest_BRAKE_STATE)eBrakeStatus);
    pRequest->set_auto_clear_error(bAutoClearError);

    // Invoke service
    auto pResponse = std::make_unique<H10WService::BrakeControlResponse>();
    grpc::ClientContext oContext;
   
    grpc::Status oStatus = m_pStub->ControlBrakeStatus(&oContext, *pRequest, pResponse.get());
    
    // Output invoking service result
    auto bServiceResult = oStatus.ok();
    
    std::cout << "Control brake result: " << (bServiceResult ? "OK" : "FAIL")
              << ", request brake status: " << ((BRAKE_STATUS::OFF != eBrakeStatus) ? "ON" : "OFF");
    if (!bServiceResult) {
        std::cout << ", error code: " << oStatus.error_code() << ", error message: " << oStatus.error_message();
    }
    std::cout << std::endl;

    return bServiceResult;
}

bool CDeviceControlServiceClient::controlPowerStatus(POWER_STATUS ePowerStatus) {
    // Setup request
    auto pRequest = std::make_unique<H10WService::PowerControlRequest>();
    pRequest->set_request_state(
        (::H10WService::PowerControlRequest_POWER_STATE)ePowerStatus);

    // Invoke service
    auto pResponse = std::make_unique<H10WService::PowerControlResponse>();
    grpc::ClientContext oContext;
    grpc::Status oStatus = m_pStub->ControlPowerStatus(&oContext, *pRequest, pResponse.get());

    // Output invoking service result
    auto bServiceResult = oStatus.ok();
    std::cout << "Control power result: " << (bServiceResult ? "OK" : "FAIL")
              << ", request power status: " << ((POWER_STATUS::ON != ePowerStatus) ? "OFF" : "ON");
    if (!bServiceResult) {
        std::cout << ", error code: " << oStatus.error_code() << ", error message: " << oStatus.error_message();
    }
    std::cout << std::endl;

    return bServiceResult;
}

bool CDeviceControlServiceClient::GetErrorCode(H10WService::PART_INDEX ePartIndex, std::string& errorCode){
    // Setup request
    auto pRequest = std::make_unique<H10WService::GetErrorCodeRequest>();
    pRequest->set_part_index(ePartIndex);

    // Invoke service
    auto pResponse = std::make_unique<H10WService::GetErrorCodeResponse>();
    grpc::ClientContext oContext;
    grpc::Status oStatus =
        m_pStub->GetJointErrorCode(&oContext, *pRequest, pResponse.get());

    // Output invoking service result
    auto bServiceResult = oStatus.ok() & pResponse->success();
    std::cout << "Connect GRPC:" << (oStatus.ok() ? "OK" : "FAIL")
                << ", Get Error result: "
                << (pResponse->success() ? "OK" : "FAIL");
    if (!bServiceResult) {
        std::cout << ", error code: " << oStatus.error_code() << ", error message: " << oStatus.error_message();
    }
    errorCode = pResponse->error_code();
    std::cout << std::endl;

    return bServiceResult;
}

bool CDeviceControlServiceClient::ClearError(H10WService::JOINT_INDEX eJointIndex) {
  // Setup request
  auto pRequest = std::make_unique<H10WService::ClearErrorRequest>();
  pRequest->set_joint_index(eJointIndex);

  // Invoke service
  auto pResponse = std::make_unique<H10WService::ClearErrorResponse>();
  grpc::ClientContext oContext;
  grpc::Status oStatus =
      m_pStub->ClearError(&oContext, *pRequest, pResponse.get());

  // Output invoking service result
  auto bServiceResult = oStatus.ok() & pResponse->success();
  std::cout << "Connect GRPC:" << (oStatus.ok() ? "OK" : "FAIL")
            << ", Clear Error result: "
            << (pResponse->success() ? "OK" : "FAIL");
  if (!bServiceResult) {
    std::cout << ", error code: " << oStatus.error_code()
              << ", error message: " << oStatus.error_message();
  }
  std::cout << std::endl;

  return bServiceResult;
}
