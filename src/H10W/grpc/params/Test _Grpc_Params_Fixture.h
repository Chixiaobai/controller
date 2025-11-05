#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include "H10wGrpcMove.h"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
class GrpcParamsTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        std::cout << "SetUpTestSuite" << std::endl;
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        grpc_params_client_ = std::make_shared<H10wGrpcMove>(GlobalConstants::H10W::IpPort);
    }

    static void TearDownTestSuite()
    {
        std::cout << "TearDownTestSuite" << std::endl;
        if (grpc_params_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            grpc_params_client_.reset();
        }
    }

    void SetUp() override
    {
        std::cout << "SetUp" << std::endl;
        // while (rclcpp::ok() && !grpc_params_client_->has_move_msg())
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        grpc_params_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        grpc_params_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        std::cout << "TearDown" << std::endl;
        grpc_params_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        grpc_params_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
    }

    inline static std::shared_ptr<H10wGrpcMove> grpc_params_client_;
};