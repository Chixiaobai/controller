#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include "H10wGrpcMove.h"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
class GrpcMotionTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        std::cout<<"SetUpTestSuite"<<std::endl;
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        grpc_motion_client_ = std::make_shared<H10wGrpcMove>(GlobalConstants::H10W::IpPort);
    }

    static void TearDownTestSuite()
    {
        std::cout<<"TearDownTestSuite"<<std::endl;
        if (grpc_motion_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            grpc_motion_client_.reset();
        }
    }

    void SetUp() override
    {
        std::cout<<"SetUp"<<std::endl;
        // while (rclcpp::ok() && !grpc_motion_client_->has_move_msg())
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        std::cout<<"TearDown"<<std::endl;
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
    }

    inline static std::shared_ptr<H10wGrpcMove> grpc_motion_client_;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
};