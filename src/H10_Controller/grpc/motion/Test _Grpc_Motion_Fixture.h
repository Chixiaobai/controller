#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <signal.h>
#include "H10wGrpcMove.h"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
class GrpcMotionTest : public testing::Test
{
protected:
    inline static H10wGrpcMove *g_pTester = nullptr;
    inline static struct sigaction originalSigInt = {0};
    inline static struct sigaction originalSigTerm = {0};
    std::map<std::string, std::vector<float>> robotParameters;
    static void consoleHandler(int intSigNum)
    {
        if ((SIGINT == intSigNum) || (SIGTERM == intSigNum))
        {
            if (nullptr != g_pTester)
            {
                std::cout << "consoleHandler: received ctrl+c signal " << std::endl;
                g_pTester->m_pControllerClient->stop();
                rclcpp::shutdown();
            }
            sigaction(SIGINT, &originalSigInt, nullptr);
            sigaction(SIGTERM, &originalSigTerm, nullptr);
        }
    }

    static void setConsoleHandler()
    {
        struct sigaction stSigAction;
        stSigAction.sa_handler = &consoleHandler;
        sigemptyset(&stSigAction.sa_mask);
        stSigAction.sa_flags = 0;

        sigaction(SIGINT, nullptr, &originalSigInt);
        sigaction(SIGTERM, nullptr, &originalSigTerm);

        if (sigaction(SIGINT, &stSigAction, nullptr) != 0 ||
            sigaction(SIGTERM, &stSigAction, nullptr) != 0)
        {
            printf("Fail to set callback function for console application!\n");
            fflush(stdout);
        }
    }
    static void SetUpTestSuite()
    {
        std::cout << "SetUpTestSuite" << std::endl;

        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        grpc_motion_client_ = std::make_shared<H10wGrpcMove>(GlobalConstants::H10W::IpPort);
        g_pTester = grpc_motion_client_.get();
        setConsoleHandler();
    }

    static void TearDownTestSuite()
    {
        std::cout << "TearDownTestSuite" << std::endl;
        if (grpc_motion_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            grpc_motion_client_.reset();
        }
        g_pTester = nullptr;
        sigaction(SIGINT, &originalSigInt, nullptr);
        sigaction(SIGTERM, &originalSigTerm, nullptr);
    }

    void SetUp() override
    {
        std::cout << "SetUp" << std::endl;
        std::string robotConfigPath;
        Get_robot_config_file_path(robotConfigPath);
        robotParameters = getTestParameters(robotConfigPath);

        // while (rclcpp::ok() && !grpc_motion_client_->has_move_msg())
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        // grpc_motion_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        // grpc_motion_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        std::cout << "TearDown" << std::endl;
        // grpc_motion_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        // grpc_motion_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
    }

    inline static std::shared_ptr<H10wGrpcMove> grpc_motion_client_;
};