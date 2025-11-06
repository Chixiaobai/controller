#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <signal.h>
#include "xmlHandler.h"
#include "H10wRosClient.h"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
namespace fs = std::filesystem;
class Ros2ParamsTest : public testing::Test
{
protected:
    inline static H10wRosClient *g_pTester = nullptr;
    inline static struct sigaction originalSigInt = {0};
    inline static struct sigaction originalSigTerm = {0};
    inline static std::map<std::string, std::vector<float>> robotParameters;
    inline static XML_HANDLER *xml_handler = nullptr;
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
        std::string robotConfigPath;
        Get_robot_config_file_path(robotConfigPath);
        fs::path fPath(robotConfigPath);
        xml_handler = new XML_HANDLER(fPath.string());

        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        ros_params_client_ = std::make_shared<H10wRosClient>(GlobalConstants::H10W::IpPort);
        std::thread spin_thread([]()
                                {
                                    rclcpp::spin(ros_params_client_); // 持续处理回调
                                });
        spin_thread.detach();
        g_pTester = ros_params_client_.get();
        setConsoleHandler();
    }

    static void TearDownTestSuite()
    {
        std::cout << "TearDownTestSuite" << std::endl;
        if (ros_params_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            ros_params_client_.reset();
        }
        g_pTester = nullptr;
        delete xml_handler;
        xml_handler = nullptr;
        
        sigaction(SIGINT, &originalSigInt, nullptr);
        sigaction(SIGTERM, &originalSigTerm, nullptr);
    }

    void SetUp() override
    {
        std::cout << "SetUp" << std::endl;
        robotParameters = xml_handler->get_parameters();
        while (rclcpp::ok() && !ros_params_client_->has_move_msg())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // ros_params_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        ros_params_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        std::cout << "TearDown" << std::endl;
        ros_params_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        // ros_params_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
    }

    inline static std::shared_ptr<H10wRosClient> ros_params_client_;
};