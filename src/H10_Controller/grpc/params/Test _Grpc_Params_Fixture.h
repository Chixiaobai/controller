#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <signal.h>
#include "xmlHandler.h"
#include "H10wGrpcMove.h"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
namespace fs = std::filesystem;
class GrpcParamsTest : public testing::Test
{
protected:
    inline static H10wGrpcMove *g_pTester = nullptr;
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
        const std::string robotConfigPath = std::filesystem::path(__FILE__).parent_path() / "config" / "config.xml";
        fs::path fPath(robotConfigPath);
        xml_handler = new XML_HANDLER(fPath.string());

        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        grpc_params_client_ = std::make_shared<H10wGrpcMove>(GlobalConstants::H10W::IpPort);
        std::thread spin_thread([]()
                                {
                                    rclcpp::spin(grpc_params_client_); // 持续处理回调
                                });
        spin_thread.detach();
        g_pTester = grpc_params_client_.get();
        setConsoleHandler();
    }

    static void TearDownTestSuite()
    {
        std::cout << "TearDownTestSuite" << std::endl;
        if (grpc_params_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            grpc_params_client_.reset();
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

        // while (rclcpp::ok() && !grpc_params_client_->has_move_msg())
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        // grpc_params_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        // grpc_params_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        std::cout << "TearDown" << std::endl;
        // grpc_params_client_->m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        // grpc_params_client_->m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
    }

    inline static std::shared_ptr<H10wGrpcMove> grpc_params_client_;

    bool check_soft_limits_match(const std::vector<std::tuple<uint32_t, double, double>> &soft_limits,
                                 const std::map<uint32_t, std::pair<float, float>> &expect_limit)
    {
        for (auto &item : soft_limits)
        {
            auto [joint_id, act_max, act_min] = item;
            auto [exp_max, exp_min] = expect_limit.at(joint_id);
            if (!isFloatValueEqual(act_max, exp_max) || !isFloatValueEqual(act_min, exp_min))
            {
                std::cout << "关节 " << joint_id << " 限位不匹配："
                          << "预期(max:" << exp_max << ", min:" << exp_min << ")，"
                          << "实际(max:" << act_max << ", min:" << act_min << ")" << std::endl;
                return false;
            }
        }
        return true;
    }
};
