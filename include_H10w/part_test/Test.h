#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include "H10wRosClient.h"
#include "H10wGrpcMove.h"
#include "H10Wglobalconstants.h"
#include "DeviceControlServiceClient.h"
#include "humanoid_controller_client.h"
// 1. GrpcParams模块测试类
class GrpcParamsTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        std::cout<<"SetUpTestSuite"<<std::endl;
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        grpc_params_client_ = std::make_shared<H10wGrpcMove>(GlobalConstants::H10W::IpPort);
    }

    static void TearDownTestSuite()
    {
        std::cout<<"TearDownTestSuite"<<std::endl;
        if (grpc_params_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            grpc_params_client_.reset();
        }
    }

    void SetUp() override
    {
        std::cout<<"SetUp"<<std::endl;
        // while (rclcpp::ok() && !grpc_params_client_->has_move_msg())
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        // m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        // m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        std::cout<<"TearDown"<<std::endl;
        // m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        // m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
    }

    // 模块共享的客户端实例
    inline static std::shared_ptr<H10wGrpcMove> grpc_params_client_;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::thread spin_thread_; // 每个用例独立线程
};
// 2. GrpcMotion模块测试类
class GrpcMotionTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        grpc_motion_client_ = std::make_shared<H10wGrpcMove>(GlobalConstants::H10W::IpPort);
    }

    static void TearDownTestSuite()
    {
        if (grpc_motion_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            grpc_motion_client_.reset();
        }
    }

    void SetUp() override
    {
        spin_thread_ = std::thread([this]()
                                   {
            if (grpc_motion_client_) {
                rclcpp::spin(grpc_motion_client_);
            } });
        while (rclcpp::ok() && !grpc_motion_client_->has_move_msg())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    inline static std::shared_ptr<H10wGrpcMove> grpc_motion_client_;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::thread spin_thread_;
};

// 3. Ros2Params模块测试类
class Ros2ParamsTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        ros_params_client_ = std::make_shared<H10wRosClient>(GlobalConstants::H10W::IpPort);
    }

    static void TearDownTestSuite()
    {
        if (ros_params_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            ros_params_client_.reset();
        }
    }

    void SetUp() override
    {
        spin_thread_ = std::thread([this]()
                                   {
            if (ros_params_client_) {
                rclcpp::spin(ros_params_client_);
            } });
        while (rclcpp::ok() && !ros_params_client_->has_move_msg())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    inline static std::shared_ptr<H10wRosClient> ros_params_client_;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::thread spin_thread_;
};

// 4. Ros2Motion模块测试类
class Ros2MotionTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        ros_motion_client_ = std::make_shared<H10wRosClient>(GlobalConstants::H10W::IpPort);
    }

    static void TearDownTestSuite()
    {
        if (ros_motion_client_ && rclcpp::ok())
        {
            rclcpp::shutdown();
            ros_motion_client_.reset();
        }
    }

    void SetUp() override
    {
        spin_thread_ = std::thread([this]()
                                   {
            if (ros_motion_client_) {
                rclcpp::spin(ros_motion_client_);
            } });
        while (rclcpp::ok() && !ros_motion_client_->has_move_msg())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::ON);
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::ON, true);
    }

    void TearDown() override
    {
        m_pDevCtrlSvrClient->controlBrakeStatus(BRAKE_STATUS::OFF, true);
        m_pDevCtrlSvrClient->controlPowerStatus(POWER_STATUS::OFF);
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    inline static std::shared_ptr<H10wRosClient> ros_motion_client_;
    std::unique_ptr<CDeviceControlServiceClient> m_pDevCtrlSvrClient;
    std::thread spin_thread_;
};