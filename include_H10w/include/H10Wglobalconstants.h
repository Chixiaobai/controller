#pragma once

#include <cstdint>
#include <array>
#include <chrono>
#include "main.h"
#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>
#include <cassert>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

namespace GlobalConstants
{
    namespace H10W
    {

        enum MOTOR_DATA
        {
            ARM_L_JOINT_1 = 0,
            ARM_L_JOINT_2,
            ARM_L_JOINT_3,
            ARM_L_JOINT_4,
            ARM_L_JOINT_5,
            ARM_L_JOINT_6,
            ARM_L_JOINT_7,
            ARM_R_JOINT_1,
            ARM_R_JOINT_2,
            ARM_R_JOINT_3,
            ARM_R_JOINT_4,
            ARM_R_JOINT_5,
            ARM_R_JOINT_6,
            ARM_R_JOINT_7,
            HEAD_JOINT_TILT,
            HEAD_JOINT_ROTATE,
            MOTOR_DATA_SIZE
        };
        constexpr int32_t BASE_JOINT_COUNT = 2;
        constexpr uint32_t SINGLE_ARM_JOINT_COUNT = 7;
        constexpr uint32_t HEAD_JOINT_COUNT = 2;
        constexpr uint32_t GRIPPER_JOINT_COUNT = 2;
        constexpr uint32_t ALL_JOINT_COUNT = 19;

        // Topic Name
        constexpr const char *STATE_TOPIC_NAME = "rt/low_state";

        // Time interval
        constexpr std::chrono::milliseconds WAIT_DRIVER_INTERVAL(100);
        constexpr std::chrono::seconds WAIT_DRIVER_TIMEOUT(30);
        constexpr std::chrono::milliseconds OPERATION_INTERVAL(1000);
        constexpr std::chrono::milliseconds READ_INTERVAL(100);

        // Ranges
        constexpr std::array<float, 2> ELEVATOR_RANGE = {{0.0f, 0.827f}};
        constexpr std::array<std::array<float, 2>, 7> ARM_JOINTS_RANGES = {{{{-3.1230f, 3.1230f}},
                                                                            {{-1.5708f, 1.5708f}},
                                                                            {{-3.1230f, 3.1230f}},
                                                                            {{0.0000f, 3.0543f}},
                                                                            {{-3.1230f, 3.1230f}},
                                                                            {{-3.1230f, 3.1230f}},
                                                                            {{-3.1230f, 3.1230f}}}};

        constexpr std::array<float, 2> HEAD_JOINT_TILT_RANGE = {{-0.785f, 0.785f}};
        constexpr std::array<float, 2> HEAD_JOINT_ROTATE_RANGE = {{-0.698f, 0.698f}};
        constexpr std::array<float, 2> ARM_WAVE_RANGE = {{1.1f, 2.3f}};

        const float HEAD_MOVE_OFFSET = to_rad(0.02);
        const float ARM_MOVE_OFFSET = to_rad(0.1);
        const float ELEVATOR_MOVE_OFFSET = to_rad(0.01);

        constexpr float BASE_TEST_WHEEL_VELOCITY = 0.03f;

        constexpr uint32_t WAVE_ARM_COUNT = 300;

        // inline std::string IpPort = "192.168.1.75";
        inline std::string IpPort = "127.0.0.1";

        struct ControllerVersion
        {
            std::string main;
            std::map<std::string, std::string> plugins;
            ControllerVersion(std::string m, std::map<std::string, std::string> p) : main(m), plugins(p) {}
        };

        struct MoveParams
        {
            uint32_t joint_index;
            double target_position;
            double velocity;
            MoveParams()= default;
            MoveParams(uint32_t idx, double pos, double vel) : joint_index(idx), target_position(pos), velocity(vel) {}
        };

        struct LinearMoveParams
        {
            int32_t type;
            std::vector<double> pose;
            double velocity_percent;
            double acceleration_percent;
            LinearMoveParams() = default;
            LinearMoveParams(int32_t t, std::vector<double> p, double v, double a) : type(t), pose(p), velocity_percent(v), acceleration_percent(a) {}
        };

        struct JointSoftLimitParams

        {
            uint32_t joint_index; // 关节索引
            double max_pos;       // 最大位置
            double min_pos;       // 最小位置
            JointSoftLimitParams() = default;
            JointSoftLimitParams(uint32_t idx, double max, double min) : joint_index(idx), max_pos(max), min_pos(min) {}
            bool operator==(const JointSoftLimitParams &o) const noexcept
            {
                return joint_index == o.joint_index &&
                       isFloatValueEqual(max_pos, o.max_pos, 1e-6) &&
                       isFloatValueEqual(min_pos, o.min_pos, 1e-6);
            }
        };

        struct JointMaxParams
        {
            uint32_t joint_index; // 关节索引
            double value;         // 关节参数值（速度/加速度等）
            JointMaxParams() = default;
            JointMaxParams(uint32_t idx, double val) : joint_index(idx), value(val) {}
            bool operator==(const JointMaxParams &o) const noexcept
            {
                return joint_index == o.joint_index &&
                       isFloatValueEqual(value, o.value, 1e-6);
            }
        };

        struct CartMaxParams
        {
            uint32_t cartesian_index; // 笛卡尔坐标索引
            double value;             // 坐标参数值（速度/加速度等）
            CartMaxParams() = default;
            CartMaxParams(uint32_t idx, double val) : cartesian_index(idx), value(val) {}
        };

        struct TcpParam
        {
            int32_t type = 0;         // TCP类型
            std::vector<double> data; // TCP参数数据（偏移量/负载等）
            TcpParam() = default;
            TcpParam(int32_t t, const std::vector<double> &d) : type(t), data(d) {}
        };

        struct ForwardRequest
        {
            std::vector<int32_t> type;        // 类型列表
            std::vector<double> joint_angles; // 关节角度列表
            ForwardRequest() = default;
            ForwardRequest(const std::vector<int32_t> &t, const std::vector<double> &j) : type(t), joint_angles(j) {}
        };

        struct KinematicsParams
        {
            std::vector<int32_t> type;             // 类型列表
            std::vector<std::vector<double>> pose; // 位姿列表
            std::vector<double> joint_angle;
            bool if_use_whole_body = false; // 是否使用全身解算
            KinematicsParams(const std::vector<int32_t> &t, const std::vector<double> &j, const std::vector<std::vector<double>> &p, bool i) : type(t), joint_angle(j), pose(p), if_use_whole_body(i) {}
        };

    }
}
