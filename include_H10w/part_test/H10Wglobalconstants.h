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

    }
}
