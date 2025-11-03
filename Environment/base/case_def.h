#pragma once

#include <gtest/gtest.h>
#include <string>

// 测试用例基础信息（仅保留必要元数据，不关联旧框架）
struct TestCaseMeta
{
    std::string case_id;     // 测试用例唯一标识
    std::string description; // 用例描述
};

// 注册测试用例元信息（仅用于记录，不影响GTest执行流程）
#define REGISTER_TEST_META(case_group, case_name, desc)          \
    static TestCaseMeta case_meta_##case_group##_##case_name = { \
        #case_group "_" #case_name,                              \
        desc};

// 修改case_def.h中的GTEST_CASE宏定义部分
#define GTEST_CASE(case_group, case_name, desc)                            \
    void case_##case_group##_##case_name##_body();                         \
    REGISTER_TEST_META(case_group, case_name, desc);                       \
    TEST(case_group, case_name)                                            \
    {                                                                      \
        extern TestTaskConfig g_test_config;                               \
        std::cout << "DEBUG: case_type='" << g_test_config.case_type       \
                  << "', case_group='" << #case_group << "'" << std::endl; \
        if (g_test_config.case_type.empty())                             \
        {                                                                  \
            if (std::string(#case_group).substr(0, 4) != "auto")           \
            {                                                              \
                GTEST_SKIP() << "非自动执行用例组，默认跳过";              \
            }                                                              \
        }                                                                  \
        case_##case_group##_##case_name##_body();                          \
    }                                                                      \
    void case_##case_group##_##case_name##_body()