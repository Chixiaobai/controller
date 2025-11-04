#pragma once

#include <gtest/gtest.h>
#include <string>

// 测试用例基础信息（仅保留必要元数据，不关联旧框架）
struct TestCaseMeta
{
    std::string case_id;
    std::string description; 
};

#define REGISTER_TEST_META(case_group, case_name, desc)          \
    static TestCaseMeta case_meta_##case_group##_##case_name = { \
        #case_group "_" #case_name,                              \
        desc};

#define GTEST_CASE(case_group, case_name, desc)                     \
    void case_##case_group##_##case_name##_body();                  \
    REGISTER_TEST_META(case_group, case_name, desc);                \
    TEST(case_group, case_name)                                     \
    {                                                               \
        extern TestTaskConfig g_test_config;                        \
        std::string group = #case_group;                            \
        std::string full_case = group + "." + #case_name;           \
                                                                    \
        if (!g_test_config.case_type.empty())                       \
        {                                                           \
            bool is_match = (g_test_config.case_type == group) ||   \
                            (g_test_config.case_type == full_case); \
            if (!is_match)                                          \
            {                                                       \
                GTEST_SKIP() << "非指定组/用例，跳过";              \
            }                                                       \
        }                                                           \
        else                                                        \
        {                                                           \
            if (group.substr(0, 4) != "auto")                       \
            {                                                       \
                GTEST_SKIP() << "非自动执行用例组，默认跳过";       \
            }                                                       \
        }                                                           \
        case_##case_group##_##case_name##_body();                   \
    }                                                               \
    void case_##case_group##_##case_name##_body()
