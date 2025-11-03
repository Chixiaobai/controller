#pragma once
#include <string>
#include <map>
#include "case_def.h"
#include <gtest/gtest.h>

struct TestTaskConfig {
    std::string case_type;        // 改为：测试用例所在文件夹名称
    std::string robot_type;       // 机器人类型（如"H10w"）
    std::string case_filter;      // 保留（但不再对外展示）
    std::string report_path;      // 测试报告路径
    std::map<std::string, std::string> extra_params;  // 其他扩展参数
};
extern TestTaskConfig g_test_config;

bool parse_test_arguments(int argc, char** argv, TestTaskConfig& config);
bool save_html_report(const std::string& report_path, const TestTaskConfig& config);
std::string get_current_time_str();
