#pragma once
#include <string>
#include <map>
#include <gtest/gtest.h>
// 测试用例参数结构体
struct TestCaseParam {
    std::string name;               // 用例名称
    std::map<std::string, std::string> params;  // 参数键值对
};
// 精简配置结构，仅保留报告路径
struct TestTaskConfig {
    std::string report_path;
};
extern TestTaskConfig g_test_config;

// 函数声明调整（仅解析-report参数）
bool parse_test_arguments(int argc, char** argv, TestTaskConfig& config);
bool save_html_report(const std::string& report_path, const TestTaskConfig& config);
std::string get_current_time_str();
