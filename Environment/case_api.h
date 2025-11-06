#pragma once
#include <string>
#include <map>
#include <gtest/gtest.h>

struct TestCaseParam {
    std::string name; 
    std::map<std::string, std::string> params; 
};
struct TestTaskConfig {
    std::string report_path;
};
extern TestTaskConfig g_test_config;

bool parse_test_arguments(int argc, char** argv, TestTaskConfig& config);
bool save_html_report(const std::string& report_path, const TestTaskConfig& config);
std::string get_current_time_str();
