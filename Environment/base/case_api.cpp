#include "case_api.h"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <iomanip>
#include <string>

using namespace testing;
// 在parse_test_arguments函数中，修改过滤规则生成逻辑
bool parse_test_arguments(int argc, char **argv, TestTaskConfig &config)
{
    std::cout << "解析-case参数：" << std::endl;
    // 初始化默认过滤规则（执行所有用例，不提前过滤非auto组）
    config.case_filter = "*"; // 默认执行所有用例

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-case" && i + 1 < argc)
        {
            std::string case_input = argv[++i];
            config.case_type = case_input;

            // 仅当指定了-case参数时才修改过滤规则
            if (case_input.find('.') != std::string::npos)
            {
                config.case_filter = case_input; // 单个用例
            }
            else
            {
                config.case_filter = case_input + ".*"; // 批次用例
            }
        }
        else if (arg == "-robot" && i + 1 < argc)
        {
            config.robot_type = argv[++i];
        }
        else if (arg == "-report" && i + 1 < argc)
        {
            config.report_path = argv[++i];
        }
    }

    if (config.robot_type.empty())
    {
        std::cerr << "错误：必须使用-robot指定机器人类型（如-robot H10-W）" << std::endl;
        return false;
    }

    return true;
}

bool save_html_report(const std::string &report_path, const TestTaskConfig &config) {
    const UnitTest* unit_test = UnitTest::GetInstance();
    if (!unit_test) {
        std::cerr << "Failed to get UnitTest instance" << std::endl;
        return false;
    }

    std::ofstream ofs(report_path);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open report file: " << report_path << std::endl;
        return false;
    }

    std::time_t now = std::time(nullptr);
    std::tm tm = *std::localtime(&now);
    std::stringstream time_ss;
    time_ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    std::string current_time = time_ss.str();

    std::map<std::string, int> skipped_suite_count; 
    int total_skipped_all = 0;

    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx) {
        const TestSuite* suite = unit_test->GetTestSuite(suite_idx);
        std::string suite_name = suite->name();
        int suite_skipped = 0;

        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx) {
            const TestInfo* test_case = suite->GetTestInfo(case_idx);
            const TestResult* case_result = test_case->result();
            if (case_result->Skipped()) {
                suite_skipped++;
                total_skipped_all++;
            }
        }

        if (suite_skipped > 0) {
            skipped_suite_count[suite_name] = suite_skipped;
        }
    }

    int filtered_total = 0;
    int filtered_passed = 0;
    int filtered_failed = 0;
    int filtered_skipped = 0;

    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx) {
        const TestSuite* suite = unit_test->GetTestSuite(suite_idx);
        std::string suite_name = suite->name();
        bool is_target_suite = false;

        if (!config.case_type.empty()) {
            is_target_suite = (suite_name == config.case_type);
        } else {
            is_target_suite = (suite_name.substr(0, 4) == "auto");
        }

        if (!is_target_suite || suite->total_test_count() == 0) {
            continue;
        }

        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx) {
            const TestInfo* test_case = suite->GetTestInfo(case_idx);
            const TestResult* case_result = test_case->result();
            filtered_total++;

            if (case_result->Passed()) filtered_passed++;
            else if (case_result->Failed()) filtered_failed++;
            else if (case_result->Skipped()) filtered_skipped++;
        }
    }

    ofs << R"(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>测试报告</title>
    <style type="text/css">
        body {
            font-family: "Microsoft YaHei", Arial, sans-serif;
            font-size: 14px;
            margin: 0;
            padding: 20px;
            background-color: #f9f9f9;
        }
        .report-title {
            text-align: center;
            font-size: 24px;
            color: #333;
            margin-bottom: 30px;
            font-weight: bold;
        }
        .report-info {
            text-align: center;
            color: #666;
            margin-bottom: 20px;
            line-height: 1.8;
        }
        .stats-table {
            width: 80%;
            margin: 0 auto 40px;
            border-collapse: collapse;
            background-color: #fff;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .stats-table th, .stats-table td {
            border: 1px solid #ddd;
            padding: 12px 15px;
            text-align: center;
        }
        .stats-table th {
            background-color: #428bca;
            color: #fff;
            font-weight: bold;
        }
        .stats-table td {
            font-size: 16px;
        }
        .total {
            color: #333;
        }
        .passed {
            color: #5cb85c;
            font-weight: bold;
        }
        .failed {
            color: #d9534f;
            font-weight: bold;
        }
        .skipped {
            color: #f0ad4e;
            font-weight: bold;
        }
        .skipped-stats-container {
            width: 95%;
            margin: 0 auto 40px;
            background-color: #fff;
            border-radius: 8px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            padding: 20px;
        }
        .skipped-stats-title {
            font-size: 18px;
            color: #333;
            margin-bottom: 15px;
            font-weight: bold;
            border-bottom: 1px solid #eee;
            padding-bottom: 10px;
        }
        .skipped-suite-list {
            color: #666;
            line-height: 1.6;
            padding-left: 20px;
        }
        .cases-table {
            width: 95%;
            margin: 0 auto;
            border-collapse: collapse;
            background-color: #fff;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .cases-table th {
            background-color: #f8f8f8;
            border: 1px solid #ddd;
            padding: 10px 15px;
            text-align: left;
            font-weight: bold;
            color: #333;
        }
        .cases-table td {
            border: 1px solid #ddd;
            padding: 10px 15px;
            color: #666;
        }
        .cases-table tr:nth-child(even) {
            background-color: #f9f9f9;
        }
        .cases-table tr:hover {
            background-color: #f1f1f1;
        }
        .status-tag {
            display: inline-block;
            padding: 4px 8px;
            border-radius: 4px;
            color: #fff;
            font-size: 12px;
            font-weight: bold;
        }
        .status-pass {
            background-color: #5cb85c;
        }
        .status-fail {
            background-color: #d9534f;
        }
        .status-skip {
            background-color: #f0ad4e;
        }
        .fail-detail {
            margin-top: 8px;
            padding: 10px;
            background-color: #f2dede;
            border-radius: 4px;
            font-size: 12px;
            color: #a94442;
            white-space: pre-wrap;
            word-break: break-all;
        }
    </style>
</head>
<body>
    <h1 class="report-title">测试报告</h1>
    <div class="report-info">
        执行时间：)" << current_time << R"(<br>
        执行范围：)" << (config.case_type.empty() ? "自动化测试用例" : config.case_type) << R"(
    </div>

    <table class="stats-table">
        <tr>
            <th>总用例数</th>
            <th>通过</th>
            <th>失败</th>
            <th>跳过</th>
        </tr>
        <tr>
            <td class="total">)" << filtered_total << R"(</td>
            <td class="passed">)" << filtered_passed << R"(</td>
            <td class="failed">)" << filtered_failed << R"(</td>
            <td class="skipped">)" << total_skipped_all << R"(</td>
        </tr>
    </table>

    <div class="skipped-stats-container">
        <div class="skipped-stats-title">跳过</div>
        )" << (total_skipped_all == 0 ? R"(<div style="color: #666; text-align: center; padding: 10px;">无跳过的测试用例</div>)" : "") << R"(
        <div class="skipped-suite-list">
    )";

    for (const auto& [suite_name, count] : skipped_suite_count) {
        ofs << R"(
            <li>模块：)" << suite_name << R"(: 跳过 )" << count << R"( 条用例</li>
        )";
    }

    ofs << R"(
        </div>
    </div>

    <table class="cases-table">
        <thead>
            <tr>
                <th>模块</th>
                <th>测试用例</th>
                <th>执行结果</th>
                <th>执行时间(ms)</th>
                <th>失败详情</th>
            </tr>
        </thead>
        <tbody>
    )";

    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx) {
        const TestSuite* suite = unit_test->GetTestSuite(suite_idx);
        std::string suite_name = suite->name();
        bool is_target_suite = false;

        if (!config.case_type.empty()) {
            is_target_suite = (suite_name == config.case_type);
        } else {
            is_target_suite = (suite_name.substr(0, 4) == "auto");
        }

        if (!is_target_suite || suite->total_test_count() == 0) {
            continue;
        }

        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx) {
            const TestInfo* test_case = suite->GetTestInfo(case_idx);
            const TestResult* case_result = test_case->result();
            std::string case_name = test_case->name();
            std::string status_text, status_class;
            std::string detail = "-";
            double elapsed = case_result->elapsed_time() / 1000.0;

            if (case_result->Passed()) {
                status_text = "通过";
                status_class = "status-pass";
            } else if (case_result->Failed()) {
                status_text = "失败";
                status_class = "status-fail";
                std::string fail_msg;
                for (int i = 0; i < case_result->total_part_count(); ++i) {
                    const TestPartResult& part = case_result->GetTestPartResult(i);
                    if (part.failed()) {
                        fail_msg += "行号：" + std::to_string(part.line_number()) + "\n";
                        fail_msg += "错误信息：" + std::string(part.message());
                    }
                }
                detail = R"(<div class="fail-detail">)" + fail_msg + R"(</div>)";
            } else if (case_result->Skipped()) {
                status_text = "跳过";
                status_class = "status-skip";
                detail = "该用例未执行（跳过原因：按执行范围过滤或主动跳过）";
            }

            ofs << R"(
                <tr>
                    <td>)" << suite_name << R"(</td>
                    <td>)" << case_name << R"(</td>
                    <td><span class="status-tag )" << status_class << R"(">)" << status_text << R"(</span></td>
                    <td>)" << std::fixed << std::setprecision(2) << elapsed << R"(</td>
                    <td>)" << detail << R"(</td>
                </tr>
            )";
        }
    }

    ofs << R"(
        </tbody>
    </table>
</body>
</html>
    )";

    ofs.close();
    std::cout << "[报告生成] HTML报告已保存至：" << report_path << std::endl;
    return true;
}
// 获取当前时间字符串（Linux系统专用，格式：YYYY-MM-DD HH:MM:SS），线程安全
std::string get_current_time_str()
{
    // 1. 获取当前系统时间（秒级精度）
    std::time_t now_time_t = std::time(nullptr);

    // 2. Linux线程安全的本地时间转换（替换非线程安全的localtime）
    std::tm local_tm{};
    localtime_r(&now_time_t, &local_tm); // Linux专用线程安全接口

    // 3. 格式化时间（HH:MM:SS自动补零，如08:05:03）
    char time_buf[64] = {0};
    std::strftime(
        time_buf,
        sizeof(time_buf),
        "%Y-%m-%d %H:%M:%S", // 格式符：%H(24小时制)、%M(分钟)、%S(秒)
        &local_tm);

    return time_buf;
}