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

bool save_html_report(const std::string &report_path, const TestTaskConfig &config)
{
    const UnitTest *unit_test = UnitTest::GetInstance();
    if (!unit_test)
    {
        std::cerr << "Failed to get UnitTest instance" << std::endl;
        return false;
    }

    std::ofstream ofs(report_path);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open report file: " << report_path << std::endl;
        return false;
    }

    // 时间格式化
    std::time_t now = std::time(nullptr);
    std::tm tm = *std::localtime(&now);
    std::stringstream time_ss;
    time_ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    std::string current_time = time_ss.str();

    // 统计变量
    std::map<std::string, int> moduleTotal;                            // 模块→总用例数
    std::map<std::string, int> modulePassed;                           // 模块→通过数
    std::map<std::string, int> moduleFailed;                           // 模块→失败数
    std::map<std::string, int> moduleSkipped;                          // 模块→跳过数
    std::map<std::string, std::vector<std::string>> moduleFailDetails; // 模块→失败详情
    int totalAllCases = 0, totalPassed = 0, totalFailed = 0, totalSkipped = 0;

    // 遍历所有测试组，收集统计数据
    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx)
    {
        const TestSuite *suite = unit_test->GetTestSuite(suite_idx);
        std::string moduleName = suite->name();

        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const TestInfo *test_case = suite->GetTestInfo(case_idx);
            const TestResult *case_result = test_case->result();
            moduleTotal[moduleName]++;
            totalAllCases++;

            if (case_result->Skipped())
            {
                moduleSkipped[moduleName]++;
                totalSkipped++;
            }
            else if (case_result->Passed())
            {
                modulePassed[moduleName]++;
                totalPassed++;
            }
            else if (case_result->Failed())
            {
                moduleFailed[moduleName]++;
                totalFailed++;
                // 格式化失败信息（一行显示，直接在循环内处理）
                std::string failMsg;
                for (int i = 0; i < case_result->total_part_count(); ++i)
                {
                    const TestPartResult &part = case_result->GetTestPartResult(i);
                    if (part.failed())
                    {
                        // 1. 拼接行号和错误信息前缀
                        failMsg += "行号：" + std::to_string(part.line_number()) + " | 错误信息：";
                        // 2. 处理原始错误信息，替换换行和连续空格
                        std::string raw_msg = part.message();
                        // 替换换行符为空格
                        for (char &c : raw_msg)
                        {
                            if (c == '\n' || c == '\r')
                                c = ' ';
                        }
                        // 替换连续空格为单个空格
                        std::string compressed_msg;
                        bool prev_space = false;
                        for (char c : raw_msg)
                        {
                            if (c == ' ')
                            {
                                if (!prev_space)
                                {
                                    compressed_msg += c;
                                    prev_space = true;
                                }
                            }
                            else
                            {
                                compressed_msg += c;
                                prev_space = false;
                            }
                        }
                        // 去除首尾空格
                        if (!compressed_msg.empty() && compressed_msg.front() == ' ')
                        {
                            compressed_msg.erase(0, 1);
                        }
                        if (!compressed_msg.empty() && compressed_msg.back() == ' ')
                        {
                            compressed_msg.pop_back();
                        }
                        // 3. 拼接处理后的错误信息
                        failMsg += compressed_msg;
                    }
                }
                moduleFailDetails[moduleName].push_back(std::string(test_case->name()) + "：" + failMsg);
            }
        }
    }

    // 生成HTML报告
    ofs << R"(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>测试报告详情</title>
    <style type="text/css">
        body {
            font-family: "Microsoft YaHei", Arial, sans-serif;
            font-size: 14px;
            margin: 0;
            padding: 20px;
            background-color: #f9f9f9;
        }
        .report-header {
            background-color: #337ab7;
            color: white;
            padding: 15px;
            margin-bottom: 20px;
            text-align: center; /* 标题和执行信息居中 */
        }
        .report-title {
            font-size: 24px;
            font-weight: bold;
            margin: 0;
        }
        .report-meta {
            margin-top: 10px;
            color: #d9edf7;
            font-size: 12px;
        }
        .stats-container {
            display: flex;
            justify-content: space-around;
            margin-bottom: 30px;
        }
        .stat-card {
            background-color: white;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            padding: 15px;
            width: 20%;
            text-align: center;
        }
        .stat-label {
            font-size: 16px;
            margin-bottom: 5px;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
        }
        .passed { color: #5cb85c; }
        .failed { color: #d9534f; }
        .skipped { color: #f0ad4e; }
        .module-stats-table {
            width: 100%;
            border-collapse: collapse;
            background-color: white;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            margin-bottom: 30px;
        }
        .module-stats-table th, .module-stats-table td {
            border: 1px solid #ddd;
            padding: 10px;
            text-align: left;
        }
        .module-stats-table th {
            background-color: #f8f8f8;
            font-weight: bold;
        }
        .cases-table {
            width: 100%;
            border-collapse: collapse;
            background-color: white;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            margin-bottom: 30px;
        }
        .cases-table th, .cases-table td {
            border: 1px solid #ddd;
            padding: 10px;
            text-align: left;
        }
        .cases-table th {
            background-color: #f8f8f8;
            font-weight: bold;
        }
        .cases-table tr:nth-child(even) {
            background-color: #f9f9f9;
        }
        .status-tag {
            display: inline-block;
            padding: 3px 8px;
            border-radius: 3px;
            color: white;
            font-size: 12px;
            font-weight: bold;
        }
        .status-pass { background-color: #5cb85c; }
        .status-fail { background-color: #d9534f; }
        .status-skip { background-color: #f0ad4e; }
        .fail-details {
            margin-top: 8px;
            padding: 10px;
            background-color: #f2dede;
            border-radius: 4px;
            color: #a94442;
            white-space: nowrap; /* 不换行 */
            overflow-x: auto; /* 横向滚动（避免溢出） */
        }
    </style>
</head>
<body>
    <div class="report-header">
        <h1 class="report-title">测试报告详情</h1>
        <div class="report-meta">
            执行时间：)"
        << current_time << R"(<br>
            执行范围：)"
        << (config.case_type.empty() ? "所有auto开头的测试组" : config.case_type) << R"(
        </div>
    </div>

    <div class="stats-container">
        <div class="stat-card">
            <div class="stat-label">总用例数</div>
            <div class="stat-value total">)"
        << totalAllCases << R"(</div>
        </div>
        <div class="stat-card">
            <div class="stat-label">通过</div>
            <div class="stat-value passed">)"
        << totalPassed << R"(</div>
        </div>
        <div class="stat-card">
            <div class="stat-label">失败</div>
            <div class="stat-value failed">)"
        << totalFailed << R"(</div>
        </div>
        <div class="stat-card">
            <div class="stat-label">跳过</div>
            <div class="stat-value skipped">)"
        << totalSkipped << R"(</div>
        </div>
    </div>

    <table class="module-stats-table">
        <thead>
            <tr>
                <th>模块</th>
                <th>总用例数</th>
                <th>通过</th>
                <th>失败</th>
                <th>跳过</th>
            </tr>
        </thead>
        <tbody>
    )";

    // 生成模块统计表格行
    for (const auto &[moduleName, _] : moduleTotal)
    {
        int total = moduleTotal[moduleName];
        int passed = modulePassed[moduleName];
        int failed = moduleFailed[moduleName];
        int skipped = moduleSkipped[moduleName];
        ofs << R"(
            <tr>
                <td>)"
            << moduleName << R"(</td>
                <td>)"
            << total << R"(</td>
                <td class="passed">)"
            << passed << R"(</td>
                <td class="failed">)"
            << failed << R"(</td>
                <td class="skipped">)"
            << skipped << R"(</td>
            </tr>
        )";
    }

    ofs << R"(
        </tbody>
    </table>

    <table class="cases-table">
        <thead>
            <tr>
                <th>模块</th>
                <th>测试用例名称</th>
                <th>执行结果</th>
                <th>执行时间(ms)</th>
                <th>详细信息</th>
            </tr>
        </thead>
        <tbody>
    )";

    // 生成用例详情表格
    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx)
    {
        const TestSuite *suite = unit_test->GetTestSuite(suite_idx);
        std::string moduleName = suite->name();
        bool isTargetSuite = false;

        if (!config.case_type.empty())
        {
            isTargetSuite = (moduleName == config.case_type);
        }
        else
        {
            isTargetSuite = (moduleName.substr(0, 4) == "auto");
        }

        if (!isTargetSuite || suite->total_test_count() == 0)
        {
            continue;
        }

        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const TestInfo *test_case = suite->GetTestInfo(case_idx);
            const TestResult *case_result = test_case->result();
            std::string caseName = test_case->name();
            std::string statusText, statusClass;
            std::string detail = "-";
            double elapsed = case_result->elapsed_time() / 1000.0;

            if (case_result->Passed())
            {
                statusText = "通过";
                statusClass = "status-pass";
            }
            else if (case_result->Failed())
            {
                statusText = "失败";
                statusClass = "status-fail";
                std::string failMsg;
                for (int i = 0; i < case_result->total_part_count(); ++i)
                {
                    const TestPartResult &part = case_result->GetTestPartResult(i);
                    if (part.failed())
                    {
                        failMsg += "行号：" + std::to_string(part.line_number()) + " | 错误信息：";
                        std::string raw_msg = part.message();
                        for (char &c : raw_msg)
                        {
                            if (c == '\n' || c == '\r')
                                c = ' ';
                        }
                        std::string compressed_msg;
                        bool prev_space = false;
                        for (char c : raw_msg)
                        {
                            if (c == ' ')
                            {
                                if (!prev_space)
                                {
                                    compressed_msg += c;
                                    prev_space = true;
                                }
                            }
                            else
                            {
                                compressed_msg += c;
                                prev_space = false;
                            }
                        }
                        if (!compressed_msg.empty() && compressed_msg.front() == ' ')
                            compressed_msg.erase(0, 1);
                        if (!compressed_msg.empty() && compressed_msg.back() == ' ')
                            compressed_msg.pop_back();
                        failMsg += compressed_msg + "<br>";
                    }
                }
                if (!failMsg.empty() && failMsg.substr(failMsg.size() - 4) == "<br>")
                    failMsg = failMsg.substr(0, failMsg.size() - 4);
                detail = R"(<div class="fail-details">)" + failMsg + R"(</div>)";
            }
            else if (case_result->Skipped())
            {
                statusText = "跳过";
                statusClass = "status-skip";
                detail = "该用例未执行（跳过原因：按执行范围过滤或主动跳过）";
            }

            ofs << R"(
                <tr>
                    <td>)"
                << moduleName << R"(</td>
                    <td>)"
                << caseName << R"(</td>
                    <td><span class="status-tag )"
                << statusClass << R"(">)" << statusText << R"(</span></td>
                    <td>)"
                << std::fixed << std::setprecision(2) << elapsed << R"(</td>
                    <td>)"
                << detail << R"(</td>
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