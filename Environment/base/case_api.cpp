#include "case_api.h"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <iomanip>
#include <string>

using namespace testing;
bool parse_test_arguments(int argc, char **argv, TestTaskConfig &config)
{
    std::cout << "解析-case参数：" << std::endl;
    config.case_type = "";
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-case" && i + 1 < argc)
        {
            std::string case_input = argv[++i];
            config.case_type = case_input;
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
    std::map<std::string, std::vector<std::string>> moduleFailDetails; // 模块→失败详情
    int totalAllCases = 0, totalPassed = 0, totalFailed = 0;

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

            if (case_result->Passed())
            {
                modulePassed[moduleName]++;
                totalPassed++;
            }
            else if (case_result->Failed())
            {
                moduleFailed[moduleName]++;
                totalFailed++;
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
                        {
                            compressed_msg.erase(0, 1);
                        }
                        if (!compressed_msg.empty() && compressed_msg.back() == ' ')
                        {
                            compressed_msg.pop_back();
                        }
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
    <title>测试报告</title>
    <style type="text/css">
        body {
            font-family: "Microsoft YaHei", Arial, sans-serif;
            font-size: 14px;
            margin: 0;
            padding: 20px;
            background-color: #f9f9f9;
        }
        .report-header {
            /* 移除蓝色背景 */
            color: #333; /* 文字改为深灰色 */
            padding: 15px;
            margin-bottom: 20px;
            text-align: center;
        }
        .report-title {
            font-size: 24px;
            font-weight: bold;
            margin: 0;
            /* 可添加标题下划线增强视觉效果 */
            border-bottom: 2px solid #ddd;
            padding-bottom: 10px;
        }
        .report-meta {
            margin-top: 10px;
            color: #666; /* 元信息文字颜色 */
            font-size: 12px;
        }
        /* 其他样式保持不变 */
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
        .module-stats-table .total-row {
            background-color: #f0f7ff;
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
        .fail-details {
            margin-top: 8px;
            padding: 10px;
            background-color: #f2dede;
            border-radius: 4px;
            color: #a94442;
            white-space: pre-wrap;
        }
    </style>
</head>
<body>
    <div class="report-header">
        <h1 class="report-title">测试报告</h1>
        <div class="report-meta">
            执行时间：)"
        << current_time << R"(<br>
            执行范围：)"
        << (config.case_type.empty() ? "自动化测试用例" : config.case_type) << R"(
        </div>
    </div>

    <table class="module-stats-table">
        <thead>
            <tr>
                <th>模块</th>
                <th>总用例数</th>
                <th>通过</th>
                <th>失败</th>
            </tr>
        </thead>
        <tbody>
    )";

    // 统计指定模块的总计数据
    int targetTotal = 0, targetPassed = 0, targetFailed = 0;

    // 生成模块统计表格行（仅保留指定模块）
    for (const auto &[moduleName, _] : moduleTotal)
    {
        // 判断是否为指定模块
        bool isTargetModule = false;
        if (!config.case_type.empty())
        {
            isTargetModule = (moduleName == config.case_type);
        }
        else
        {
            isTargetModule = (moduleName.substr(0, 4) == "auto");
        }

        // 只显示指定模块
        if (!isTargetModule)
        {
            continue;
        }

        int total = moduleTotal[moduleName];
        int passed = modulePassed[moduleName];
        int failed = moduleFailed[moduleName];

        // 累加指定模块的总计
        targetTotal += total;
        targetPassed += passed;
        targetFailed += failed;

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
            </tr>
        )";
    }

    // 添加指定模块的总计行
    ofs << R"(
            <tr class="total-row">
                <td>总计</td>
                <td>)"
        << targetTotal << R"(</td>
                <td class="passed">)"
        << targetPassed << R"(</td>
                <td class="failed">)"
        << targetFailed << R"(</td>
            </tr>
        </tbody>
    </table>

    <table class="cases-table">
        <thead>
            <tr>
                <th>模块</th>
                <th>测试用例</th>
                <th>执行结果</th>
                <th>执行时间(ms)</th>
                <th>详细信息</th>
            </tr>
        </thead>
        <tbody>
    )";

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

            if (!case_result->Failed())
            {
                continue;
            }
            std::string caseName = test_case->name();
            std::string statusText = "失败";
            std::string statusClass = "status-fail";
            std::string detail;
            double elapsed = case_result->elapsed_time() / 1000.0;

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
            {
                failMsg = failMsg.substr(0, failMsg.size() - 4);
            }
            detail = R"(<div class="fail-details">)" + failMsg + R"(</div>)";

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