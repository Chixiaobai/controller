#include "case_api.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <iomanip>
#include <string>

using namespace testing;

bool parse_test_arguments(int argc, char **argv, TestTaskConfig &config)
{
    config.report_path = "";

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-report" && i + 1 < argc)
        {
            config.report_path = argv[++i];
        }
        else
        {
            continue;
        }
    }
    return true;
}

bool save_html_report(const std::string &report_path, const TestTaskConfig &config)
{
    const ::testing::UnitTest *unit_test = ::testing::UnitTest::GetInstance();
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

    // 时间格式化（兼容旧GCC版本）
    std::time_t now = std::time(nullptr);
    std::tm tm = *std::localtime(&now);
    char time_buf[64];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm);
    std::string current_time = time_buf;

    // 统计变量 + 存储筛选后的用例
    std::map<std::string, int> moduleTotal;
    std::map<std::string, int> modulePassed;
    std::map<std::string, int> moduleFailed;
    std::vector<std::pair<const ::testing::TestSuite *, const ::testing::TestInfo *>> filteredTests;
    int totalAllCases = 0, totalPassed = 0, totalFailed = 0;

    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx)
    {
        const ::testing::TestSuite *suite = unit_test->GetTestSuite(suite_idx);
        std::string suite_name = suite->name();
        int suite_executed_count = 0;

        // 统计当前套件中「实际执行的用例数」：名称非空 + （有片段 或 有执行时间）
        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const ::testing::TestInfo *test_case = suite->GetTestInfo(case_idx);
            const ::testing::TestResult *case_result = test_case->result();

            // 关键修改：同时覆盖「有断言（片段数>0）」和「无断言（执行时间>0）」的执行用例
            bool isExecuted = (test_case->name() != nullptr && *test_case->name() != '\0') &&
                              (case_result->total_part_count() > 0 || case_result->elapsed_time() > 0);

            if (isExecuted)
            {
                suite_executed_count++;
            }
        }

        // 跳过无实际执行用例的套件
        if (suite_executed_count == 0)
            continue;

        // 第二步：筛选当前套件中实际执行的用例
        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const ::testing::TestInfo *test_case = suite->GetTestInfo(case_idx);
            const ::testing::TestResult *case_result = test_case->result();
            std::string case_name = test_case->name();

            // 同样的筛选条件，确保一致性
            bool isExecuted = (test_case->name() != nullptr && *test_case->name() != '\0') &&
                              (case_result->total_part_count() > 0 || case_result->elapsed_time() > 0);

            if (!isExecuted)
            {
                continue;
            }

            // 记录并统计
            filteredTests.emplace_back(suite, test_case);
            totalAllCases++;
            moduleTotal[suite_name]++;

            if (case_result->Passed())
            {
                totalPassed++;
                modulePassed[suite_name]++;
            }
            else
            {
                totalFailed++;
                moduleFailed[suite_name]++;
            }
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
        .report-header {
            color: #333;
            padding: 15px;
            margin-bottom: 20px;
            text-align: center;
        }
        .report-title {
            font-size: 24px;
            font-weight: bold;
            margin: 0;
            border-bottom: 2px solid #ddd;
            padding-bottom: 10px;
        }
        .report-meta {
            margin-top: 10px;
            color: #666;
            font-size: 12px;
            line-height: 1.5;
        }
        .module-stats-table {
            width: 100%;
            border-collapse: collapse;
            background-color: white;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            margin-bottom: 30px;
        }
        .module-stats-table th, .module-stats-table td {
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }
        .module-stats-table th {
            background-color: #f8f8f8;
            font-weight: bold;
            color: #555;
        }
        .module-stats-table .total-row {
            background-color: #f0f7ff;
            font-weight: bold;
            color: #333;
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
            padding: 12px;
            text-align: left;
        }
        .cases-table th {
            background-color: #f8f8f8;
            font-weight: bold;
            color: #555;
        }
        .cases-table tr:nth-child(even) {
            background-color: #f9f9f9;
        }
        .cases-table tr:hover {
            background-color: #f5f5f5;
        }
        .status-tag {
            display: inline-block;
            padding: 4px 8px;
            border-radius: 4px;
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
            font-size: 12px;
            white-space: pre-wrap;
            line-height: 1.5;
        }
        .empty-tip {
            text-align: center;
            padding: 30px;
            color: #666;
            font-style: italic;
        }
    </style>
</head>
<body>
    <div class="report-header">
        <h1 class="report-title">测试报告</h1>
        <div class="report-meta">
            执行时间：)"
        << current_time << R"(<br>
            实际执行总用例数：)"
        << totalAllCases << R"( | 通过：)" << totalPassed << R"( | 失败：)" << totalFailed << R"(<br>
            报告说明：仅显示 --gtest_filter 筛选的实际执行用例
        </div>
    </div>

    <!-- 模块统计表格 -->
    <table class="module-stats-table">
        <thead>
            <tr>
                <th>测试模块（套件）</th>
                <th>实际执行用例数</th>
                <th>通过数</th>
                <th>失败数</th>
            </tr>
        </thead>
        <tbody>
    )";

    if (moduleTotal.empty())
    {
        ofs << R"(
            <tr>
                <td colspan="4" class="empty-tip">无实际执行的测试用例</td>
            </tr>
        )";
    }
    else
    {
        for (const auto &[suite_name, total] : moduleTotal)
        {
            int passed = modulePassed[suite_name];
            int failed = moduleFailed[suite_name];

            ofs << R"(
            <tr>
                <td>)"
                << suite_name << R"(</td>
                <td>)"
                << total << R"(</td>
                <td>)"
                << passed << R"(</td>
                <td>)"
                << failed << R"(</td>
            </tr>
        )";
        }
        ofs << R"(
            <tr class="total-row">
                <td>总计</td>
                <td>)"
            << totalAllCases << R"(</td>
            <td>)"
            << totalPassed << R"(</td>
            <td>)"
            << totalFailed << R"(</td>
            </tr>
        )";
    }

    ofs << R"(
        </tbody>
    </table>

    <!-- 用例详情表格 -->
    <table class="cases-table">
        <thead>
            <tr>
                <th>测试模块（套件）</th>
                <th>测试用例名称</th>
                <th>执行结果</th>
                <th>执行时间(ms)</th>
                <th>详细信息</th>
            </tr>
        </thead>
        <tbody>
    )";

    if (filteredTests.empty())
    {
        ofs << R"(
            <tr>
                <td colspan="5" class="empty-tip">无实际执行的测试用例</td>
            </tr>
        )";
    }
    else
    {
        for (const auto &[suite, test_case] : filteredTests)
        {
            const ::testing::TestResult *case_result = test_case->result();
            std::string suite_name = suite->name();
            std::string case_name = test_case->name();
            std::string statusText = case_result->Passed() ? "通过" : "失败";
            std::string statusClass = case_result->Passed() ? "status-pass" : "status-fail";
            std::string detail = "无";
            double elapsed = case_result->elapsed_time();

            // 失败用例添加详细信息
            if (case_result->Failed())
            {
                std::string failMsg;
                for (int i = 0; i < case_result->total_part_count(); ++i)
                {
                    const ::testing::TestPartResult &part = case_result->GetTestPartResult(i);
                    if (part.failed())
                    {
                        failMsg += "行号" + std::to_string(part.line_number()) + "：";
                        std::string raw_msg = part.message();
                        std::replace(raw_msg.begin(), raw_msg.end(), '\n', ' ');
                        std::replace(raw_msg.begin(), raw_msg.end(), '\r', ' ');
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

            ofs << R"(
                <tr>
                    <td>)"
                << suite_name << R"(</td>
                    <td>)"
                << case_name << R"(</td>
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
    std::cout << "[报告统计] 实际执行用例：总" << totalAllCases << "个，通过" << totalPassed << "个，失败" << totalFailed << "个" << std::endl;
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
