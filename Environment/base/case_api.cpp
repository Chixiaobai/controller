#include "case_api.h"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <iomanip>
#include <string>

// 在parse_test_arguments函数中，修改过滤规则生成逻辑
bool parse_test_arguments(int argc, char**argv, TestTaskConfig& config) {
    std::cout << "解析-case参数："  << std::endl;
    // 初始化默认过滤规则（执行所有用例，不提前过滤非auto组）
    config.case_filter = "*";  // 默认执行所有用例

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-case" && i + 1 < argc) {
            std::string case_input = argv[++i];
            config.case_type = case_input;
            
            // 仅当指定了-case参数时才修改过滤规则
            if (case_input.find('.') != std::string::npos) {
                config.case_filter = case_input;  // 单个用例
            } else {
                config.case_filter = case_input + ".*";  // 批次用例
            }
        } else if (arg == "-robot" && i + 1 < argc) {
            config.robot_type = argv[++i];
        } else if (arg == "-report" && i + 1 < argc) {
            config.report_path = argv[++i];
        }
    }

    if (config.robot_type.empty()) {
        std::cerr << "错误：必须使用-robot指定机器人类型（如-robot H10-W）" << std::endl;
        return false;
    }

    return true;
}


bool save_html_report(const std::string &report_path, const TestTaskConfig &config)
{
    // 校验报告路径
    if (report_path.empty())
    {
        std::cerr << "警告：HTML报告路径为空，无法生成" << std::endl;
        return false;
    }

    // 打开文件（若不存在则创建，存在则覆盖）
    std::ofstream ofs(report_path);
    if (!ofs.is_open())
    {
        std::cerr << "错误：无法打开HTML报告文件（路径：" << report_path << "）" << std::endl;
        return false;
    }

    // 统计运行时跳过的用例（GTEST_SKIP()）和编译时禁用的用例（TEST_DISABLED_）
    const testing::UnitTest *unit_test = testing::UnitTest::GetInstance();
    int runtime_skipped = 0; // 运行时跳过（新增统计）

    // 遍历所有用例计算统计值
    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx)
    {
        const testing::TestSuite *suite = unit_test->GetTestSuite(suite_idx);
        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const testing::TestInfo *test_case = suite->GetTestInfo(case_idx);
            const testing::TestResult *case_result = test_case->result();

            if (case_result->Skipped())
            {
                runtime_skipped++; // 运行时跳过
            }
        }
    }

    // 1. HTML头部（引入样式，确保美观且响应式）
    ofs << R"(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>测试报告（HTML版）</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: "Microsoft YaHei", Arial, sans-serif; background-color: #f5f5f5; padding: 20px; }
        .report-wrapper { max-width: 1200px; margin: 0 auto; background: white; border-radius: 8px; box-shadow: 0 2px 12px rgba(0,0,0,0.08); overflow: hidden; }
        .report-header { background-color: #2c3e50; color: white; padding: 30px; text-align: center; }
        .header-title { font-size: 24px; margin-bottom: 15px; }
        .header-info { font-size: 14px; line-height: 1.6; opacity: 0.9; }
        .stats-container { display: flex; justify-content: center; gap: 25px; padding: 30px; flex-wrap: wrap; background-color: #fafafa; border-bottom: 1px solid #eee; }
        .stat-card { background-color: white; padding: 20px 30px; border-radius: 6px; box-shadow: 0 1px 4px rgba(0,0,0,0.05); min-width: 140px; text-align: center; }
        .stat-label { font-size: 14px; color: #666; margin-bottom: 8px; }
        .stat-value { font-size: 28px; font-weight: bold; }
        .stat-total { color: #3498db; }
        .stat-passed { color: #2ecc71; }
        .stat-failed { color: #e74c3c; }
        .stat-skipped { color: #f39c12; }
        .stat-disabled { color: #9b59b6; } /* 新增：编译时禁用的颜色 */
        .suites-container { padding: 30px; }
        .suite-card { margin-bottom: 25px; border: 1px solid #eee; border-radius: 6px; overflow: hidden; }
        .suite-header { background-color: #f8f8f8; padding: 15px 20px; font-size: 16px; font-weight: bold; color: #333; border-bottom: 1px solid #eee; }
        .case-list { padding: 10px 0; }
        .case-item { padding: 12px 20px; border-bottom: 1px solid #f5f5f5; display: flex; align-items: center; justify-content: space-between; }
        .case-item:last-child { border-bottom: none; }
        .case-pass { color: #27ae60; }
        .case-fail { color: #c0392b; }
        .case-skip { color: #f39c12; } /* 新增：运行时跳过的样式 */
        .case-disable { color: #9b59b6; } /* 新增：编译时禁用的样式 */
        .case-icon { font-size: 18px; margin-right: 12px; }
        .case-name { font-size: 14px; flex: 1; }
        .case-time { font-size: 13px; color: #999; }
        .fail-detail { padding: 12px 20px; background-color: #fdf2f2; border-left: 4px solid #e74c3c; margin: 0 20px 15px; border-radius: 4px; font-size: 14px; color: #c0392b; }
        .skip-detail { padding: 12px 20px; background-color: #fef5e7; border-left: 4px solid #f39c12; margin: 0 20px 15px; border-radius: 4px; font-size: 14px; color: #d35400; } /* 新增：跳过详情样式 */
        .report-footer { background-color: #fafafa; padding: 15px 20px; text-align: center; font-size: 13px; color: #999; border-top: 1px solid #eee; }
    </style>
</head>
<body>
    <div class="report-wrapper">
        <!-- 报告头部 -->
        <div class="report-header">
            <div class="header-title">测试报告</div>
            <div class="header-info">测试时间：)"
        << get_current_time_str() << R"(</div>
            <div class="header-info">臂型：)"
        << config.robot_type << R"(</div>
            <div class="header-info">执行范围：)"
        << (config.case_type.empty() ? "所有auto开头的测试组" : config.case_type) << R"(</div>
        </div>

        <!-- 统计卡片区 -->
        <div class="stats-container">
            <div class="stat-card">
                <div class="stat-label">总用例数</div>
                <div class="stat-value stat-total">)"
        << unit_test->total_test_count() << R"(</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">通过</div>
                <div class="stat-value stat-passed">)"
        << unit_test->successful_test_count() << R"(</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">失败</div>
                <div class="stat-value stat-failed">)"
        << unit_test->failed_test_count() << R"(</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">运行时跳过</div>
                <div class="stat-value stat-skipped">)"
        << runtime_skipped << R"(</div>
            </div>
        </div>

        <!-- 测试组详情区 -->
        <div class="suites-container">
    )";

    // 2. 遍历所有测试组
    for (int suite_idx = 0; suite_idx < unit_test->total_test_suite_count(); ++suite_idx)
    {
        const testing::TestSuite *suite = unit_test->GetTestSuite(suite_idx);
        if (suite->total_test_count() == 0)
            continue;

        // 计算当前组内的统计细分
        int suite_passed = 0, suite_failed = 0, suite_skipped = 0;
        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const testing::TestInfo *test_case = suite->GetTestInfo(case_idx);
            const testing::TestResult *case_result = test_case->result();

            if (case_result->Passed())
                suite_passed++;
            else if (case_result->Failed())
                suite_failed++;
            else if (case_result->Skipped())
                suite_skipped++;
        }

        ofs << R"(
            <div class="suite-card">
                <div class="suite-header">
                    测试模块：)"
            << suite->name() << R"( 
                    （通过：)"
            << suite_passed
            << R"( | 失败：)" << suite_failed
            << R"( | 跳过：)" << suite_skipped
            << R"( | 总计：)" << suite->total_test_count() << R"(）
                </div>
                <div class="case-list">
        )";

        // 遍历组内每个用例
        for (int case_idx = 0; case_idx < suite->total_test_count(); ++case_idx)
        {
            const testing::TestInfo *test_case = suite->GetTestInfo(case_idx);
            const testing::TestResult *case_result = test_case->result();
            std::string case_status, case_icon;

            // 判断用例状态并设置样式
            if (case_result->Passed())
            {
                case_status = "case-pass";
                case_icon = "✅";
            }
            else if (case_result->Failed())
            {
                case_status = "case-fail";
                case_icon = "❌";
            }
            else if (case_result->Skipped()) // 新增：处理跳过状态
            {
                case_status = "case-skip";
                case_icon = "⏭️";
            }

            // 处理时间格式化
            std::ostringstream time_stream;
            time_stream << std::fixed << std::setprecision(3)
                        << case_result->elapsed_time() / 1000.0;
            std::string time_str = time_stream.str();

            // 写入用例基本信息
            ofs << R"(
            <div class="case-item )"
                << case_status << R"(">
                <div class="case-icon">)"
                << case_icon << R"(</div>
                <div class="case-name">)"
                << test_case->name() << R"(</div>
                <div class="case-time">耗时：)"
                << time_str << R"(秒</div>
            </div>
    )";

            // 写入失败详情
            if (case_result->Failed() && case_result->total_part_count() > 0)
            {
                const testing::TestPartResult &fail_part = case_result->GetTestPartResult(0);
                ofs << R"(
            <div class="fail-detail">
                失败原因：)"
                    << fail_part.message() << R"(
            </div>
        )";
            }
        }

        ofs << R"(
                </div>
            </div>
        )";
    }

    // 3. HTML尾部
    ofs << R"(
        </div>
        <div class="report-footer">
            报告生成时间：)"
        << get_current_time_str() << R"( | 生成工具：GTest HTML Reporter
        </div>
    </div>
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