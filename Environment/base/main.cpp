#include "main.h"
#include "case_api.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <ctime>
#include <thread>
#include <unistd.h>
#include <algorithm>
#include <string.h>
#include <filesystem>
#include </usr/include/pugixml.hpp> // 需要已安装 pugixml

using namespace std;
namespace fs = std::filesystem;

// const char *strIpAddress = "192.168.10.75";

float to_degree(float x) { return ((x) * 180.0 / M_PI); }

float to_rad(float x) { return ((x)*M_PI / 180.0); }

// 定义数据结构存储解析结果
struct User
{
    int id;
    std::string name;
    int age;
    std::string role;
};

void sleepMilliseconds(unsigned int uintMilliseconds)
{
#ifdef _WIN32
    Sleep(dwMilliseconds);
#else
    struct timespec ts;
    ts.tv_sec = uintMilliseconds / 1000;
    ts.tv_nsec = (uintMilliseconds % 1000) * 1000000;
    while ((-1 == nanosleep(&ts, &ts)) && (EINTR == errno))
        ;
#endif
}

// const char * get_const_strIpAddress()
// {
//     return strIpAddress;
// }

char read_input(const char *strPrompt)
{
    char input = '\0';
    if (strPrompt != nullptr)
    {
        cout << strPrompt;
    }
    cin >> input;
    return input;
}

vector<vector<double>> read_data(string file_name)
{
    std::istringstream iss;
    string linedata;
    double jointdata;
    vector<double> row(7);
    vector<vector<double>> data;

    ifstream infile;
    infile.open(file_name, ios::in);
    if (!infile)
    {
        cout << "error" << endl;
    }

    while (getline(infile, linedata))
    {
        iss.clear();
        string::iterator it;
        for (it = linedata.begin(); it != linedata.end(); it++)
        {
            if (*it == ';')
            {
                linedata.erase(it);
                break;
            }
        }
        iss.str(linedata);
        int i = 0;
        while (iss >> jointdata)
        {
            if (i > 6)
            {
                break;
            }
            row[i] = jointdata;
            i++;
        }
        data.push_back(row);
    }

    infile.close();
    return data;
}
bool isFloatValueEqual(float fltValueLeft, float fltValueRight, float fltEpsilon)
{
    return (fltEpsilon > fabs(fltValueLeft - fltValueRight));
}

float Sin(float index, float lowerlimit, float upperlimit, float samplecount)
{
    if (samplecount == 0)
    {
        std::cerr << "Error: sampleCount cannot be zero." << std::endl;
        return 0.0f;
    }
    float halfRange = (upperlimit - lowerlimit) / 2;
    float midpoint = (upperlimit + lowerlimit) / 2;
    float angle = 2 * M_PI * index / samplecount;
    return halfRange * std::sin(angle) + midpoint;
}

// 时间转换函数，将 index 转换为 mm:ss:zzz 格式的时间
std::string indexToTime(int ms)
{
    // 处理负数
    bool isNegative = ms < 0;
    ms = isNegative ? -ms : ms;

    // 计算各时间单位
    int minutes = ms / 60000;
    int seconds = (ms % 60000) / 1000;
    int millis = ms % 1000;

    // 格式化输出
    std::ostringstream oss;
    if (isNegative)
        oss << "-";
    oss << std::setfill('0') << std::setw(2) << minutes << ":" << std::setfill('0') << std::setw(2) << seconds << "."
        << std::setfill('0') << std::setw(3) << millis;
    return oss.str();
}

// 时间转换函数，将 index 转换为 hh:mm:ss 格式的时间

// std::string indexToTime(int index) {
//     int hours = index / 3600;
//     int minutes = (index % 3600) / 60;
//     int seconds = index % 60;

//     std::ostringstream oss;
//     oss << std::setfill('0') << std::setw(2) << hours << ":" << std::setfill('0') << std::setw(2) << minutes << ":"
//         << std::setfill('0') << std::setw(2) << seconds;
//     return oss.str();
// }

// 判断输入值是否在指定的上下限范围内
bool isWithinRange(double inputValue, double lowerBound, double upperBound)
{
    return inputValue >= lowerBound && inputValue <= upperBound;
}

// 解析 XML 并返回 User 列表
std::vector<User> parse_users_from_xml(const fs::path &xml_path)
{
    std::vector<User> users;

    // 检查文件是否存在
    if (!fs::exists(xml_path))
    {
        throw std::runtime_error("XML file not found: " + xml_path.string());
    }

    // 加载 XML 文档
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(xml_path.c_str());

    if (!result)
    {
        throw std::runtime_error("XML parse error: " + std::string(result.description()));
    }

    // 遍历 XML 节点
    pugi::xml_node test_data = doc.child("TestData");
    pugi::xml_node users_node = test_data.child("Users");

    for (pugi::xml_node user_node : users_node.children("User"))
    {
        User user;

        // 读取属性
        user.id = user_node.attribute("id").as_int();

        // 读取子节点内容
        user.name = user_node.child("Name").text().as_string();
        user.age = user_node.child("Age").text().as_int();
        user.role = user_node.child("Role").text().as_string();

        users.push_back(user);
    }
    return users;
}

// 毫秒级睡眠（基于std::this_thread）
void sleep_ms(unsigned int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// 判断当前机器人类型是否匹配目标类型
bool is_robot_type(const std::string &type)
{
    return g_test_config.robot_type == type;
}

// 带时间戳的日志输出
void test_log(const std::string &case_id, const std::string &message)
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::cout << "[" << std::put_time(std::localtime(&now_time), "%H:%M:%S")
              << "][" << case_id << "][INFO] " << message << std::endl;
}

// 错误日志输出
void test_error(const std::string &case_id, const std::string &error)
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::cerr << "[" << std::put_time(std::localtime(&now_time), "%H:%M:%S")
              << "][" << case_id << "][ERROR] " << error << std::endl;
}
TestTaskConfig g_test_config;
int main(int argc, char **argv)
{
    if (!parse_test_arguments(argc, argv, g_test_config))
    {
        return 1;
    }

    testing::InitGoogleTest(&argc, argv);
    testing::GTEST_FLAG(filter) = g_test_config.case_filter;

    int test_result = RUN_ALL_TESTS();

    if (!g_test_config.report_path.empty())
    {
        save_html_report(g_test_config.report_path, g_test_config);
    }

    return test_result;
}
