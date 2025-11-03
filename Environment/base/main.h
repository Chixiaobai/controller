#pragma once

#ifdef _WIN32
#include "windows.h"
#else
#include "time.h"
#endif
#include <rclcpp/rclcpp.hpp>
#include "string.h"
#include "float.h"
#include "math.h"
#include "iostream"
#include "fstream"
#include "case_api.h"
#include <vector>
#include "xmlHandler.h"
using namespace std;

#define Get_robot_config_file_path(robot_point_file)\
    int pos1=std::string(__FILE__).rfind("/");\
    int pos2=std::string(__FILE__).rfind(".");\
    robot_point_file=std::string(__FILE__).substr(0,pos1+1)+"config/"+"config.xml";\
    std::cout << "robot_config_file:" << robot_point_file.c_str() << std::endl; \

// 角度转换：弧度-->度
float to_degree(float x);

float to_rad(float x);

// 线程睡眠
void sleepMilliseconds(unsigned int uintMilliseconds);

// 等待用户输入一个字符
char read_input(const char* strPrompt);

//读取数据
vector<vector<double>> read_data(string file_name);

//浮点数是否相等
bool isFloatValueEqual(float fltValueLeft, float fltValueRight, float fltEpsilon = 0.01f);

// 时间转换函数，将 index 转换为 mm:ss:zzz 格式的时间
std::string indexToTime(int ms);

//缩放正弦函数
float Sin(float index,float lowerlimit,float upperlimit,float samplecount);

// 判断输入值是否在指定的上下限范围内
bool isWithinRange(double inputValue, double lowerBound, double upperBound);

// std::string indexToTime(int index);

// 时间工具函数
void sleep_ms(unsigned int ms);  // 毫秒级睡眠

// 机器人类型相关工具
bool is_robot_type(const std::string& type);  // 判断当前机器人类型是否匹配

// 日志输出（封装打印，附加时间和用例信息）
void test_log(const std::string& case_id, const std::string& message);
void test_error(const std::string& case_id, const std::string& error);
