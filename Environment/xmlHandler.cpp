#include "xmlHandler.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double degrees)
{
    return degrees * PI / 180.0;
}

XML_HANDLER::XML_HANDLER(const fs::path &xml_path)
{
    if (!fs::exists(xml_path))
    {
        throw std::runtime_error("XML file not found: " + xml_path.string());
    }
    pugi::xml_parse_result result = xml_doc_.load_file(xml_path.c_str());

    if (!result)
    {
        throw std::runtime_error("XML parse error: " + std::string(result.description()));
    }
}
std::string XML_HANDLER::get_test_case_name_from_gtest()
{
    ::testing::UnitTest *unit_test = ::testing::UnitTest::GetInstance();
    if (unit_test == nullptr)
    {
        throw std::runtime_error("获取 GTest 全局实例失败，无法定位测试用例！");
    }

    const ::testing::TestInfo *current_test = unit_test->current_test_info();
    if (current_test == nullptr)
    {
        throw std::runtime_error("当前无正在执行的测试用例！");
    }

    std::string test_case_name = current_test->name();
    if (test_case_name.empty())
    {
        throw std::runtime_error("测试用例名为空，请检查测试用例定义！");
    }

    return test_case_name;
}

std::map<std::string, std::vector<float>> XML_HANDLER::get_parameters()
{
    std::map<std::string, std::vector<float>> xml_parse_result;
    std::string test_case_name = get_test_case_name_from_gtest();

    if (test_case_name.empty())
    {
        throw std::runtime_error("请检查配置或测试用例！");
    }
    std::string node_name = "//" + test_case_name;

    auto find_node_result = xml_doc_.select_nodes(node_name.c_str());
    if (find_node_result.empty())
    {
        throw std::runtime_error(test_case_name + "node not found, please make sure the node exists and try again.");
    }
    else
    {
        for (auto &xnode : find_node_result)
        {
            pugi::xml_node node = xnode.node();
            pugi::xml_node child = node.first_child();
            for (auto rnode : child)
            {
                std::vector<float> node_value;
                std::istringstream iss(rnode.text().as_string());
                std::string data;
                while (std::getline(iss, data, ','))
                {
                    if (std::string attr = rnode.attribute("unit").as_string(); attr.compare("degree") == 0)
                    {
                        node_value.push_back(deg2rad(std::stod(data)));
                    }
                    else
                    {
                        node_value.push_back(std::stod(data));
                    }
                }
                xml_parse_result[rnode.name()] = node_value;
            }
        }
    }
    return xml_parse_result;
}