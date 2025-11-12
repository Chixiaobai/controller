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

std::map<std::string, std::vector<double>> XML_HANDLER::get_parameters() {
    std::map<std::string, std::vector<double>> xml_parse_result;

    const ::testing::UnitTest* unit_test = ::testing::UnitTest::GetInstance();
    const ::testing::TestInfo* current_test_info = unit_test->current_test_info();
    if (!current_test_info) {
        throw std::runtime_error("Failed to get current test info (check --gtest_filter)");
    }
    std::string test_case_name = current_test_info->name();
    std::string node_name = "//" + test_case_name;

    pugi::xpath_node_set find_node_result = xml_doc_.select_nodes(node_name.c_str());
    if (find_node_result.empty()) {
        throw std::runtime_error(test_case_name + " 节点未找到，请确保节点存在后重试。");
    }

    for (auto& xnode : find_node_result) {
        pugi::xml_node test_case_node = xnode.node();
        pugi::xml_node param_node = test_case_node.first_child();
        
        while (param_node) {
            if (param_node.type() == pugi::node_comment || 
                (param_node.type() == pugi::node_pcdata && param_node.text().as_string()[0] == '\0')) {
                param_node = param_node.next_sibling();
                continue;
            }

            std::vector<double> node_value;
            std::istringstream iss(param_node.text().as_string());
            std::string data;

            while (std::getline(iss, data, ',')) {
                data.erase(data.begin(), std::find_if(data.begin(), data.end(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }));
                data.erase(std::find_if(data.rbegin(), data.rend(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }).base(), data.end());

                if (data.empty()) continue; 

                // 角度转弧度
                if (std::string attr = param_node.attribute("unit").as_string(); attr == "degree") {
                    node_value.push_back(deg2rad(std::stod(data)));
                } else {
                    node_value.push_back(static_cast<double>(std::stod(data)));
                }
            }

            if (!node_value.empty()) {
                xml_parse_result[param_node.name()] = node_value;
            }

            param_node = param_node.next_sibling();
        }
    }

    // 校验是否解析到参数
    if (xml_parse_result.empty()) {
        throw std::runtime_error(test_case_name + " 节点下未找到任何有效参数。");
    }

    return xml_parse_result;
}
