#include "xmlHandler.h"
#include <cmath>

constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double degrees) {
    return degrees * PI / 180.0;
}

XML_HANDLER::XML_HANDLER(const fs::path & xml_path)
{
    //判断xml文件是否存在
    if(!fs::exists(xml_path))
    {
        throw std::runtime_error("XML file not found: " + xml_path.string());
    }
    // 加载 XML 文档
    pugi::xml_parse_result result = xml_doc_.load_file(xml_path.c_str());

    if (!result) {
        throw std::runtime_error("XML parse error: " + std::string(result.description()));
    }
}
std::map<std::string, std::vector<float>> XML_HANDLER::get_parameters(const std::string test_case_name , const std::string robot_type)
{
    std::map<std::string,std::vector<float>> xml_parse_result;
    if(robot_type.empty() || test_case_name.empty())
    {
        if(robot_type.empty())
            throw std::runtime_error("robot type is wrong, please use the right robot type parameter try again.");
        else
            throw std::runtime_error("test name is wrong, please use the right robot type parameter try again.");
    }
    std::string node_name = "//"+test_case_name;
    
    //搜索测试机器人类型对应的参数节点
    auto find_node_result = xml_doc_.select_nodes(node_name.c_str());
    if(find_node_result.empty())
    {
        throw std::runtime_error(test_case_name+"node not found, please make sure the node exists and try again.");
    }
    else
    {
        //读取测试所用参数
        for(auto &xnode: find_node_result)
        {
            pugi::xml_node node = xnode.node();
            pugi::xml_node child = node.first_child();
            while (robot_type.compare(child.name()) != 0)
            {
                child = child.next_sibling();
            }
            if(child == nullptr)
            {
                throw std::runtime_error(robot_type+"node not found, please make sure the node exists and try again.");
            }
            for(auto rnode:child)
            {
                std::vector<float> node_value;
                std::istringstream iss(rnode.text().as_string());
                std::string data;
                while(std::getline(iss,data,','))
                {
                    //判断传入的值是否是角度
                    if(std::string attr = rnode.attribute("unit").as_string(); attr.compare("degree") == 0)
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