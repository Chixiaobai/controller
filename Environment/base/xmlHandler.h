#pragma once
#ifndef _XMLHANDELER_H_
#define _XMLHANDELER_H_
#include "pugixml.hpp"
#include <map>
#include <filesystem>
#include <any>
#include <string>
#include <vector>
#include <iostream>
namespace fs = std::filesystem;
class XML_HANDLER
{
    public:
        XML_HANDLER() = delete;
        XML_HANDLER(const fs::path& xml_path);
        ~XML_HANDLER() = default;
        std::map<std::string, std::vector<float>> get_parameters(const std::string test_case_name , const std::string robot_type);
    private:
        pugi::xml_document xml_doc_;
};
#endif