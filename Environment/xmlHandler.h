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
#include <gtest/gtest.h>
namespace fs = std::filesystem;
class XML_HANDLER
{
public:
    XML_HANDLER() = delete;
    XML_HANDLER(const fs::path &xml_path);
    ~XML_HANDLER() = default;
    std::map<std::string, std::vector<float>> get_parameters();

private:
    pugi::xml_document xml_doc_;
};
#endif