#pragma once
#include "AmwToolDef.h"
#include <string>
#include <map>
#include "Singleton.hpp"
typedef std::map<std::string /*key*/, std::string /*value*/> Section;

class AMWTOOL_CLASS IniFile {
public:
    IniFile() = default;
    ~IniFile() = default;
    bool load(const std::string &filePath);
    bool has(const std::string &section);
    bool has(const std::string &section, const std::string &key);
    std::string getString(const std::string &section, const std::string &key, const std::string &defaultValue = "");
    int getInt(const std::string &section, const std::string &key, int defaultValue = 0);
    double getDouble(const std::string &section, const std::string &key, double defaultValue = 0.0);
    bool getBool(const std::string &section, const std::string &key, bool defaultValue = false);
    Section &operator[](const std::string &section);
    std::string str() const;
    void show();

private:
    std::map<std::string /*section*/, Section> m_iniFile;
};