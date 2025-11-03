
#pragma once

#include "AmwToolDef.h"

/*命令行解析类
    支持的选项类型：
    短选项
        无参数的选项：-a -bcd
        带参数的选项：-b value
        可选参数：-c[value]
    长选项
        无参数的选项：--help
        带参数的选项：--file value --file=value
        可选参数：--verbose[=value]
*/
class AMWTOOL_CLASS Option {
public:
    enum class OptionType : unsigned char {
        UNKNOWN,
        // 无参数的选项
        NO_OPTION,
        // 带参数的选项
        REQUIRED_OPTION,
        // 可选参数
        OPTIONAL_OPTION
    };
    struct OptionProperty {
        OptionType m_optionType;
        std::string m_optionName;
        std::string m_optionValue;
        std::string m_description;
    };

    Option() = default;
    ~Option() = default;

    // 注册选项
    void addOption(const std::string& optionName, OptionType optionType = OptionType::NO_OPTION,
                   const std::string& description = "");
    OptionType type(const std::string& optionName) const;
    // 解析命令行参数
    void parse(int argc, char* argv[]);

    bool hasOption(const std::string& optionName) const;

    void printUsage() const;

    bool getOptionValue(const std::string& optionName, std::string& value) const;

    std::string getString(const std::string& optionName) const;

    int getInt(const std::string& optionName) const;

    double getDouble(const std::string& optionName) const;

    bool getBool(const std::string& optionName) const;

private:
    std::map<std::string /*option name*/, OptionProperty> m_optionMap;
};
