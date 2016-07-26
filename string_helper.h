#pragma once
#include <string>
namespace cloud_icp_reg {
class StringHelper {
public:
    StringHelper();
    ~StringHelper();
    static std::vector< std::string> split(std::string& str, const std::string find);
    static string get_time_string();
};
} // cloud_icp_reg