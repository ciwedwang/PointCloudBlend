#include "stdafx.h"
#include "string_helper.h"

namespace cloud_icp_reg {
StringHelper::StringHelper(){
}


StringHelper::~StringHelper(){
}

string StringHelper::get_time_string(){
    time_t t;  //秒时间  
    tm* local; //本地时间   
    tm* gmt;   //格林威治时间  
    char buf[128] = {
        0
    };

    t = time(NULL); //获取目前秒时间  
    local = localtime(&t); //转为本地时间  
    strftime(buf, 64, "%Y%m%d%H%M%S", local);
    string strTime = string(buf);
    return strTime;
    //std::cout << buf << std::endl;

    //gmt = gmtime(&t);//转为格林威治时间  
    //strftime(buf, 64, "%Y-%m-%d %H:%M:%S", gmt);
    //std::cout << buf << std::endl;
}

std::vector< std::string> StringHelper::split(std::string& str, const std::string find){
    std::vector< std::string> ret;
    int start = 0;
    int pos = str.find(find, start);
    int len = find.length();
    int i = 0;
    while (true) {
        if (pos < 0) {
            ret.push_back(str.substr(start));
            break;
        } else {
            ret.push_back(str.substr(start, pos - start));
            start = pos + len;
            pos = str.find(find, start);
        }
        i++;
    }
    return ret;
}
} // cloud_icp_reg