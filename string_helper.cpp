#include "stdafx.h"
#include "string_helper.h"

namespace cloud_icp_reg {
StringHelper::StringHelper(){
}


StringHelper::~StringHelper(){
}

string StringHelper::get_time_string(){
    time_t t;  //��ʱ��  
    tm* local; //����ʱ��   
    tm* gmt;   //��������ʱ��  
    char buf[128] = {
        0
    };

    t = time(NULL); //��ȡĿǰ��ʱ��  
    local = localtime(&t); //תΪ����ʱ��  
    strftime(buf, 64, "%Y%m%d%H%M%S", local);
    string strTime = string(buf);
    return strTime;
    //std::cout << buf << std::endl;

    //gmt = gmtime(&t);//תΪ��������ʱ��  
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