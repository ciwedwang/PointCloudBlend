/*
*   功能描述: 点云密度统计,采用投影统计,过滤运动的物体和其他杂点,
*            生成纯净的点云
*/
#pragma once
#include <boost/unordered_map.hpp>
#include <list>
#include <Eigen/Eigen>
#include "cmm_types.h"

namespace cloud_icp_reg {
const double MAX_DIS_2POINT_X = 0.05; //定义网格宽度
const double MAX_DIS_2POINT = MAX_DIS_2POINT_X* MAX_DIS_2POINT_X; //小于0.01米距离的合成为1个点
typedef uint64_t MapKeyType;

/*
*   功能描述: 网格统计子单元             
*/
class SubDensityItem {
public:
    SubDensityItem(CloudItem item)
            : hit_num(1)
    {
        data = item;
    }    
    /*
    *  函数名称: hit
    *  功能描述: 命中,增加一次命中计数
    */
    inline void hit(){
        hit_num++;
    }    
    /*
    *  函数名称: get_data
    *  返回值: CloudItem 
    *  功能描述: 获取子单元内的点云节点数据
    */
    CloudItem get_data(){
        return data;
    }    
    /*
    *  函数名称: get_hit_num
    *  返回值: uint64_t
    *  功能描述: 
    */
    uint64_t get_hit_num(){
        return hit_num;
    }
private:
    CloudItem data; //点云节点
    uint64_t hit_num; //命中数
};

/*
*   功能描述: 网格统计单元             
*/
class CloudDensityItem {
public:
    std::vector< SubDensityItem> node_points; //子单元列表
    int x;
    int y;
    CloudDensityItem()
            : x(0), y(0)
    {
    }    
    /*
    *  函数名称: Add_points 
    *  参数说明: item_src 增加一个点云节点到网格中
    *  返回值: bool 是否成功
    *  功能描述: 
    */
    bool add_point(CloudItem item_src){
        bool is_hitted = false;
        for (int k = 0; k < node_points.size(); k++) {
            if (is_2point_high_x(node_points[k].get_data(), item_src, MAX_DIS_2POINT_X)) {
                continue;
            }
            //距离小于MAX_DIS_2_global_transformPOINT开方的合并为1个点
            if (dis_two_point(node_points[k].get_data(), item_src) < MAX_DIS_2POINT) {
                node_points[k].hit();
                is_hitted = true;
                break;
            }
        }
        if (!is_hitted) {
            SubDensityItem item(item_src);
            node_points.push_back(item);
        }
        return is_hitted;
    }
private:    
    /*
    *  函数名称: Is_2point_high_xes.
    *  参数说明: point1 
    *            point2 
    *            dis 特定距离 
    *  返回值: bool
    *  功能描述: 简要计算2个点是不是大于特定距离
    */
    inline bool is_2point_high_x(CloudItem point1, CloudItem point2, double dis){
        return (abs(point1.x - point2.x) > dis || abs(point1.y - point2.y) > dis ||
            abs(point1.z - point2.z) > dis);
    }    
    /*
    *  函数名称: dis_two_points
    *  参数说明: point1 点1
    *            point2 点2
    *  返回值: float
    *  功能描述: 返回2个点的距离平方
    */
    inline float dis_two_point(CloudItem point1, CloudItem point2){
        return (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) *
            (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
    }
};


/*
*   功能描述: 点云密度统计分析         
*/
class CloudDensity {
    SINGLETON_CLASS(CloudDensity)
public:        
    void add_cloud(CloudPtr newpoint);        
    /*
    *  函数名称: get_cloud_with_pos
    *  参数说明: cloud 点云数据
    *            cur_rot 特定位置
    *  功能描述: 根据特定位置获取周边点云数据
    */
    void get_cloud_with_pos(CloudPtr& cloud, Eigen::Matrix4f cur_rot);
    /*  函数名称: get_cloud_with_pos
    *  参数说明: cloud 点云数据
    *            cur_rot 特定位置
    *            dis 距离
    *  功能描述: 根据特定位置和距离获取周边点云数据
    */
    void get_cloud_with_pos(CloudPtr& cloud, Eigen::Matrix4f cur_rot, int dis);

    /*
    *  函数名称: get_cloud_hit_nums
    *  参数说明: cloud 点云数据
    *            hit_num 命中数
    *  功能描述: 获取网格中大于特定命中数的纯净点云
    */
    void get_cloud_hit_num(CloudPtr& cloud, uint64_t hit_num);    

    /*
    *  函数名称: get_avg_hit_nums
    *  返回值: uint64_t
    *  功能描述: 获取网格空间平均命中数
    */    
    uint64_t get_avg_hit_num();    

    /*
    *  函数名称: clears
    *  功能描述: 清空网格空间
    */
    void clear();
    static const int MAX_DIS = 60; //只参考60米内的点云
private:
    boost::unordered_map< MapKeyType, CloudDensityItem> _grid;//网格实体
    uint16_t _idx;    
    /*
    *  函数名称: get_hash_keys
    *  参数说明: x 
    *            y 
    *  返回值: MapKeyType
    *  功能描述: 根据xy坐标快速生成hash
    */
    inline MapKeyType get_hash_key(int x, int y, int z) const{
        MapKeyType key = (((MapKeyType) (x)) << 16) + (MapKeyType) (y) +
                (((MapKeyType) (z)) << 32);
        return key;
    }

    DISALLOW_COPY_AND_ASSIGN(CloudDensity);
};
} // cloud_icp_reg