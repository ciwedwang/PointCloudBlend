/*
*   功能描述: 点云全局cache
*            用于保存全局的点云,和根据当前位置获取特定范围内的点云数据
*/
#pragma once
#include <boost/unordered_map.hpp>
#include <list>
#include <Eigen/Eigen>
#include "cmm_types.h"

namespace cloud_icp_reg {
typedef unsigned long MapKeyType;

class CloudGridItem {
public:
    Cloud nodepoint;
    int x;
    int y;
    CloudGridItem()
            : x(0), y(0)
    {
    }
    bool cmp_pos(int xTmp, int yTmp){
        if (x == xTmp && y == yTmp) {
            return true;
        } else
            return false;
    }
};


class CloudGrid {
    SINGLETON_CLASS(CloudGrid)
public:
    void qframe_clear();
    void add_cloud(CloudPtr newpoint);
    void get_cloud_with_pos(CloudPtr& CloudCache, Eigen::Matrix4f curRot);
    void get_cloud_with_pos(CloudPtr& CloudCache, Eigen::Matrix4f curRot ,int dis);
	void get_cloud_with_pos_short(CloudPtr& CloudCache, Eigen::Matrix4f curRot, int num_short);
    void get_grid_cloud(CloudPtr& Cloud);
    void clear();
	static const int MAX_DIS = 60;
private:
	std::list<CloudPtr> _qframe;
    boost::unordered_map< MapKeyType, CloudGridItem> _grid;
    uint16_t _idx;
    MapKeyType get_hash_key(int x, int y) const{
        MapKeyType key = (((MapKeyType) (x)) << 16) + (MapKeyType) (y);
        return key;
    }

    float dis_two_point(CloudItem point1, CloudItem point2){
        return (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) *
            (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
    }
    bool is_2point_high_x(CloudItem point1, CloudItem point2, double x){
        return (abs(point1.x - point2.x) > x || abs(point1.y - point2.y) > x ||
            abs(point1.z - point2.z) > x);
    }
	void add_cloud_internal(CloudPtr newpoint);
    DISALLOW_COPY_AND_ASSIGN(CloudGrid);
};
} // cloud_icp_reg