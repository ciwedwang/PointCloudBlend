#include "stdafx.h"
#include "cloud_density.h"
#include "point_cloud_helper.h"

namespace cloud_icp_reg {
using Eigen::Matrix4f;
using boost::unordered_map;

const double INIT_MAP_SIZE = 700000.0; //map初始大小

CloudDensity::CloudDensity(){
    //全局存储的点云量比较大,避免反复扩充大小影响性能.
    _grid.rehash(INIT_MAP_SIZE);
}


void CloudDensity::add_cloud(CloudPtr pt_new){
    for (int i = 0; i < pt_new->size(); i++) {
        int irow = pt_new->points[i].x;
        int icol = pt_new->points[i].y;
        int ih = pt_new->points[i].z;
        bool comparev = false;

        MapKeyType key = get_hash_key(irow, icol, ih);
        //简化不考虑边界
        unordered_map< MapKeyType, CloudDensityItem>::iterator it = _grid.find(key);
        if (it == _grid.end()) {
            CloudDensityItem tempnote;
            tempnote.x = irow;
            tempnote.y = icol;
            tempnote.node_points.push_back(pt_new->points[i]);
            _grid[key] = tempnote;
        } else {
            comparev = true;

            it->second.add_point(pt_new->points[i]);
        }
    }
}

void CloudDensity::get_cloud_with_pos(CloudPtr& cloud, Matrix4f cur_rot){
    get_cloud_with_pos(cloud, cur_rot, MAX_DIS);
}

void CloudDensity::get_cloud_with_pos(CloudPtr& cloud, Matrix4f cur_rot, int dis){
    int irow = cur_rot(0, 3);
    int icol = cur_rot(1, 3);
    if (cloud->size() != 0) {
        cloud->clear();
    }
    //基于当前位置取特定区块内点云
    for (int i = irow - dis; i < irow + dis; i++) {
        for (int j = icol - dis; j < icol + dis; j++) {
            for (int z = icol - dis; z < icol + dis; z++) {
                MapKeyType key = get_hash_key(i, j, z);
                unordered_map< MapKeyType, CloudDensityItem>::iterator it = _grid.find(
                        key);
                if (it == _grid.end()) {
                } else {

                    for (int k = 0; k < it->second.node_points.size(); k++) {
                        cloud->points.push_back(it->second.node_points[k].get_data());
                    }
                }
            }
        }
    }
}

uint64_t CloudDensity::get_avg_hit_num(){
    uint64_t hit_num_all = 0;
    uint64_t num = 0;
    for (unordered_map< MapKeyType, CloudDensityItem>::iterator it = _grid.begin();
        it != _grid.end(); it++) {
        for (int i = 0; i < it->second.node_points.size(); i++) {
            hit_num_all += it->second.node_points[i].get_hit_num();  
            num ++;
        }
    }
    hit_num_all /= num; 
    return hit_num_all;
}

void CloudDensity::get_cloud_hit_num(CloudPtr& cloud, uint64_t hit_num){
    cloud->clear();
    for (unordered_map< MapKeyType, CloudDensityItem>::iterator it = _grid.begin();
        it != _grid.end(); it++) {
        for (int i = 0; i < it->second.node_points.size(); i++) {
            if (hit_num >= it->second.node_points[i].get_hit_num()) {
                continue;
            }
            cloud->push_back(it->second.node_points[i].get_data());
        }
    }
}

void CloudDensity::clear(){
    _grid.clear();
    cout << "clear grid ..." << endl;
}
} // cloud_icp_reg