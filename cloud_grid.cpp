#include "stdafx.h"
#include "cloud_grid.h"
#include "point_cloud_helper.h"

namespace cloud_icp_reg {
using Eigen::Matrix4f;
using boost::unordered_map;
using boost::queue;

const double MAX_DIS_2POINT_X = 0.15;
const double MAX_DIS_2POINT = MAX_DIS_2POINT_X* MAX_DIS_2POINT_X; //С��0.15�׾���ĺϳ�Ϊ1����
const double INIT_MAP_SIZE = 700000.0; //map��ʼ��С
const int GRID_DIS = 10;
//const int MAX_DIS = 40; //����λ�û�ȡ��Χ40���ڵĵ�������,����µĵ���
const bool is_rebuild_cache = true;

CloudGrid::CloudGrid()
        : _idx(0)
{
    //ȫ�ִ洢�ĵ������Ƚϴ�,���ⷴ�������СӰ������.
    _grid.rehash(INIT_MAP_SIZE);
}

void CloudGrid::add_cloud(CloudPtr newpoint)
{
    //����
    _qframe.push_back(newpoint);
    add_cloud_internal(newpoint);
    if (_qframe.size() > 5000) {
        if (is_rebuild_cache) {
            cout << "rebuid cache .... " << _qframe.size() << endl;
            CloudGrid::instance().clear();
            while (_qframe.size() > 1000) {
                //_qframe.pop();
                _qframe.remove(*_qframe.begin());
            }
            while (_qframe.size() > 0) {
                //CloudPtr item=_qframe.front();
                CloudPtr item = *_qframe.begin();
                add_cloud_internal(item);
                //_qframe.pop();
                _qframe.remove(*_qframe.begin());
                cout << "qframe " << _qframe.size() << endl;
            }
            PointCloudHelper::clear_trimesh_cache();
        } else {
            _qframe.remove(*_qframe.begin());
        }
    } else {
        //_qframe.push(newpoint);
    }
}

void CloudGrid::qframe_clear()
{
    _qframe.clear();
}

void CloudGrid::add_cloud_internal(CloudPtr newpoint)
{
    for (int i = 0; i < newpoint->size(); i++) {
        int irow = newpoint->points[i].x;
        int icol = newpoint->points[i].y;
        bool comparev = false;

        MapKeyType key = get_hash_key(irow, icol);
        //�򻯲����Ǳ߽�
        unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.find(key);
        if (it == _grid.end()) {
            CloudGridItem tempnote;
            tempnote.x = irow;
            tempnote.y = icol;
            tempnote.nodepoint.push_back(newpoint->points[i]);
            _grid[key] = tempnote;
        } else {
            comparev = true;
            bool mindis = true;
            for (int k = 0; k < it->second.nodepoint.size(); k++) {
                if (is_2point_high_x(it->second.nodepoint[k], newpoint->points[i],
                    MAX_DIS_2POINT_X)) {
                    continue;
                }
                //����С��MAX_DIS_2_global_transformPOINT�����ĺϲ�Ϊ1����,�����辫ȷ,�����Ż�,����Ҫ�õ�
                if (dis_two_point(it->second.nodepoint[k], newpoint->points[i]) <
                    MAX_DIS_2POINT) {
                    mindis = false;
                    break;
                }
            }
            if (mindis) {
                it->second.nodepoint.push_back(newpoint->points[i]);
            }
        }
    }
}

void CloudGrid::get_cloud_with_pos(CloudPtr& CloudCache, Matrix4f curRot)
{
    get_cloud_with_pos(CloudCache,curRot,MAX_DIS);
}

void CloudGrid::get_cloud_with_pos(CloudPtr& CloudCache, Matrix4f curRot ,int dis){
    _idx++;
    //if (_idx % 5 != 1) {
    //    return;
    //}
    int irow = curRot(0, 3);
    int icol = curRot(1, 3);
    if (CloudCache->size() != 0) {
        CloudCache->clear();
    }
    //���ڵ�ǰλ��ȡ�ض������ڵ���
    for (int i = irow - dis; i < irow + dis; i++) {
        for (int j = icol - dis; j < icol + dis; j++) {
            MapKeyType key = get_hash_key(i, j);
            unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.find(key);
            if (it == _grid.end()) {
            } else {
                for (int k = 0; k < it->second.nodepoint.size(); k++) {
                    CloudCache->points.push_back(it->second.nodepoint[k]);
                }
            }
        }
    }
}

void CloudGrid::get_cloud_with_pos_short(CloudPtr& CloudCache, Matrix4f curRot, int num_short){
	int frame_size = 200 + num_short;
	if(_qframe.size() > frame_size)
	{
		cout << "get cloud short .... "<< frame_size <<endl;
		CloudCache->clear();
		int idx=0;
		std::list<CloudPtr>::iterator it = _qframe.end();
		it--;
		for (; it != _qframe.begin() && idx < frame_size; -- it,idx++)  
		{  
			CloudPtr& item=*it;
			*CloudCache += *item;  
		}  
	}else{
		get_cloud_with_pos(CloudCache,curRot);
	}
}
void CloudGrid::get_grid_cloud(CloudPtr& cloud){
    cloud->clear();
    for (unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.begin();
        it != _grid.end(); it++) {
        for (int i = 0; i < it->second.nodepoint.size(); i++) {
            cloud->push_back(it->second.nodepoint[i]);
        }
    }
}

void CloudGrid::clear(){
    _grid.clear();
    cout << "clear cache ..." << endl;
}
} // cloud_icp_reg