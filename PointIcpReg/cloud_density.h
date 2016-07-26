/*
*   ��������: �����ܶ�ͳ��,����ͶӰͳ��,�����˶�������������ӵ�,
*            ���ɴ����ĵ���
*/
#pragma once
#include <boost/unordered_map.hpp>
#include <list>
#include <Eigen/Eigen>
#include "cmm_types.h"

namespace cloud_icp_reg {
const double MAX_DIS_2POINT_X = 0.05; //����������
const double MAX_DIS_2POINT = MAX_DIS_2POINT_X* MAX_DIS_2POINT_X; //С��0.01�׾���ĺϳ�Ϊ1����
typedef uint64_t MapKeyType;

/*
*   ��������: ����ͳ���ӵ�Ԫ             
*/
class SubDensityItem {
public:
    SubDensityItem(CloudItem item)
            : hit_num(1)
    {
        data = item;
    }    
    /*
    *  ��������: hit
    *  ��������: ����,����һ�����м���
    */
    inline void hit(){
        hit_num++;
    }    
    /*
    *  ��������: get_data
    *  ����ֵ: CloudItem 
    *  ��������: ��ȡ�ӵ�Ԫ�ڵĵ��ƽڵ�����
    */
    CloudItem get_data(){
        return data;
    }    
    /*
    *  ��������: get_hit_num
    *  ����ֵ: uint64_t
    *  ��������: 
    */
    uint64_t get_hit_num(){
        return hit_num;
    }
private:
    CloudItem data; //���ƽڵ�
    uint64_t hit_num; //������
};

/*
*   ��������: ����ͳ�Ƶ�Ԫ             
*/
class CloudDensityItem {
public:
    std::vector< SubDensityItem> node_points; //�ӵ�Ԫ�б�
    int x;
    int y;
    CloudDensityItem()
            : x(0), y(0)
    {
    }    
    /*
    *  ��������: Add_points 
    *  ����˵��: item_src ����һ�����ƽڵ㵽������
    *  ����ֵ: bool �Ƿ�ɹ�
    *  ��������: 
    */
    bool add_point(CloudItem item_src){
        bool is_hitted = false;
        for (int k = 0; k < node_points.size(); k++) {
            if (is_2point_high_x(node_points[k].get_data(), item_src, MAX_DIS_2POINT_X)) {
                continue;
            }
            //����С��MAX_DIS_2_global_transformPOINT�����ĺϲ�Ϊ1����
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
    *  ��������: Is_2point_high_xes.
    *  ����˵��: point1 
    *            point2 
    *            dis �ض����� 
    *  ����ֵ: bool
    *  ��������: ��Ҫ����2�����ǲ��Ǵ����ض�����
    */
    inline bool is_2point_high_x(CloudItem point1, CloudItem point2, double dis){
        return (abs(point1.x - point2.x) > dis || abs(point1.y - point2.y) > dis ||
            abs(point1.z - point2.z) > dis);
    }    
    /*
    *  ��������: dis_two_points
    *  ����˵��: point1 ��1
    *            point2 ��2
    *  ����ֵ: float
    *  ��������: ����2����ľ���ƽ��
    */
    inline float dis_two_point(CloudItem point1, CloudItem point2){
        return (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) *
            (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
    }
};


/*
*   ��������: �����ܶ�ͳ�Ʒ���         
*/
class CloudDensity {
    SINGLETON_CLASS(CloudDensity)
public:        
    void add_cloud(CloudPtr newpoint);        
    /*
    *  ��������: get_cloud_with_pos
    *  ����˵��: cloud ��������
    *            cur_rot �ض�λ��
    *  ��������: �����ض�λ�û�ȡ�ܱߵ�������
    */
    void get_cloud_with_pos(CloudPtr& cloud, Eigen::Matrix4f cur_rot);
    /*  ��������: get_cloud_with_pos
    *  ����˵��: cloud ��������
    *            cur_rot �ض�λ��
    *            dis ����
    *  ��������: �����ض�λ�ú;����ȡ�ܱߵ�������
    */
    void get_cloud_with_pos(CloudPtr& cloud, Eigen::Matrix4f cur_rot, int dis);

    /*
    *  ��������: get_cloud_hit_nums
    *  ����˵��: cloud ��������
    *            hit_num ������
    *  ��������: ��ȡ�����д����ض��������Ĵ�������
    */
    void get_cloud_hit_num(CloudPtr& cloud, uint64_t hit_num);    

    /*
    *  ��������: get_avg_hit_nums
    *  ����ֵ: uint64_t
    *  ��������: ��ȡ����ռ�ƽ��������
    */    
    uint64_t get_avg_hit_num();    

    /*
    *  ��������: clears
    *  ��������: �������ռ�
    */
    void clear();
    static const int MAX_DIS = 60; //ֻ�ο�60���ڵĵ���
private:
    boost::unordered_map< MapKeyType, CloudDensityItem> _grid;//����ʵ��
    uint16_t _idx;    
    /*
    *  ��������: get_hash_keys
    *  ����˵��: x 
    *            y 
    *  ����ֵ: MapKeyType
    *  ��������: ����xy�����������hash
    */
    inline MapKeyType get_hash_key(int x, int y, int z) const{
        MapKeyType key = (((MapKeyType) (x)) << 16) + (MapKeyType) (y) +
                (((MapKeyType) (z)) << 32);
        return key;
    }

    DISALLOW_COPY_AND_ASSIGN(CloudDensity);
};
} // cloud_icp_reg