/*
*   功能描述: imu姿态还原类
*            根据记录的imu数据,将当前点云姿态还原为基于上一帧的姿态,从而排除各种抖动干扰
*            imu的频率非常高,数据量大,已知点云的时间戳,需要从imu中找出最接近点云的时间戳
*            先进行大而化小,再进行找最接近的时间戳
*            为了避免imu的累计误差,均采用相对于上一点云帧的姿态变化
*/
#pragma once
#include <boost/unordered_map.hpp>
#include <set>
#include <Eigen/Eigen>
#include "cmm_types.h"

namespace cloud_icp_reg {
/*
*   功能描述: imu姿态还原类,还原点云姿态,排除干扰
*/
class ImuTras {
public:
    /*
    *  参数说明: str_path imu文件路径
    *   功能描述: 创建一个imu姿态还原类
    */
    explicit ImuTras(std::string str_path);

    ImuTras()
    {

    }
    /*
     *  函数名称: trans_cloud
     *  参数说明: cloud_src输入点云数据,
     *           cloud_out输出经过变化后的点云数据
     *  返回值: bool 返回是是否找到对应的点云数据
     *  功能描述: 输入一帧点云数据,将点云姿态还原
     */
    bool trans_cloud(const Cloud& cloud_src, Cloud& cloud_out);
    /*
    *  函数名称: get_transaction_mx
    *  参数说明: stamp_cloud 输入点云时间戳,
    *           mat_rot 输出相对于上一帧的旋转平移矩阵,
    *           d_yaw输出相对于上一帧的yaw值,
    *           d_pitch 输出相对于上一帧的pitch值,
    *           d_roll 输出相对于上一帧的roll值,
    *  返回值: bool 返回是否找到最近的imu对应
    *  功能描述: 根据时间戳,返回基于上一帧请求的相对姿态
    */
    bool get_transaction_mx(const uint64_t stamp_cloud, Eigen::Matrix4f& mat_rot, double& d_yaw,
        double& d_pitch, double& d_roll);

	bool ImuTras::get_transaction_mx_mcs(const uint64_t stamp_cloud, Eigen::Matrix4f& mat_rot,
		double& d_yaw, double& d_pitch, double& d_roll);

    inline void get_mid_mt(cv::Point3d& pt_valid, uint64_t stamp, uint64_t stamp1, uint64_t stamp2);
    
    inline Eigen::Matrix4f pt2mat(const cv::Point3d& pt_gradient);

    static void get_matrix(float *cg, double a, double b, double r, double z, double y, double x);
    /*
     *  函数名称: 返回imu姿态还原类
     *  参数说明: 无
     *  返回值: bool 
     *  功能描述: 返回imu数据是否正常
     */
    bool is_valid() const{
        return _is_valid;
    };

private:

    /*
     *  函数名称: get_hash_key
     *  参数说明: stamp 输入的微秒时间戳
     *  返回值: uint64_t 
     *  功能描述: 以1/10毫秒作为map的key值,将大数据量的点云划分成多个区段
     */
    uint64_t get_hash_key(uint64_t stamp) const{
        return stamp / 100000;
    }

    boost::unordered_map< uint64_t, std::set< uint64_t> > _map_mt_idx;//将imu数据划分区段
    boost::unordered_map< uint64_t, cv::Point3d> _map_mt;//imu姿态数据集合
    Eigen::Matrix4f _last_mat;//上一请求的点云帧的姿态
    bool _is_valid;//保存imu数据是否正常
    uint64_t _last_ms;//记录上一次请求的毫秒数,用来做跨越1小时检测,因为点云不具备小时时间戳
    int _last_hour;//记录共跨越的小时数
    cv::Point3d _last_ori;//记录上一帧请求的绝对姿态
	cv::Point3d _last_read_point;//记录上一次读取的imu数据
	bool _is_last_stamp_error;//记录上一帧是不是出错的,防止连续出错记录

    //DISALLOW_COPY_AND_ASSIGN(ImuTras);
};
} // cloud_icp_reg