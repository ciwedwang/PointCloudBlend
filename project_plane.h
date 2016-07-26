/*
*   功能描述: 点云切面获取类
*            将各种倾斜姿态的三维点云场景,修正为地面法向量平行z轴方向,
*            地面平行于XOY平面,并根据指定的高度范围截取不同点云切面
*            
*/
#pragma once
#include <Eigen/Eigen>
#include <opencv/cv.h>
#include "cmm_types.h"

namespace cloud_icp_reg {
/*
*   功能描述: 点云rotate_arbitrary_line切面获取类
*/
class ProjectPlane {
public:
    /*
    *  参数说明: cloud_src 输入点云数据
    *   功能描述: 创建一个点云切面获取类
    */
    explicit ProjectPlane(const Cloud& cloud_src, const Cloud& cloud_trace);
    /*
     *  函数名称: get_plane  
     *  参数说明: height_min 点云切面的最小高度,
                    height_max 点云切面的最大高度,
                    cloud_out返回的点云切面
     *  返回值: void
     *  功能描述: 返回特定高度范围内的点云切面
     */
    void get_plane(const float height_min, const float height_max, Cloud& cloud_out);
    /*
     *  函数名称: get_cloud
     *  参数说明: cloud_out返回的点云数据
     *  返回值: void 
     *  功能描述: 返回法向量和z轴重合的点云数据
     */
    void get_cloud(Cloud& cloud_out);
	Eigen::Matrix4f _mat_rot;
private:
    /*
     *  函数名称: get_rot
     *  参数说明: mat_rot_out得到法向量变道z轴方向的旋转矩阵
     *  返回值: void 
     *  功能描述: 获得将地面法向量修正为z轴,所需要的旋转矩阵
     */
    void get_rot(Eigen::Matrix4f& mat_rot_out);
    /*
    *  函数名称: rotate_arbitrary_line
    *  参数说明: vec_z_normal旋转轴向量, angle角度 ,mat_rot_out返回旋转矩阵
    *  返回值: bool
    *  功能描述:绕任意向量旋转特定的角度,得到旋转矩阵
    */
    bool rotate_arbitrary_line(CvMat* vec_z_normal, const float angle, Eigen::Matrix4f& mat_rot_out);

    Cloud _cloud;//存储法向量方向矫正后的点云数据
    Cloud _cloud_trace;

    DISALLOW_COPY_AND_ASSIGN(ProjectPlane);
};
} // cloud_icp_reg