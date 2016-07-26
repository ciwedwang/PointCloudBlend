#include "stdafx.h"
#include "project_plane.h"

namespace cloud_icp_reg {
using Eigen::Matrix4f;

const int MAX_ITERATIONS = 1000;//求最大平面的最大迭代次数
const double DIS_THRESHOLD = 0.1;//点与点相关性的距离阈值
const int ANGLE_THRESHOLD = 10.0f * (M_PI / 180.0f);//求最大面所用的角度阈值

ProjectPlane::ProjectPlane(const Cloud& cloud_src, const Cloud& cloud_trace){
    pcl::copyPointCloud(cloud_src, _cloud);
    pcl::copyPointCloud(cloud_trace, _cloud_trace);
    get_rot(_mat_rot);
    //z归0
    pcl::transformPointCloud(_cloud_trace, _cloud_trace, _mat_rot);
    float dz=0;
    for (int i=0;i<_cloud_trace.size();i++)
    {
         dz += _cloud_trace.points[i].z;
    }
    dz/=_cloud_trace.size();
    pcl::transformPointCloud(_cloud, _cloud, _mat_rot);
    for (int i=0;i<_cloud.size();i++)
    {
        _cloud.points[i].z = _cloud.points[i].z - dz;
    }
}

void ProjectPlane::get_plane(const float height_min, const float height_max, Cloud& cloud_out){
    cloud_out.clear();
    for (int i = 0; i < _cloud.size(); i++) {
        float height = _cloud.points[i].z;
        if (height > height_min && height < height_max) {
            cloud_out.push_back(_cloud.points[i]);
        }
    }
}

void ProjectPlane::get_cloud(Cloud& cloud_out){
    pcl::copyPointCloud(_cloud, cloud_out);
}

void ProjectPlane::get_rot(Matrix4f& mat_rot_out){
    mat_rot_out = Matrix4f::Identity();
    Cloud cloud_floor;
    //创建结果点云shared_ptr
    //jake 这里利用轨迹确定最大平面
    CloudPtr cloud_process(new Cloud);
    //pcl::copyPointCloud(_cloud, *cloud_process);
    pcl::copyPointCloud(_cloud_trace, *cloud_process);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // 创建分割对象
    pcl::SACSegmentation< CloudItem> ssg;
    ssg.setOptimizeCoefficients(true);
    ssg.setModelType(pcl::SACMODEL_PLANE);
    ssg.setMethodType(pcl::SAC_RANSAC);
    ssg.setMaxIterations(MAX_ITERATIONS);
    ssg.setDistanceThreshold(DIS_THRESHOLD);
    //Eigen::Vector3f vc;
    //vc(0) = 0;
    //vc(1) = 0;
    //vc(2) = 1;
    //ssg.setAxis(vc);
    //ssg.setEpsAngle(ANGLE_THRESHOLD);
    // 创建滤波器对象
    pcl::ExtractIndices< CloudItem> extract;
    int num_points = (int) cloud_process->points.size();
    // 获取点云中的最大平面
    ssg.setInputCloud(cloud_process);
    ssg.segment(*inliers, *coefficients);
    extract.setInputCloud(cloud_process);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_floor);
    Eigen::Vector4f vec_normal_eigen(0, 0, 0, 0);
    float curvature = 0.0;
    //计算法向量
    computePointNormal(cloud_floor, vec_normal_eigen, curvature);
    //建立z轴向量,法向量表示
    CvMat* vec_z = cvCreateMat(3, 1, CV_32FC1);
    vec_z->data.fl[0] = 0.0;
    vec_z->data.fl[1] = 0.0;
    vec_z->data.fl[2] = 1.0;
    CvMat* vec_normal = cvCreateMat(3, 1, CV_32FC1);
    vec_normal->data.fl[0] = vec_normal_eigen(0);
    vec_normal->data.fl[1] = vec_normal_eigen(1);
    vec_normal->data.fl[2] = vec_normal_eigen(2);
    CvMat* vec_z_normal = cvCreateMat(3, 1, CV_32FC1);
    cvCrossProduct(vec_z, vec_normal, vec_z_normal);
    //计算法向量和z轴夹角
    float ang = acos((float) cvDotProduct(vec_z, vec_normal) /
            (vec_normal_eigen(0) * vec_normal_eigen(0) + vec_normal_eigen(1) * vec_normal_eigen(1) +
            vec_normal_eigen(2) * vec_normal_eigen(2)));
    //获得法向量到z轴的旋转矩阵
    rotate_arbitrary_line(vec_z_normal, ang, mat_rot_out);
    cvReleaseMat(&vec_z_normal);
    cvReleaseMat(&vec_normal);
    cvReleaseMat(&vec_z);
}

//参考自http://www.cnblogs.com/graphics/archive/2012/08/10/2627458.html
bool ProjectPlane::rotate_arbitrary_line(CvMat* vec_z_normal, const float angle,
    Matrix4f& mat_rot_out){
    if (NULL == vec_z_normal) {
        return false;
    }
    float a = 0;
    float b = 0;
    float c = 0;
    cvNormalize(vec_z_normal, vec_z_normal);
    float u = vec_z_normal->data.fl[0];
    float v = vec_z_normal->data.fl[1];
    float w = vec_z_normal->data.fl[2];

    float uu = u* u;
    float uv = u* v;
    float uw = u* w;
    float vv = v* v;
    float vw = v* w;
    float ww = w* w;
    float au = a* u;
    float av = a* v;
    float aw = a* w;
    float bu = b* u;
    float bv = b* v;
    float bw = b* w;
    float cu = c* u;
    float cv = c* v;
    float cw = c* w;

    float costheta = cosf(angle);
    float sintheta = sinf(angle);

    mat_rot_out(0, 0) = uu + (vv + ww) * costheta;
    mat_rot_out(0, 1) = uv * (1 - costheta) + w * sintheta;
    mat_rot_out(0, 2) = uw * (1 - costheta) - v * sintheta;
    mat_rot_out(0, 3) = 0;

    mat_rot_out(1, 0) = uv * (1 - costheta) - w * sintheta;
    mat_rot_out(1, 1) = vv + (uu + ww) * costheta;
    mat_rot_out(1, 2) = vw * (1 - costheta) + u * sintheta;
    mat_rot_out(1, 3) = 0;

    mat_rot_out(2, 0) = uw * (1 - costheta) + v * sintheta;
    mat_rot_out(2, 1) = vw * (1 - costheta) - u * sintheta;
    mat_rot_out(2, 2) = ww + (uu + vv) * costheta;
    mat_rot_out(2, 3) = 0;

    mat_rot_out(3, 0) = (a * (vv + ww) - u * (bv + cw)) * (1 - costheta) + (bw - cv) * sintheta;
    mat_rot_out(3, 1) = (b * (uu + ww) - v * (au + cw)) * (1 - costheta) + (cu - aw) * sintheta;
    mat_rot_out(3, 2) = (c * (uu + vv) - w * (au + bv)) * (1 - costheta) + (av - bu) * sintheta;
    mat_rot_out(3, 3) = 1;
    return true;
}
} // cloud_icp_reg