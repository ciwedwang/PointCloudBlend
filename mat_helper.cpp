#include "StdAfx.h"
#include "mat_helper.h"
#include <iostream>

namespace cloud_icp_reg {
using Eigen::Matrix4f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using Eigen::Vector4f;
using std::cout;
using boost::unordered_map;

MatHelper::MatHelper(void){
}


MatHelper::~MatHelper(void){
}

void MatHelper::get_adjust_tras(int rotatecount, Eigen::Matrix4f& adjustTransform){
    CvMat* mat_rot_v = cvCreateMat(3, 1, CV_64F);
    CvMat* mat_rot = cvCreateMat(3, 3, CV_64F);
    cvZero(mat_rot_v);
    if (rotatecount % 2 == 0) {
        double* item = mat_rot_v->data.db;
        double rotate = (double) rotatecount;
        //item[2] = rotate / 180.0;
        item[2] = rotate / 60.0 * M_PI;
        cvRodrigues2(mat_rot_v, mat_rot, NULL);
        adjustTransform(0, 0) = mat_rot->data.db[0];
        adjustTransform(0, 1) = mat_rot->data.db[1];
        adjustTransform(0, 2) = mat_rot->data.db[2];
        adjustTransform(1, 0) = mat_rot->data.db[3];
        adjustTransform(1, 1) = mat_rot->data.db[4];
        adjustTransform(1, 2) = mat_rot->data.db[5];
        adjustTransform(2, 0) = mat_rot->data.db[6];
        adjustTransform(2, 1) = mat_rot->data.db[7];
        adjustTransform(2, 2) = mat_rot->data.db[8];
    } else {
        double* item = mat_rot_v->data.db;
        double rotate = (double) rotatecount;
        item[2] = 0 - rotate / 60.0 * M_PI;
        cvRodrigues2(mat_rot_v, mat_rot, NULL);
        adjustTransform(0, 0) = mat_rot->data.db[0];
        adjustTransform(0, 1) = mat_rot->data.db[1];
        adjustTransform(0, 2) = mat_rot->data.db[2];
        adjustTransform(1, 0) = mat_rot->data.db[3];
        adjustTransform(1, 1) = mat_rot->data.db[4];
        adjustTransform(1, 2) = mat_rot->data.db[5];
        adjustTransform(2, 0) = mat_rot->data.db[6];
        adjustTransform(2, 1) = mat_rot->data.db[7];
        adjustTransform(2, 2) = mat_rot->data.db[8];
    }
    cvReleaseMat(&mat_rot_v);
    mat_rot_v = NULL;
    mat_rot = NULL;
}

bool MatHelper::get_rot_with_stamp(const unordered_map< uint64_t, Matrix4f>& rot_stamps,
    const uint64_t stamp, Matrix4f& rot_out){
    if (rot_stamps.find(stamp) == rot_stamps.end()) {
        //cout << "stamp rot not found :" << stamp;
        return false;
    } else {
        Matrix4f pair_transform = (rot_stamps.find(stamp))->second;
        rot_out = pair_transform;
        return true;
    }
}

bool MatHelper::check_rot_valid(Matrix4f mat_rot){
    //Matrix4f pairTransformCache;
    //pairTransformCache = g_status._global_transform.inverse()*trans_pair * trans_dst;
    //if (abs(pairTransformCache(0, 3)) > MAXTRACEDIS ||
    //    abs(pairTransformCache(1, 3)) > MAXTRACEDIS ||
    //    abs(pairTransformCache(2, 3)) > MAXTRACEDIS || abs(pairTransformCache(2, 2) - 1) > 0.1) {
    //        printf("stay......................\n");
    //        cout << "stay......................" << endl;
    //        trans_global=g_status._global_transform;
    //        //return;
    //} 
    //jake todo 背包暂时去掉
    //return true;
	cout<< "rot check " << abs(mat_rot(2, 2) - 1) << endl;
    if (abs(mat_rot(2, 2) - 1) >
        0.1/* || abs(matRot(0, 3)) + abs(matRot(1, 3)) + abs(matRot(2, 3))>0.5*/) {
        cout << "rot error ignore ........................................."
            << abs(mat_rot(0, 3)) + abs(mat_rot(1, 3)) + abs(mat_rot(2, 3)) << endl;
        return false;
    }

    return true;
}

bool MatHelper::check_rot_valid(uint64_t stamp, int idx_frame, Matrix4f rot1,
    Matrix4f rot2, double d_yaw){
    //cout<<idx_frame<<" "<<stamp<<" "<<abs(rot1(0,3)-rot2(0,3))<<" "
    //    <<abs(rot1(1,3)-rot2(1,3))<<" "<<abs(rot1(2,3)-rot2(2,3))<<" "<<d_yaw<<endl;
//	if(check_rot_valid(rot1)==false)
//	{
//		cout << "rot check failed" << endl;
//		return false;
//	}
    return check_rot_valid(stamp, idx_frame, d_yaw);
}

bool MatHelper::check_rot_valid(uint64_t stamp, int idx_frame, double d_yaw){
    double yaw = abs(d_yaw);
    if (yaw > 270) {
        yaw = 360 - yaw;
    }

    //if (yaw >= 6) {
    //    return false;
    //} else {
    //    return true;
    //}

    //jake todo 过快的无法修正好,可能需要放宽
    if (yaw >= 30) {
        return false;
    } else {
        return true;
    }
}

bool MatHelper::check_yaw_valid_limit(double d_yaw){
    double yaw = abs(d_yaw);
    if (yaw > 270) {
        yaw = 360 - yaw;
    }

    if (yaw >= 3) {
        return false;
    } else {
        return true;
    }
}

void MatHelper::matrix2to4(Matrix4f& transform, Matrix2f& r, Vector2f& t){
    transform << r(0, 0), r(0, 1), 0, t(0, 0),
    r(1, 0), r(1, 1), 0, t(1, 0),
    0, 0, 1, 0,
    0.0, 0.0, 0.0, 1.0;
}

void MatHelper::matrix42vector4(const Matrix4f transform, Vector4f& t){
    t << transform(0, 3), transform(1, 3), transform(2, 3),
    transform(3, 3);
}


void MatHelper::matrix3to4(Matrix4f& transform, Matrix3f& r, Vector3f& t){
    transform << r(0, 0),r(0, 1),r(0, 2),t(0, 0),
    r(1, 0),r(1, 1),r(1, 2),t(1, 0),
    r(2, 0),r(2, 1),r(2, 2),t(2, 0),
    0.0,0.0,0.0,1.0;
}

bool MatHelper::rotate_arbitrary_line(Eigen::Vector3f vec_z_normal, const float angle,
    Matrix4f& mat_rot_out){
    float a = 0;
    float b = 0;
    float c = 0;
//    cvNormalize(vec_z_normal, vec_z_normal);
	float nor = vec_z_normal.norm();
    float u = vec_z_normal(0)/nor;
    float v = vec_z_normal(1)/nor;
    float w = vec_z_normal(2)/nor;

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

float get_normal_angle(float& d_yaw)
{
	if (d_yaw<0)
	{
		d_yaw += 360;
	}else{
		d_yaw -= 360;
	}
	return d_yaw;
}



}//cloud_icp_reg
