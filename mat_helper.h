#pragma once
#include <Eigen/Eigen>
#include <boost/unordered_map.hpp>
#include "cmm_types.h"

namespace cloud_icp_reg {
class MatHelper {
public:
    MatHelper(void);
    ~MatHelper(void);
    static void get_adjust_tras(int rotatecount, Eigen::Matrix4f& adjustTransform);
    static void matrix2to4(Eigen::Matrix4f& transform, Eigen::Matrix2f& r, Eigen::Vector2f& t);
    static void matrix42vector4(const Eigen::Matrix4f transform, Eigen::Vector4f& t);
    static void matrix3to4(Eigen::Matrix4f& transform, Eigen::Matrix3f& r, Eigen::Vector3f& t);
    static bool check_rot_valid(Eigen::Matrix4f mat_rot);
    static bool get_rot_with_stamp(
        const boost::unordered_map< uint64_t, Eigen::Matrix4f>& rot_stamps,
        const uint64_t stamp, Eigen::Matrix4f& rot_out);
    static bool check_rot_valid(uint64_t stamp, int idx_frame, Eigen::Matrix4f rot1,
        Eigen::Matrix4f rot2, double d_yaw);
    static bool check_rot_valid(uint64_t stamp, int idx_frame, double d_yaw);
    static bool check_yaw_valid_limit(double d_yaw);
	static bool MatHelper::rotate_arbitrary_line(Eigen::Vector3f vec_z_normal, const float angle,
		Eigen::Matrix4f& mat_rot_out);
	static float MatHelper::get_normal_angle(float& d_yaw);
private:
    DISALLOW_COPY_AND_ASSIGN(MatHelper);
};
}//cloud_icp_reg