#pragma once
#include "cmm_types.h"
#include <Eigen/Eigen>
#include <boost/unordered_map.hpp>

namespace cloud_icp_reg {
    class PointCloudHelper {
    public:
        PointCloudHelper();
        ~PointCloudHelper();

        static void remove_outlier(CloudPtr& cloud_src, int min_count);
        static bool PointCloudHelper::cloud_convert_with_rot(
            const boost::unordered_map< uint64_t, Eigen::Matrix4f>& rot_stamps, const uint64_t stamp,
            const CloudPtr& cloud_temp, CloudPtr& cloud_trans, Eigen::Matrix4f& rot_out);
        static bool PointCloudHelper::cloud_convert_with_rot(
            const boost::unordered_map< int, Eigen::Matrix4f>& rot,
            const int frame_count, const CloudPtr& cloud_temp, CloudPtr& cloud_trans,
            Eigen::Matrix4f& rot_out);

        static void add_trace_point(CloudPtr& cloud_temp, int r = 255, int g = 0, int b = 0);
        static void add_trace_point(CloudPtr& cloud_temp, Eigen::Matrix4f rot);
        static void do_lum_elch(vector< pair< pair<uint64_t, uint64_t>, Eigen::Matrix4f> >& map_loops,
            vector< CloudPtr>& clouds_line, vector< Eigen::Matrix4f>& optimize_rot, const string& pcap_file, const int output_pcd_index, const size_t add_point_cloud_start_index);
        static float get_rot_icp_pl(CloudPtr cloud_src, CloudPtr cloud_temp, Eigen::Matrix4f& mat_rot);

        static bool loop_detection(const int end, const vector< CloudPtr>& clouds, double dist,
            int& first, int& last);
        static bool loop_detection(const int end, map< int,Eigen::Matrix4f> rots, double dist,
            int& first, int& last);

        static void find_nearest_point(CloudPtr cloud_src, CloudPtr cloud_dst, CloudPtr cloud_sim);
        static void set_zero_z(CloudPtr cloud_src);
        static void build_face(CloudPtr cloud_src);
        static void remove_duplicate(CloudPtr cloudSrc, const float voxel_grid_size);
        static void remove_duplicate(CloudPtr cloudSrc, const float voxel_grid_size, int min_num);
        static float cloudi2cloud(CloudIConstPtr cloud_src, CloudPtr cloud_out, int sample_num = 7,
            bool is_line = false, float out_side = -1, int min_ins = -1);
        static float get_rot_icp(CloudPtr cloud_src, CloudPtr cloud_temp, Eigen::Matrix4f& mat_rot, bool do_scale = false, bool do_affine = false);
        static float get_rot_icp2(CloudPtr cloud_src, CloudPtr cloud_temp, Eigen::Matrix4f& mat_rot);
        static void get_segment_plane(CloudPtr point_cloud_ptr, CloudPtr pcPlane);
        static void get_reduced_cloud(CloudPtr cloud_tras, Eigen::Matrix4f cur_out);
        static void get_cloud_line(const CloudPtr cloud_src, CloudPtr cloud_out, int level);
		static float get_cloud_max_width(Cloud& cloud);
        static void fix_cloud_cache(Cloud& cloud_cache, float f_ins, Eigen::Matrix4f& local);
        static void PointCloudHelper::clear_trimesh_cache();
        static float PointCloudHelper::get_rot_icp_cache(CloudPtr cloud_src, CloudPtr cloud_temp,
            Eigen::Matrix4f& mat_rot);
		static float get_cloud_max_width_rate(Cloud& cloud_cache,float width);
		static void PointCloudHelper::reduce_cloud(Cloud& cloud, float f_ins);

    private:
        static double point2plane_dis(double x0, double y0, double z0);
        static double point2origin_dis(double x0, double y0, double z0);
        static uint32_t intensity2color(int max_weight, int intensity_data);
    };
}// cloud_icp_reg
