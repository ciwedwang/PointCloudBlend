#include "stdafx.h"
#include <sstream>
#include <queue>
#include <iostream>
#include <fstream>
#include <map>
#include<windows.h>
#include <time.h>
#include <sstream>

#include "ficp_tools.h"
#include "lum.h"
#include "hdl_viewer_grabber_helper.h"
#include "imu_tras.h"
#include "cloud_grid.h"
#include "movement_model.h"
#include "file_helper.h"
#include "string_helper.h"
#include "project_plane.h"
#include "point_cloud_helper.h"
#include "process_status.h"
#include "mat_helper.h"
#include "pcl_pair_visualizer.h"
#include "config.h"
#include "feature_align.h"
#include "cloud_density.h"

// 建立场景时自动进行回环检测做回环优化
#define AUTOMATIC_LOOP_DETECTION_WHILE_BUILD_SCENERY 0

// #include "ndt.h"
// #include <ndt_map.h>
// #include <oc_tree.h>
// #include <NDT_histogram.h>

namespace cloud_icp_reg {
const string CLOUD_NAME = "HDL";
const float RDUMPLICATE_VALUE = 0.2f;
const int processCacheMinNum = 5;
const float LOOPOUTSIDE = 60;
const float MAXTRACEDIS = 0.4f;
const float MAX_WIDTH_CORNER = 5;


using std::cout;
using Eigen::Matrix4f;
using boost::unordered_map;
using std::list;

ImuTras* g_imu_trans;
PcapHelper* g_grabber = NULL;
MovementModel g_move_model_after;
ProcessStatus g_status;

void process_house_back(uint64_t stamp)
{
    if (g_status._house_back_stamp.size() > 0 && 
        g_status._house_back_stamp.front() >= stamp)
    {
        CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
            g_status._global_transform, 20);
        CloudGrid::instance().clear();
        CloudGrid::instance().add_cloud(g_status._cloud_cache);
        g_status._house_back_stamp.pop_front();
    }
}

CloudPtr process_end(const CloudIConstPtr& cloud, CloudPtr& cloud_temp_src,
    CloudPtr cloud_tras, bool is_update_display){
    if (is_update_display) {
        FileHelper::write_file_rot(g_status._frame_count_total, g_status._pcap_file,
            g_status._global_transform, true, cloud->header.stamp / 1000);
        // zhangyuzhi 转移到process_loop中
        // g_status._optimize_rot.push_back(g_status._global_transform);
    } else {
        // 记录拼合时跳过写入文件的帧
        g_status._ignore_invalid_stamp.insert(cloud->header.stamp);
    }
    g_status._cloud_src->clear();
    g_status._cloud_src = cloud_temp_src;
    cloud_temp_src = CloudPtr(new Cloud);
    g_status._last_mt = Matrix4f::Identity();

    int num_per_show = 10;
    if (g_status._is_debug_mode)
    {
        num_per_show = 10;
    }else {
        if (g_status._is_batch_mode == true)
        {
            num_per_show = 50;
        }
    }
    if (g_status._frame_count_process % num_per_show == 0 && is_update_display) {
        *g_status._cloud_result = *g_status._cloud_result + *cloud_tras;
        PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.3);
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    }
    //PclPairVisualizer::instance().spin_once();    
    //PclPairVisualizer::instance().update_cloud_left(g_status._cloud_cache);
    return cloud_temp_src;
}

void process_update_rot(Matrix4f trans_pair, Matrix4f trans_dst, CloudPtr cloud_temp_src,
    CloudPtr cloud_tras, const CloudIConstPtr& cloud, double& d_yaw,float max_dis){
    Eigen::Matrix4f trans_global = trans_pair* trans_dst;
    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_global);

    int num_per_show = 10;
    if (g_status._is_debug_mode)
    {
        num_per_show = 1;
    }else {
        if (g_status._is_batch_mode == true)
        {
            num_per_show = 50;
        }
    }
    if (g_status._frame_count_process % num_per_show == 0)
    {
        PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    }

    //if(!MatHelper::check_rot_valid(trans_global))
    //{
    //    cout << "ignore frame rotate"<<endl;
    //    return;
    //}
    if (!g_move_model_after.UpdataModel(trans_global,max_dis)) {
        cout << "model fix .........." << endl;
        pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_global);
        PclPairVisualizer::instance().update_cloud_right(cloud_tras);
        return;
    }

    //检查矩阵是否合理
    if (!MatHelper::check_rot_valid(cloud->header.stamp / 1000, g_status._frame_count_process,
        trans_pair * trans_dst, trans_global, d_yaw)) {
        //g_status._queue_cloud_tras.pop();
        process_end(cloud, cloud_temp_src, cloud_tras, false);
        cout << "ignore frame yaw ................" << d_yaw << endl;
        return;
    }

    g_status._global_transform = trans_global;
    //cout << gIgnoreFrameCount <<endl<< gGlobalTransform<<endl

    if (g_status._frame_count_process < processCacheMinNum) {
        CloudGrid::instance().add_cloud(cloud_tras);
        CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
            g_status._global_transform);
        process_house_back(cloud->header.stamp / 1000);
    } else {
        CloudPtr cloud_cache_tras(new Cloud);
        pcl::copyPointCloud(*cloud_tras, *cloud_cache_tras);
        g_status._queue_cloud_tras.push(cloud_cache_tras);
        if (g_status._queue_cloud_tras.size() >= processCacheMinNum) {
            CloudPtr cloud_cache_tras_temp = g_status._queue_cloud_tras.front();
            g_status._queue_cloud_tras.pop();
            CloudGrid::instance().add_cloud(cloud_cache_tras_temp);
            CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
                g_status._global_transform);
            PointCloudHelper::remove_duplicate(g_status._cloud_cache,0.15);
			//PointCloudHelper::fix_cloud_cache(*g_status._cloud_cache, LOOPOUTSIDE,
			//	g_status._global_transform);
            cout << "dup g_status._cloud_cache size: " << g_status._cloud_cache->points.size()
                << endl;
            process_house_back(cloud->header.stamp / 1000);
        }
    }

    process_end(cloud, cloud_temp_src, cloud_tras, true);
}

bool process_rotate_dropped(CloudPtr cloud_tras, Matrix4f& trans_dst, CloudPtr cloud_temp_src){
    float error2 = -1;
    Matrix4f trans_pair = Matrix4f::Identity();
    Matrix4f trans_rotate = Matrix4f::Identity();
    MatHelper::get_adjust_tras(g_status._rotate_count, trans_rotate);
    g_status._rotate_count++;
    cout << ".........................." << g_status._rotate_count << endl;
    printf("...........................%d\n", g_status._rotate_count);
    if (g_status._rotate_count > 60) {
        g_status._skip_frame++;
        return false;
    }
    //jake
    Matrix4f trans_dst_rotate = trans_dst* trans_rotate;
    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);

    PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras, trans_pair);
    cout << "error2: " << error2 << endl;
    if (MatHelper::check_rot_valid(trans_pair) == false ||
        //error2 > 0.15 + g_status._skip_frame / 20.0 / 4.0 || error2 < 0) {
        error2 > 0.15 + g_status._skip_frame * 0.03 || error2 < 0) {
        return process_rotate_dropped(cloud_tras, trans_dst, cloud_temp_src);
    } else {
        g_status._skip_frame = 0;
        trans_dst = trans_dst_rotate;
        //trans_dst = trans_rotate * trans_dst;
        //process_update_rot(trans_pari, trans_dst, cloud_temp_src, cloud_tras, cloud);
        return true;
    }
}

bool process_rotate(CloudPtr cloud_tras, Matrix4f& trans_dst, CloudPtr cloud_temp_src
    , Matrix4f& mtTmp){
    float error1 = -1;
    Matrix4f trans_pair = Matrix4f::Identity();
    Matrix4f trans_rotate = Matrix4f::Identity();
    MatHelper::get_adjust_tras(g_status._rotate_count, trans_rotate);
    g_status._rotate_count++;
    cout << ".........................." << g_status._rotate_count << endl;
    printf("...........................%d\n", g_status._rotate_count);
    if (g_status._rotate_count > 60) {
        g_status._skip_frame++;
        return false;
    }

    //jake
    Matrix4f trans_dst_rotate = g_status._global_transform * trans_rotate * mtTmp;
    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);
    PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    //jake todo 检测旋转通过1的准确性
    CloudPtr cloud_src_last(new Cloud());
    pcl::transformPointCloud(*g_status._cloud_src, *cloud_src_last, g_status._global_transform);
    error1 = PointCloudHelper::get_rot_icp(cloud_src_last, cloud_tras, trans_pair);
    //error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras, trans_pair);

    cout << "error1: " << error1 << endl;
    //jake todo 按照距离计算
    //if (MatHelper::check_rot_valid(trans_pair) == false ||
    if (error1 > 0.15 + g_status._skip_frame / 20.0 / 4.0 || error1 < 0) {
            //if (MatHelper::check_rot_valid(trans_pair) == false ||
            //    error1 > 0.15 + g_status._skip_frame * 0.08  || error1 < 0) {
            return process_rotate(cloud_tras, trans_dst, cloud_temp_src, mtTmp);
    } else {
        g_status._skip_frame = 0;
        trans_dst_rotate = trans_pair * trans_dst_rotate;
        pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);
		//jake 20150915 针对进入旋转的数据，坚持error1 
		trans_pair = Matrix4f::Identity();
        //error1 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras, trans_pair);
        trans_dst = trans_pair * trans_dst_rotate;
        return true;
    }
    ////jake
    //Matrix4f trans_dst_rotate = g_status._global_transform * trans_rotate * mtTmp;
    //pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);
    //PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    ////jake todo 检测旋转通过1的准确性
    //CloudPtr cloud_src_last(new Cloud());
    //pcl::transformPointCloud(*g_status._cloud_src, *cloud_src_last, g_status._global_transform);
    //error1 = PointCloudHelper::get_rot_icp(cloud_src_last, cloud_tras, trans_pair);
    ////error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras, trans_pair);

    //cout << "error1: " << error1 << endl;
    ////jake todo 按照距离计算
    //if (MatHelper::check_rot_valid(trans_pair) == false ||
    //    error1 > 0.15 + g_status._skip_frame / 20.0 / 4.0 || error1 < 0) {
    ////if (MatHelper::check_rot_valid(trans_pair) == false ||
    ////    error1 > 0.15 + g_status._skip_frame * 0.08  || error1 < 0) {
    //        return process_rotate(cloud_tras, trans_dst, cloud_temp_src, mtTmp);
    //} else {
    //    g_status._skip_frame = 0;
    //    trans_dst_rotate = trans_pair * trans_dst_rotate;
    //    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);
    //    error1 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras, trans_pair);
    //    trans_dst = trans_pair * trans_dst_rotate;
    //    return true;
    //}
}

bool process_rotate_dropped2(CloudPtr cloud_tras, Matrix4f& trans_dst, CloudPtr cloud_temp_src
    , Matrix4f& mtTmp){
    float error2 = -1;
    Matrix4f trans_pair = Matrix4f::Identity();
    Matrix4f trans_rotate = Matrix4f::Identity();
    MatHelper::get_adjust_tras(g_status._rotate_count, trans_rotate);
    g_status._rotate_count++;
    cout << ".........................." << g_status._rotate_count << endl;
    printf("...........................%d\n", g_status._rotate_count);
    if (g_status._rotate_count > 60) {
        g_status._skip_frame++;
        return false;
    }

    //jake
    Matrix4f trans_dst_rotate = g_status._global_transform * trans_rotate * mtTmp;
    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);
    PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    //jake todo 检测旋转通过1的准确性
    error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras, trans_pair);

    cout << "error2: " << error2 << endl;
    //jake todo 按照距离计算
    //if (MatHelper::check_rot_valid(trans_pair) == false ||
    if (error2 > 0.15 + g_status._skip_frame / 20.0 / 4.0 || error2 < 0) {
		return process_rotate_dropped2(cloud_tras, trans_dst, cloud_temp_src, mtTmp);
    } else {
        g_status._skip_frame = 0;
        trans_dst_rotate = trans_pair * trans_dst_rotate;
        pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst_rotate);
        trans_dst = trans_dst_rotate;
        return true;
    }
}

// zhangyuzhi 20151020
bool process_loop_detection_calc_feature(CloudPtr cloud, int &total_points, float &distance_max, float &distance_avg, Vector3d &vec)
{
    if (cloud->size() == 0) {
        return false;
    }
    PointXYZ origin_point(0, 0, 0);
    distance_max = 0;
    distance_avg = 0;
    total_points = cloud->size();
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    for (int i = 0; i < cloud->size(); i++) {
        const CloudItem &point = cloud->points[i];
        float dis = pcl::euclideanDistance(point, origin_point);
        if (dis < 5.0) {
            total_points--;
            continue;
        }
        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
        // cout << point.z << endl;
        distance_avg += dis;
        distance_max = (dis > distance_max) ? dis : distance_max;
    }
    vec(0) = sum_x;
    vec(1) = sum_y;
    vec(2) = sum_z;
    distance_avg = distance_avg / cloud->size();
    return true;
}
// bool process_loop_detection_calc_feature(CloudPtr cloud, int &total_points, float &distance_max, float &distance_avg, vector<int> &angle_area)
// {
//     if (cloud->size() == 0) {
//         return false;
//     }
//     PointXYZ origin_point(0, 0, 0);
//     angle_area.clear();
//     angle_area.resize(36);
//     distance_max = 0;
//     distance_avg = 0;
//     total_points = cloud->size();
//     for (int i = 0; i < cloud->size(); i++) {
//         const CloudItem &point = cloud->points[i];
//         // float tmp = pcl::euclideanDistance(PointXYZ(0, 0, 0), point);
//         // if (tmp > 5.0) {
//         //     total_points--;
//         //     continue;
//         // }
//         float angle1 = (atan2(point.y, point.x) + M_PI) / (M_PI_2) * 90;
//         float angle2 = (atan2(point.y, point.z) + M_PI) / (M_PI_2) * 90;
//         float angle3 = (atan2(point.z, point.x) + M_PI) / (M_PI_2) * 90;
//         int int_angle1 = static_cast<int>(angle1);
//         int int_angle2 = static_cast<int>(angle2);
//         int int_angle3 = static_cast<int>(angle3);
//         // check boundries
//         if (int_angle2 < 0)
//             int_angle2 = 0;
//         else if (int_angle2 >= 360)
//             int_angle2 = 359;
// 
//         if (int_angle3 < 0)
//             int_angle3 = 0;
//         else if (int_angle3 >= 360)
//             int_angle3 = 359;
// 
//         if (int_angle1 < 0)
//             int_angle1 = 0;
//         else if (int_angle1 >= 360)
//             int_angle1 = 359;
//         angle_area[int_angle1 / 30 + 12 * 0]++;
//         angle_area[int_angle2 / 30 + 12 * 1]++;
//         angle_area[int_angle3 / 30 + 12 * 2]++;
//         float dis = pcl::euclideanDistance(point, origin_point);
//         distance_avg += dis;
//         distance_max = (dis > distance_max) ? dis : distance_max;
//     }
//     distance_avg = distance_avg / cloud->size();
//     return true;
// }

// zhangyuzhi 20151022
float process_loop_detection_calc_max_width_rate(const uint64_t cloud_index)
{
    // 还原点云
    CloudPtr cloud = g_status._clouds_line[cloud_index];
    const Matrix4f &trans_mat = g_status._optimize_rot[cloud_index];
    Matrix4f trans_mat_inv = trans_mat.inverse();
    CloudPtr cloud_new(new Cloud);
    pcl::transformPointCloud(*cloud, *cloud_new, trans_mat_inv);

    return PointCloudHelper::get_cloud_max_width_rate(*cloud_new, MAX_WIDTH_CORNER);
}

// zhangyuzhi 20151022
bool process_loop_detection_check(const pair<uint64_t, uint64_t> &loop_index_pair, const float icp_threshold, const float feature_threshold, const float max_width_max_threshold)
{
    vector<CloudPtr> &clouds_line = g_status._clouds_line;
    // int first = -1, last = -1;
    int first = loop_index_pair.first;
    int last = loop_index_pair.second;


    if (first > (clouds_line.size() - 1) ||
        last > (clouds_line.size() - 1)) {
        cout << "do_test loop idx unvalid ..." << loop_index_pair.first << "........."
            << loop_index_pair.second << endl;
        return false;
    }

    // first
    CloudPtr cloud_first(new Cloud);
    CloudPtr cloud_last(new Cloud);

    uint64_t stampFirst = clouds_line[first]->header.stamp;
    for (int j = 0; j < 40; j++) {
        if (j + first<0 || j + first>clouds_line.size() - 1)
            continue;
        *cloud_first += *clouds_line[j + first];
    }
    cloud_first->header.stamp = stampFirst;
    cout << "do_test add lines to cloudline first... " << first << endl;

    // last
    for (int j = -40; j < 0; j++) {
        if (last + j < 0 || last + j > clouds_line.size() - 1)
            continue;
        *cloud_last += *clouds_line[last + j];
    }
    cout << "do_test add lines to cloudline last... " << last << endl;

    Matrix4f icp_trans_mat = Matrix4f::Identity();
    // icp
    float icp_dis = PointCloudHelper::get_rot_icp(cloud_first, cloud_last, icp_trans_mat, true);

    Matrix4f first_rot = g_status._optimize_rot[loop_index_pair.first];
    Matrix4f last_rot = g_status._optimize_rot[loop_index_pair.second];
    first_rot.block<3, 3>(0, 0) = -Matrix3f::Identity();
    last_rot.block<3, 3>(0, 0)  = -Matrix3f::Identity();

    CloudPtr cloud_mo[2];
    for (int i = 0; i < 2; i++) {
        cloud_mo[i].reset(new Cloud);
    }

    pcl::transformPointCloud(*cloud_first, *cloud_mo[0], -first_rot);
    pcl::transformPointCloud(*cloud_last, *cloud_mo[1], -last_rot);
    // zhangyuzhi test 20151022
    static int current_index = 0;
    stringstream ss1, ss2;
    ss1 << g_status._pcap_file << "_loop_test_" << current_index << "_0.pcd";
    pcl::io::savePCDFileBinary(ss1.str(), *cloud_mo[0]);
    ss2 << g_status._pcap_file << "_loop_test_" << current_index << "_1.pcd";
    pcl::io::savePCDFileBinary(ss2.str(), *cloud_mo[1]);
    current_index++;
    // zhangyuzhi test end
    int total_points[2] = { 0 };
    float distance_max[2] = { 0 };
    float distance_avg[2] = { 0 };
    for (int i = 0; i < 2; i++) {
        PointCloudHelper::remove_duplicate(cloud_mo[i], 0.15);
    }
    // 20151026
    Matrix4f rot_feature_align = Matrix4f::Identity();
    float feature_align_ret = FeatureAlign::do_align_pre(cloud_mo[0], cloud_mo[1], rot_feature_align);

    // Vector3d angle[2];
    // for (int i = 0; i < 2; i++) {
    //     // 特征
    //     bool ret = process_loop_detection_calc_feature(cloud_mo[i], total_points[i], distance_max[i], distance_avg[i], angle[i]);
    //     if (!ret) {
    //         return false;
    //     }
    // }
    // 狭小走廊
    float max_width_rate1 = 0;
    float max_width_rate2 = 0;
    max_width_rate1 = process_loop_detection_calc_max_width_rate(loop_index_pair.first);
    max_width_rate2 = process_loop_detection_calc_max_width_rate(loop_index_pair.second);


    bool feature_ret, icp_ret, max_width_max_ret, final_ret;
    feature_ret = true;
    // float cos_theta = angle[0].dot(angle[1]) / angle[0].norm() / angle[1].norm();
    // // acos returns [0,pi] radians
    // feature_ret = acos(cos_theta) < (M_PI_2 * feature_threshold / 90.0);
    feature_ret = feature_align_ret > feature_threshold;

    icp_ret = icp_dis < icp_threshold;

    max_width_max_ret = max_width_rate1 <= max_width_max_threshold && max_width_rate2 <= max_width_max_threshold;

    final_ret = feature_ret && icp_ret && max_width_max_ret;

    // 记录统计数据
    vector<CloudPtr> &cloud_line = g_status._clouds_line;
    stringstream ss;
    ss << loop_index_pair.first << ' ' <<
        loop_index_pair.second << ' ' <<
        cloud_line[first]->header.stamp << ' ' <<
        cloud_line[last]->header.stamp << ' ';
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ss << icp_trans_mat(i, j) << ' ';
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ss << rot_feature_align(i, j) << ' ';
        }
    }
    // for (int i = 0; i < 2; i++) {
    //     ss << total_points[i] << ' '
    //         << distance_max[i] << ' '
    //         << distance_avg[i] << ' ';
    // }
    ss << icp_dis << ' '
        << feature_align_ret << ' '
        << max_width_rate1 << ' ' << max_width_rate2 << ' '
        << icp_ret << ' '
        << feature_ret << ' '
        << max_width_max_ret << ' ';
    ss << endl;
    string filename = g_status._pcap_file + "_loop_test.txt";
    ofstream ofile(filename.c_str(), ios::app);
    ofile << ss.str();
    ofile.close();

    return final_ret;
}

// zhangyuzhi 20151025
bool process_loop_detection(const vector<Matrix4f> &optimize_rot, pair<uint64_t, uint64_t> &loop_index_pair)
{
    if (optimize_rot.size() == 0) {
        return false;
    }
    ProcessLoopDetectionStatus &s           = g_status._loop_detection_status;
    PointCloud<PointXYZ>::Ptr track_cloud   = s.track_cloud;
    vector<size_t> &track_cloud_index       = s.track_cloud_index;
    uint64_t &last_loop_index_start         = s.last_timestamp_start;
    uint64_t &last_loop_index_end           = s.last_timestamp_end;
    uint64_t &search_last_timestamp         = s.search_last_timestamp;
    uint64_t &last_timestamp                = s.last_timestamp;
    uint64_t &min_cloudindex                = s.min_cloudindex;
    float    &search_last_distance          = s.search_last_distance;
    bool     &is_perform_search             = s.is_perform_search;
    int      &sleep_count                   = s.sleep_count;
    float icp_threshold = 10.0f;
    float feature_threshold = 0.30f;
    float max_width_max_threshold = 0.8f;

    if (sleep_count > 0) {
        sleep_count--;
        return false;
    }
    sleep_count = 0;
    const uint64_t i = optimize_rot.size() - 1;
    const uint64_t current_index = i;
    // 当前轨迹点
    const Matrix4f &trans_mat = optimize_rot[i];
    const PointXYZ this_point(trans_mat(0, 3), trans_mat(1, 3), trans_mat(2, 3));

    // 每10帧作为一个关键帧
    if (i % 10 != 0) {
        return false;
    }

    track_cloud->push_back(this_point);
    track_cloud_index.push_back(i);
    // 利用KD树搜索最邻近点
    KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud(track_cloud);
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(this_point, 5.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    float min_distance = 10000.0f;
    int64_t min_index = -1;
    bool check_ret = false;
    // uint64_t min_cloudindex = 0;
    for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
        const PointXYZ& pt_tmp = (*track_cloud)[pointIdxRadiusSearch[j]];
        float distance = pointRadiusSquaredDistance[j];
        // uint64_t this_index = 10 * pointIdxRadiusSearch[j];
        uint64_t this_index = track_cloud_index[pointIdxRadiusSearch[j]];
        uint64_t abs_index =
            (current_index > this_index) ?
            (current_index - this_index) :
            (this_index - current_index);
        uint64_t timestamp_tmp_start = 
            (min_cloudindex > last_loop_index_start) ? 
            (min_cloudindex - last_loop_index_start) : 
            (last_loop_index_start - min_cloudindex);
        uint64_t timestamp_tmp_end = 
            (current_index > last_loop_index_end) ? 
            (current_index - last_loop_index_end) : 
            (last_loop_index_end - current_index);
        // 回环起止时间点必须大于 5min
        if (abs_index > 5 * 60 * 10
            && (timestamp_tmp_start >= 50 * 10 || timestamp_tmp_end >= 50 * 10)
            && min_distance > distance) {
            min_distance = distance;
            min_index = j;
            min_cloudindex = this_index;
        }
    }
    if (min_index != -1) {
        if (is_perform_search) {
            uint64_t abs_tmp_stamp =
                (search_last_timestamp > min_cloudindex) ?
                (search_last_timestamp - min_cloudindex) :
                (min_cloudindex - search_last_timestamp);
            if (abs_tmp_stamp >= 50 * 10 || search_last_distance < min_distance) {
                loop_index_pair.first = search_last_timestamp;
                loop_index_pair.second = last_timestamp;

                // zhangyuzhi 20151022
                check_ret = process_loop_detection_check(loop_index_pair, icp_threshold, feature_threshold, max_width_max_threshold);
                if (check_ret) {
                    last_loop_index_start   = loop_index_pair.first;
                    last_loop_index_end     = loop_index_pair.second;
                    is_perform_search       = false;
                    if (search_last_distance < min_distance) {
                        return true;
                    }
                } else { // check loop false
                    // sleep_count = 30;
                    is_perform_search = false;
                }
            } else { // continue to search
                search_last_distance = min_distance;
                search_last_timestamp = min_cloudindex;
                is_perform_search = true;
            }
        } else { // start to search
            search_last_distance = min_distance;
            search_last_timestamp = min_cloudindex;
            is_perform_search = true;
        }
    } else if (is_perform_search) {
        loop_index_pair.first = search_last_timestamp;
        loop_index_pair.second = last_timestamp;

        // zhangyuzhi 20151022
        check_ret = process_loop_detection_check(loop_index_pair, icp_threshold, feature_threshold, max_width_max_threshold);
        if (check_ret) {
            last_loop_index_start = loop_index_pair.first;
            last_loop_index_end = loop_index_pair.second;
            is_perform_search = false;
        } else {
            // sleep_count = 30;
            is_perform_search = false;
        }
    }
    last_timestamp = current_index;
    return check_ret;
}

// zhangyuzhi 20151028
// bool is_last_frame 参数用来区分是不是最后一帧，
// 并对最后一个正在搜索的回环进行处理，
// 再on_get_hdl_cloud_for_reg中总是为false
void process_loop(CloudPtr cloud_tras, bool is_last_frame)
{
    pair<uint64_t, uint64_t> loop_stamp_pair;
    pair<uint64_t, uint64_t> loop_index_pair;
    vector<pair<pair<uint64_t, uint64_t>, Matrix4f> > map_loops;
    if (is_last_frame && g_status._loop_detection_status.is_perform_search) {
        loop_index_pair.first = g_status._loop_detection_status.search_last_timestamp;
        loop_index_pair.second = g_status._clouds_line.size() - 1;
        bool loop_check_ret = process_loop_detection_check(loop_index_pair, 10.0f, 0.30f, 0.8f);
        if (!loop_check_ret) {
            return;
        }
    } else {
        CloudPtr cloud_temp(new Cloud());
        pcl::copyPointCloud(*cloud_tras, *cloud_temp);
        g_status._clouds_line.push_back(cloud_temp);
        g_status._optimize_rot.push_back(g_status._global_transform);
        std::cout << "clouds_line size: " << g_status._clouds_line.size() << std::endl;
        std::cout << "optimize_rot size: " << g_status._optimize_rot.size() << std::endl;

        // zhangyuzhi 20151211
        // 忽略回环优化时间戳区间
        uint64_t current_timestamp = cloud_temp->header.stamp / 1000;
        for (int i = 0; i < g_status._map_ignore_loops.size(); i++) {
            uint64_t start_timestamp = g_status._map_ignore_loops[i].first;
            uint64_t end_timestamp = g_status._map_ignore_loops[i].second;
            if (current_timestamp >= start_timestamp && current_timestamp <= end_timestamp) {
                cout << "ignore loop, skip" << endl;
                return;
            }
        }

        // zhangyuzhi 20151020
        // loop 检测
        bool loop_detection_ret = process_loop_detection(g_status._optimize_rot, loop_index_pair);
        if (!loop_detection_ret) {
            return;
        }
        // 将index转为时间戳
    }
    loop_stamp_pair.first = g_status._clouds_line[loop_index_pair.first]->header.stamp / 1000;
    loop_stamp_pair.second = g_status._clouds_line[loop_index_pair.second]->header.stamp / 1000;
    map_loops.push_back(make_pair(loop_stamp_pair, Matrix4f::Identity()));

    //loop 优化
    if (map_loops.size() > 0) {
		//zhangyuzhi
        static int loop_index = 0;
        static int clouds_line_last_size = 0;
		uint64_t last_end = map_loops[0].first.second;
		if (last_end <= cloud_tras->header.stamp / 1000) {
			PointCloudHelper::do_lum_elch(
                map_loops, 
                g_status._clouds_line, 
                g_status._optimize_rot, 
                g_status._pcap_file, 
                loop_index, 
                clouds_line_last_size);
            clouds_line_last_size = g_status._clouds_line.size();
            loop_index++;

            // zhangyuzhi 20151027
            // 更新回环检测全局变量
            ProcessLoopDetectionStatus &s = g_status._loop_detection_status;
            vector<size_t> &track_cloud_index = s.track_cloud_index;
            PointCloud<PointXYZ>::Ptr &track_cloud = s.track_cloud;
            track_cloud->clear();
            for (int i = 0; i < track_cloud_index.size(); i++) {
                const Matrix4f &rot = g_status._optimize_rot[track_cloud_index[i]];
                PointXYZ p(rot(0, 3), rot(1, 3), rot(2, 3));
                track_cloud->push_back(p);
            }

			g_status._start_frame = 0;
			g_move_model_after.clear();
			g_status._cloud_result->clear();
			// int num_re_icp = 100;

			namespace bf = boost::filesystem;
			bf::path path_rot(g_status._pcap_file + "_pos.txt");
			bf::path path_rot_old(g_status._pcap_file + "_pos_loop_src_" +
				StringHelper::get_time_string() + ".txt");
			bf::copy_file(path_rot, path_rot_old);

            set<uint64_t>::iterator end_it = g_status._ignore_invalid_stamp.end();
			for (int i = 0; i < g_status._clouds_line.size(); i++) {
				CloudPtr item = g_status._clouds_line[i];
                if (g_status._ignore_invalid_stamp.find(item->header.stamp) != end_it) {
                    continue;
                }
                FileHelper::write_file_rot(i, g_status._pcap_file,
                    g_status._optimize_rot[i], i == 0 ? false : true,
                    g_status._clouds_line[i]->header.stamp / 1000);
                *g_status._cloud_result += *item;
                if (i % 50 == 0) {
                    PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.3);
                }
			}

			PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.15f);
			PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
			if (g_status._current_mode == ENUM_MODE_PROCESS_POS) {
				//恢复cache
				g_status._global_transform = g_status._optimize_rot[g_status._optimize_rot.size(
					) - 1];
				CloudGrid::instance().clear();
                CloudGrid::instance().qframe_clear();
                PointCloudHelper::clear_trimesh_cache();
				CloudGrid::instance().add_cloud(g_status._cloud_result);
				CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
					g_status._global_transform);
				//剩余的rot列表作废
				g_status._map_clouds_rot_stamps.clear();
			}
		}
    }
}

bool process_icp(const CloudPtr cloud_temp_imu, Matrix4f mtTmp, Matrix4f& trans_pair,
    Matrix4f& trans_dst, const CloudPtr cloud_temp_src, CloudPtr cloud_tras,
    const CloudIConstPtr& cloud, CloudPtr cloud_temp_icp, bool is_rotate, float max_dis){
	//float max_width = PointCloudHelper::get_cloud_max_width(*cloud_temp_imu);
	float max_width=g_status._max_dis;
	cout << "max_width " << max_width << endl;
	
	float error1 = PointCloudHelper::get_rot_icp(g_status._cloud_src, cloud_temp_imu,
            trans_pair);
    //jake 确保顺序
    trans_dst = g_status._global_transform * trans_pair * mtTmp;
    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst);
	////直接利用imu
	//float error1 = 0.149;
	//trans_pair=Matrix4f::Identity();
	//trans_dst=mtTmp;
	//trans_dst(0,3)=g_status._global_transform(0,3);
	//trans_dst(1,3)=g_status._global_transform(1,3);
	//trans_dst(2,3)=g_status._global_transform(2,3);
 //   pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst);
	//PclPairVisualizer::instance().update_cloud_left(cloud_tras);
    
	cout << "g_status._cloud_cache size: " << g_status._cloud_cache->points.size()
        << endl;

    float error2 = -1;
	//jake 100帧之内启用缓冲
	if (g_status._frame_count_process <100)
	{
		PointCloudHelper::clear_trimesh_cache();
	}
	error2 = PointCloudHelper::get_rot_icp_cache(g_status._cloud_cache, cloud_tras,
            trans_pair);
    cout << "error2:" << error2 << " error1:" << error1 << endl;

	//CloudPtr temp(new Cloud());
	//pcl::transformPointCloud(*cloud_tras, *g_status._cloud_cache, trans_pair);
	//*temp = *g_status._cloud_cache + *temp;
	//PclPairVisualizer::instance().update_cloud_left(temp);;

    //if (is_rotate)
    //{ 
    //jake todo 5帧内
    //if (error2 > 0.15 + g_status._skip_frame / 20.0 / 4.0 || error2 < 0) {
	Matrix4f rot_mt=trans_pair * trans_dst *mtTmp.inverse();
	float pitch = asin(-rot_mt(2,0))/M_PI *180 ; 
	//cout << "pitch " << pitch << ((abs(pitch) > 30)?" fail... ":"") <<endl;
	cout << "pitch " << pitch <<endl;
    if (error2 > 0.15 + g_status._skip_frame * 0.03 || error2 < 0/* || abs(pitch) > 30*/) {
        if (g_status._queue_cloud_tras.size() > 0) {
            CloudPtr cloud_cache_tras_temp = g_status._queue_cloud_tras.front();
            g_status._queue_cloud_tras.pop();
            CloudGrid::instance().add_cloud(cloud_cache_tras_temp);
            CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
                g_status._global_transform);
            process_house_back(cloud->header.stamp / 1000);
            return process_icp(cloud_temp_imu, mtTmp, trans_pair, trans_dst,
                cloud_temp_src, cloud_tras, cloud, cloud_temp_icp, is_rotate, max_dis);
        } else {
            PointCloudHelper::clear_trimesh_cache();
            if (cloud_temp_icp->size() == 0) {
                pcl::copyPointCloud(*cloud_tras, *cloud_temp_icp);
                g_status._rotate_count = 1;
            }
            if (error1 < 0.15 && error2 < 0.25) {
                cout << "error1 ..............error1 " << error1 << " error2 "
                    << error2 << endl;
                return true;
                /*if (error2<0.7)
                {
                return true;
                }
                else
                {
                PointCloudHelper::get_rot_icp(g_status._cloud_src, cloud_temp_imu,
                trans_pair);
                trans_dst = g_status._global_transform * trans_pair * mtTmp;
                pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst);
                return true;
                }*/
                //jake todo 
                //trans_pair = Matrix4f::Identity();
                //pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst);
                //return true;
            }
			//else if (error1 < 0.5 && max_width < MAX_WIDTH_CORNER)
   //         {
			//	//jake 20150813 狭小走廊,增大兼容性
   //             cout << "error1 ..............error1 conner " << error1 << " error2 "
   //                 << error2 << endl;
   //             return true;
   //         }
			else{
				//jake 现在依赖imu,不应当出现rotate的情况
				  //cout << "rotate fail..............." << endl;
				  //return false;
				  //jake 如果error1 error2都比较大,则利用之前的
				if (max_width < MAX_WIDTH_CORNER)
				{
					cout << "error ........... conner fail rotate" << endl;
				}
				trans_dst = g_status._global_transform * mtTmp;
				pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_dst);
				pcl::copyPointCloud(*cloud_tras, *cloud_temp_icp);
				error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloud_tras,
					  trans_pair);
				cout << "second error2:" << error2 << " error1:" << error1 << endl;
				//if (error2 > 0.15 + g_status._skip_frame / 20.0 / 4.0 || error2 < 0) {
				if (error2 > 0.15 + g_status._skip_frame * 0.03 || error2 < 0) {
					//if (!is_rotate) {
					//    cout << "rotate ignore..............." << endl;
					//    g_status._skip_frame++;
					//    return false;
					//}
				} else {
					return true;
				}

				bool flag_rotate = process_rotate(cloud_tras, trans_dst, cloud_temp_src, mtTmp);
				if (flag_rotate == true) {
					trans_pair = Matrix4f::Identity();
				} else {
					cout << "rotate fail..............." << endl;
					return false;
				}
			}
        }
    }
    //}else
    //{
    //if ((error2 > (0.15 + g_status._skip_frame *0.01)) || (error2 < 0)) {
    // if (g_status._queue_cloud_tras.size() > 0) {
    //  CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
    //  g_status._queue_cloud_tras.pop();
    //  //GetReducedCloud(gCloudCache, true);
    //  CloudGrid::instance().add_cloud(cloudCacheTrasTemp);
    //  CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
    //   g_status._global_transform);
    //  if (g_status._house_back_stamp.size() > 0 &&
    //   g_status._house_back_stamp.front() >= cloud->header.stamp / 1000) {
    //   CloudGrid::instance().add_cloud(g_status._cloud_cache);
    //   g_status._house_back_stamp.pop();
    //  }
    //  return process_icp(cloud_temp_imu, mtTmp, trans_pair, trans_dst, cloud_temp_src, cloud_tras,
    //   cloud, cloud_temp_icp,is_rotate);
    // } else {   
    //  if (g_status._skip_frame<5)
    //  {
    //   g_status._skip_frame++;
    //  }
    //           if (error2>3)
    //           {
    //               //jake 直接跳过
    //               cout << "ignore frame icp................"<<error2<<endl;
    //               return false;
    //           }else
    //           {
    //      return true;
    //           }
    // }
    //}else
    //{
    // g_status._skip_frame=0;
    //}
    //}
    return true;
}

void on_get_hdl_cloud_for_reg(const CloudIConstPtr& cloud){
    CloudPtr cloud_temp_imu(new Cloud);
    CloudPtr cloud_temp_icp(new Cloud);
    CloudPtr cloud_temp_src(new Cloud);
    CloudPtr cloud_tras(new Cloud);
    double d_yaw = 0.0;
    double d_pitch = 0.0;
    double d_roll = 0.0;
    g_status._frame_count_total++;

	if (cloud->size() <= 10)
	{
		cout << ".................点云数量太少 " <<
			cloud->header.stamp << " " << cloud->size() << endl;
		fstream outfile;
		string filePath = g_status._pcap_file + "_mt_error.txt";
		outfile.open(filePath, ios::app);
		outfile << cloud->header.stamp << " " << cloud->size() << " 点云数量太少" << endl;
		outfile.close();
		return;
	}
	if (check_ignore_frame_valid(cloud->header.stamp / 1000, g_status._ignore_stamp) == false) {
		cout << "ignore frame ........." << cloud->header.stamp / 1000 << endl;
		return;
	}
    //imu
	Matrix4f mt_tmp = Matrix4f::Identity();
	uint64_t stamp =  cloud->header.stamp;
	//static int test_idx=0;
	//test_idx++;
	//if (test_idx==1 )
	//{
	//	stamp = 2497375000;
	//}else{
	//	stamp = 2497475000;
	//}
    bool flag = g_imu_trans->get_transaction_mx_mcs(cloud->header.stamp, mt_tmp, d_yaw,
        d_pitch, d_roll);

    float max_dis = PointCloudHelper::cloudi2cloud(cloud, cloud_temp_src, 1, false, LOOPOUTSIDE,
        get_glass_min_ins(cloud->header.stamp / 1000, g_status._glass_stamp));
	//pcl::copyPointCloud(*cloud_temp_src ,*g_status._cloud_temp_src_last);


    cout << "-------------------------------------------------" << endl;
    cout << "stamp " << cloud->header.stamp / 1000 << " " << d_yaw <<" max_dis "<< max_dis << endl;
    cout << "cloudi2cloud" << " src " << cloud->points.size() << " dst "
        << cloud_temp_src->points.size() << endl;
	//允许排除特定的走廊,根据ignore起始点排除
	for (map< uint64_t, uint64_t>::iterator it = g_status._ignore_stamp.begin(); it != g_status._ignore_stamp.end();
		it++) {
		if (abs(double(cloud->header.stamp / 1000 - it->second)) < 500) {
			d_yaw = 1.111111;
			cout << "ignore end allow d_yaw " << d_yaw << endl;
			break;
		}
	}
	//取消5米限制
	float rate = PointCloudHelper::get_cloud_max_width_rate(*cloud_temp_src,MAX_WIDTH_CORNER);	
	//rate = 0.1;
	if(rate>0.8)
		max_dis = MAX_WIDTH_CORNER - 1;
	g_status._max_dis = max_dis;
	//距离小的加强密度权重

	//if (max_dis < MAX_WIDTH_CORNER)
	//{
	//	PointCloudHelper::remove_duplicate(cloud_temp_src, 0.05);
	//}else{
		PointCloudHelper::remove_duplicate(cloud_temp_src, 0.13);
	//}
	PointCloudHelper::remove_outlier(cloud_temp_src, 5000);
	PointCloudHelper::add_trace_point(cloud_temp_src);

    //#ifdef _DEBUG
/*	if (flag) {
		mt_tmp = mt_tmp * g_status._last_mt;
		g_status._last_mt = mt_tmp;
		pcl::transformPointCloud(*cloud_temp_src, *cloud_temp_imu, mt_tmp);
		cout << "Imu transform:" << endl;
		cout << mt_tmp << endl;
	}
	PclPairVisualizer::instance().update_cloud_left(cloud_temp_imu);
	return;*/
    //#endif

    //#endif
    //PclPairVisualizer::instance().update_cloud_left(cloud_temp_src);
    //PclPairVisualizer::instance().spin();
    //for (int i=0;i<60;i++)
    //{
    //    Matrix4f trans_rotate = Matrix4f::Identity();
    //    MatHelper::get_adjust_tras(i, trans_rotate);
    //    pcl::transformPointCloud(*cloud_temp_src, *cloud_tras, trans_rotate);
    //    PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    //    PclPairVisualizer::instance().spin();
    //}


    //if (cloud->size()<10000)
    //{
    //    return;
    //}
    //Matrix4f mat_rot=Matrix4f::Identity();
    //cv::Point3d pt_gradient;
    ////_last_ori = pt_valid;
    //pt_gradient.x = d_yaw;
    //pt_gradient.y = d_pitch;
    //pt_gradient.z = d_roll;
    ////根据点云安装和imu的位置坐标轴差异,将imu的各角度还原到点云坐标系下
    //Eigen::AngleAxisf angle_yaw(DEG2RAD(pt_gradient.x), Eigen::Vector3f::UnitZ());
    //Eigen::AngleAxisf angle_pitch(DEG2RAD(pt_gradient.y), Eigen::Vector3f::UnitY());
    //Eigen::AngleAxisf angle_roll(DEG2RAD(pt_gradient.z), Eigen::Vector3f::UnitX());
    //Eigen::Quaternion< float> qt = angle_roll* angle_pitch*angle_yaw;
    //mat_rot.block(0, 0, 3, 3) = qt.matrix();
    //    Matrix4f mat_rot2=mat_rot.inverse();
    //    mat_rot=mat_rot2;
    //    pcl::transformPointCloud(*cloud_temp_src, *cloud_temp_imu, mat_rot);
    //    PclPairVisualizer::instance().update_cloud_left(cloud_temp_imu);
    //    PclPairVisualizer::instance().spin();
    //return;
  
    //恢复历史记录
    if (g_status._start_frame > 0) {
        Matrix4f rot_old;
        if (MatHelper::get_rot_with_stamp(g_status._map_clouds_rot_stamps,
            cloud->header.stamp / 1000, rot_old)) {
            cout << "use old rot " << cloud->header.stamp / 1000 << endl;
            g_status._global_transform = rot_old;
            if (PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot_stamps,
                cloud->header.stamp / 1000, cloud_temp_src, cloud_tras,
                g_status._global_transform)) {
                //g_status._map_clouds_rot_stamps.erase(cloud->header.stamp / 1000);
            }
            CloudGrid::instance().add_cloud(cloud_tras);
            CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
                g_status._global_transform);
            g_status._frame_count_process++;
            int num_per_show = 10;
            if (g_status._is_debug_mode)
            {
                num_per_show = 1;
            }else {
                if (g_status._is_batch_mode == true)
                {
                    num_per_show = 50;
                }
            }
            if (g_status._frame_count_process % num_per_show == 0)
            {
                PclPairVisualizer::instance().update_cloud_right(cloud_tras);
            }
            process_end(cloud, cloud_temp_src, cloud_tras, true);
            process_loop(cloud_tras, false);
            return;
        }
        if (cloud->header.stamp / 1000 < g_status._start_frame) {
            return;
        }
    }

    //mt
    if (flag) {
        mt_tmp = mt_tmp * g_status._last_mt;
        g_status._last_mt = mt_tmp;
        pcl::transformPointCloud(*cloud_temp_src, *cloud_temp_imu, mt_tmp);
    } else {
        g_status._last_mt = Matrix4f::Identity();
        pcl::copyPointCloud(*cloud_temp_src, *cloud_temp_imu);
        cout << g_status._frame_count_total
            << " not find correspond imu stamp..................."
            << cloud_temp_src->header.stamp / 1000 << endl;
        printf("%d not find correspond imu stamp...................%llu\n",
            g_status._frame_count_total, cloud_temp_src->header.stamp / 1000);
    }
    g_status._frame_count_process++;
    cout << "process count " << g_status._frame_count_process << "   count "
        << cloud_temp_src->points.size() << endl;

    //1st frame
    if (g_status._cloud_src->points.size() == 0) {
        pcl::copyPointCloud(*cloud_temp_src, *g_status._cloud_src);
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_src);
		//1st frame 从0开始
		mt_tmp = Matrix4f::Identity();
		pcl::transformPointCloud(*cloud_temp_src, *cloud_temp_imu, mt_tmp);
		g_status._global_transform = mt_tmp;
		//g_status._global_transform = Matrix4f::Identity(); 
        //imu
        //Matrix4f mtTmp = Matrix4f::Identity();
        //gImuTrans->GetTransactionMx(gCloudSrc->header.stamp / 1000, mtTmp, dYaw, dPitch, dRoll);
        //gMoveModel.UpdateModelLocal(mtTmp);
        //cout << "Imu transform:" << endl;
        //cout << mtTmp << endl;d

		CloudGrid::instance().add_cloud(cloud_temp_imu);
		CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
			g_status._global_transform);
		g_status._frame_count_process++;
		process_end(cloud, cloud_temp_src, cloud_temp_imu, true);
		process_loop(cloud_temp_imu, false);
        return;
    }

    //mat 
    Matrix4f trans_pair = Matrix4f::Identity();
    Matrix4f trans_dst = Matrix4f::Identity();
    ////jake todo 
    //PointCloudHelper::fix_cloud_cache(*g_status._cloud_cache,*cloud_temp_imu,
    //    g_status._global_transform,*g_status._cloud_cache);

	CloudPtr cloud_temp_src_short(new Cloud());
	pcl::copyPointCloud(*cloud_temp_src,*cloud_temp_src_short);
	//PointCloudHelper::reduce_cloud(*cloud_temp_src_short,CloudGrid::instance().MAX_DIS);
	bool is_first_inside_short = false;
	//jake test
	if(g_status._max_dis < 5)
	{
		////太小的加强距离权重
		//CloudPtr last_cloud(new Cloud());
		//while (g_status._queue_cloud_tras.size() > 0) {
		//	CloudPtr cloud_cache_tras_temp = g_status._queue_cloud_tras.front();
		//	g_status._queue_cloud_tras.pop();
		//	CloudGrid::instance().add_cloud(cloud_cache_tras_temp);
		//	last_cloud = cloud_cache_tras_temp;
		//}
		//CloudGrid::instance().get_cloud_with_pos_short(g_status._cloud_cache,
		//	g_status._global_transform, g_status._num_short);
		//PointCloudHelper::remove_duplicate(g_status._cloud_cache, 0.05);
		////第一帧进入
		//if (g_status._num_short == 0)
		//{
		//	*g_status._cloud_cache += *last_cloud;
		//	is_first_inside_short = true;
		//	PointCloudHelper::remove_duplicate(cloud_temp_src_short, 0.13);
		//}
		//cout << "inside short 0.05" << endl;
		//cout << g_status._num_short << endl;
		//g_status._num_short++;


		//直接使用最近100帧计算
		CloudPtr last_cloud(new Cloud());
		while (g_status._queue_cloud_tras.size() > 0) {
			  CloudPtr cloud_cache_tras_temp = g_status._queue_cloud_tras.front();
			  g_status._queue_cloud_tras.pop();
			  CloudGrid::instance().add_cloud(cloud_cache_tras_temp);
		}
		PointCloudHelper::clear_trimesh_cache();
		g_status._num_short++;
		CloudGrid::instance().get_cloud_with_pos_short(g_status._cloud_cache,
			g_status._global_transform,g_status._num_short);
		PointCloudHelper::remove_duplicate(g_status._cloud_cache, 0.15);
		cout << "inside short 100 0.05" << endl;
		cout << g_status._num_short << endl;
		g_status._num_short++;;
	}
	else
	{
		//太小的加强距离权重
		if (g_status._num_short > 0)
		{
			CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
				g_status._global_transform);
			PointCloudHelper::clear_trimesh_cache();
			cout << "outside short" << endl;
		}
		g_status._num_short = 0;
	}

    flag = process_icp(cloud_temp_imu, mt_tmp, trans_pair, trans_dst, cloud_temp_src_short,
        cloud_tras, cloud, cloud_temp_icp,
        MatHelper::check_rot_valid(cloud->header.stamp / 1000, g_status._frame_count_process,
        d_yaw), max_dis);
    if (false == flag) {
        // zhangyuzhi 
        // 记录拼合时跳过写入文件的帧
        g_status._ignore_invalid_stamp.insert(cloud->header.stamp);
        return;
    }
    process_update_rot(trans_pair, trans_dst, cloud_temp_src, cloud_tras, cloud, d_yaw, max_dis);
    process_loop(cloud_tras, false);
}

void on_get_hdl_cloud_for_loop(const CloudIConstPtr& cloud){
    CloudPtr cloud_temp(new Cloud);
    CloudPtr cloud_tras(new Cloud);
    g_status._frame_count_total++;

	if (cloud->size() <= 10)
	{
		cout << ".................点云数量太少 " <<
			cloud->header.stamp << " " << cloud->size() << endl;
		fstream outfile;
		string filePath = g_status._pcap_file + "_mt_error.txt";
		outfile.open(filePath, ios::app);
		outfile << cloud->header.stamp << " " << cloud->size() << " 点云数量太少" << endl;
		outfile.close();
		return;
	}
    //imu
    double d_yaw = 0.0;
    double d_pitch = 0.0;
    double d_roll = 0.0;
    Matrix4f mt_tmp = Matrix4f::Identity();
    bool flag = g_imu_trans->get_transaction_mx_mcs(cloud->header.stamp, mt_tmp, d_yaw,
        d_pitch, d_roll);
    if (MatHelper::check_yaw_valid_limit(d_yaw) == false)
    {
        cout << "ignore frame yaw ........." << cloud->header.stamp / 1000 << endl;
        return;
    }

    if (check_ignore_frame_valid(cloud->header.stamp / 1000, g_status._ignore_stamp) == false) {
        cout << "ignore frame ........." << cloud->header.stamp / 1000 << endl;
        return;
    }

    PointCloudHelper::cloudi2cloud(cloud, cloud_temp, 11, false, 40,
        get_glass_min_ins(cloud->header.stamp / 1000, g_status._glass_stamp));
    PointCloudHelper::remove_duplicate(cloud_temp, 0.1);
    PointCloudHelper::remove_outlier(cloud_temp, -1);
    PointCloudHelper::add_trace_point(cloud_temp);


    Matrix4f rot;
    if (g_status._map_clouds_rot_stamps.size() != 0) {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot_stamps,
            cloud->header.stamp / 1000, cloud_temp, cloud_tras, rot)) {
            return;
        }
        //jake todo 有些时候,有几帧旋转过大,导致不能正确画出投影图
        //if (abs(rot(2, 2) - 1) > 0.1)
        //{
        //    cout << "ignore frame pitch roll ........." << cloud->header.stamp / 1000 << endl;
        //    return;
        //}
    } else {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot,
              g_status._frame_count_total, cloud_temp, cloud_tras, rot)) {
            return;
        }
    }

    g_status._optimize_rot.push_back(rot);
    //if (g_status._cloud_result->size() == 0) {
    //    pcl::copyPointCloud(*cloud_tras, *g_status._cloud_result);
    //    PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    //}
    if (g_status._is_batch_mode == false) {
        *g_status._cloud_result = *g_status._cloud_result + *cloud_tras;
        PointCloudHelper::remove_duplicate(g_status._cloud_result, RDUMPLICATE_VALUE);
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
        PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    }

    cloud_tras->header.stamp = cloud->header.stamp;
    cloud_tras->header.seq = g_status._frame_count_total;

    g_status._clouds_line.push_back(cloud_tras);
    cout << g_status._frame_count_total << ":" << g_status._cloud_result->size() << endl;

    //cout << rot << endl;

#if AUTOMATIC_LOOP_DETECTION_WHILE_BUILD_SCENERY
    // zhangyuzhi 20151020
    // loop 检测
    pair<uint64_t, uint64_t> loop_stamp_pair;
    pair<uint64_t, uint64_t> loop_index_pair;
    bool ret = process_loop_detection(g_status._optimize_rot, loop_index_pair);
    // process_loop_detection2(g_status._optimize_rot);
    if (!ret) {
        return;
    }
    loop_stamp_pair.first = g_status._clouds_line[loop_index_pair.first]->header.stamp / 1000;
    loop_stamp_pair.second = g_status._clouds_line[loop_index_pair.second]->header.stamp / 1000;
    g_status._map_loops.push_back(make_pair(loop_stamp_pair, Matrix4f::Identity()));
#endif

}

void on_get_hdl_cloud_for_show(const CloudIConstPtr& cloud){
    CloudPtr cloud_temp(new Cloud);
    CloudPtr cloud_tras(new Cloud);
    g_status._frame_count_total++;

	if (cloud->size() <= 10)
	{
		cout << ".................点云数量太少 " <<
			cloud->header.stamp << " " << cloud->size() << endl;
		fstream outfile;
		string filePath = g_status._pcap_file + "_mt_error.txt";
		outfile.open(filePath, ios::app);
		outfile << cloud->header.stamp << " " << cloud->size() << " 点云数量太少" << endl;
		outfile.close();
		return;
	}
    //imu
    double d_yaw = 0.0;
    double d_pitch = 0.0;
    double d_roll = 0.0;
    Matrix4f mt_tmp = Matrix4f::Identity();
    bool flag = g_imu_trans->get_transaction_mx_mcs(cloud->header.stamp, mt_tmp, d_yaw,
        d_pitch, d_roll);
    if (MatHelper::check_yaw_valid_limit(d_yaw) == false)
    {
        cout << "ignore frame yaw ........." << cloud->header.stamp / 1000 << endl;
        return;
    }

    if (check_ignore_frame_valid(cloud->header.stamp / 1000, g_status._ignore_stamp) == false) {
        cout << "ignore frame ........." << cloud->header.stamp / 1000 << endl;
        return;
    }

    PointCloudHelper::cloudi2cloud(cloud, cloud_temp, 3, false, 40,
        get_glass_min_ins(cloud->header.stamp / 1000, g_status._glass_stamp));
    PointCloudHelper::remove_duplicate(cloud_temp, 0.1);
    PointCloudHelper::remove_outlier(cloud_temp, -1);
    //for (int i=0;i<25;i++)
    //{
    //    PointCloudHelper::add_trace_point(cloud_temp,255,255,255);
    //}

	
	for (int i=0;i<25;i++)
	{
		PointCloudHelper::add_trace_point(cloud_temp,8,196,8);
	}

    Matrix4f rot;
    if (g_status._map_clouds_rot_stamps.size() != 0) {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot_stamps,
            cloud->header.stamp / 1000, cloud_temp, cloud_tras, rot)) {
            return;
        }
    } else {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot,
              g_status._frame_count_total, cloud_temp, cloud_tras, rot)) {
            return;
        }
    }

    g_status._optimize_rot.push_back(rot);
    if (g_status._is_batch_mode == false) {
        *g_status._cloud_result = *g_status._cloud_result + *cloud_tras;
        PointCloudHelper::remove_duplicate(g_status._cloud_result, RDUMPLICATE_VALUE);
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
        PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    }

    cloud_tras->header.stamp = cloud->header.stamp;
    cloud_tras->header.seq = g_status._frame_count_total;

    //g_status._clouds_line.push_back(cloud_tras);
    CloudDensity::instance().add_cloud(cloud_tras);
    cout << g_status._frame_count_total << ":" << g_status._cloud_result->size() << endl;
    g_status._frame_count_process++;
    CloudPtr cloud_show(new Cloud());
    CloudDensity::instance().get_cloud_hit_num(cloud_show,10);


	Matrix4f d_rot = rot * g_status._last_rot.inverse();
	double dis = pow(d_rot(0,1),2) +  pow(d_rot(0,2),2) +  pow(d_rot(0,3),2); 
	if (sqrt(dis)>0.05)
	{
		d_rot = Eigen::Matrix4f::Identity();
		//d_rot.block<3,1>(3,0) = rot.inverse().block<3,1>(3,0);
		//d_rot =  g_status._global_transform * d_rot.inverse(); 
		g_status._global_transform=rot.inverse();
	}
	g_status._last_rot=rot;


	PointCloudHelper::cloudi2cloud(cloud, cloud_temp, 3, false, 15,
		get_glass_min_ins(cloud->header.stamp / 1000, g_status._glass_stamp));
	pcl::transformPointCloud(*cloud_temp,*cloud_tras,rot);
	PointCloudHelper::change_cloud_rgb(cloud_tras,255,0,0);
	*cloud_show += *cloud_tras;
	//pcl::transformPointCloud(*cloud_show,*cloud_show,g_status._global_transform);
	PclPairVisualizer::instance().update_cloud_left(cloud_show);

}

void on_get_hdl_cloud_for_loop_trace(CloudPtr& cloud_temp){
    CloudPtr cloud_tras(new Cloud);
    g_status._frame_count_total++;

    Matrix4f rot;
    //if (gMapCloudsRot.find(gIgnoreFrameCount) == gMapCloudsRot.end())
    //{
    //  cout << "frame rot not found :" << gIgnoreFrameCount;
    //  return;
    //}
    //else{
    //  Matrix4f pairTransform = (gMapCloudsRot.find(gIgnoreFrameCount))->second;
    //  rot = pairTransform;
    //  cout << "final rot matrix:" << endl << pairTransform << endl << endl;
    //  pcl::transformPointCloud(*cloudTemp, *cloud_tras, pairTransform);
    //}
    if (g_status._map_clouds_rot_stamps.size() != 0) {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot_stamps,
            cloud_temp->header.stamp / 1000, cloud_temp, cloud_tras, rot)) {
            return;
        }
    } else {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot,
              g_status._frame_count_total, cloud_temp, cloud_tras, rot)) {
            return;
        }
    }
    g_status._optimize_rot.push_back(rot);
    if (g_status._cloud_result->size() == 0) {
        pcl::copyPointCloud(*g_status._cloud_src, *g_status._cloud_result);
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    }
    //loop
    //CloudPtr cloudLine(new Cloud);
    //GetCloudLine(cloud_tras, cloudLine);
    //cloudsLine.push_back(cloudLine);
    CloudPtr cloudLine(new Cloud);
    pcl::copyPointCloud(*cloud_tras, *cloudLine);
    g_status._clouds_line.push_back(cloudLine);

    *g_status._cloud_result = *g_status._cloud_result + *cloud_tras;
    //RemoveDuplicate(gCloudResult, RDUMPLICATE_VALUE);

    cout << g_status._frame_count_total << ":" << g_status._cloud_result->size() << endl;
    g_status._cloud_src->clear();
    g_status._cloud_src = cloud_temp;
    cloud_temp = CloudPtr(new Cloud);

    if (g_status._frame_count_total % 100 == 0) {
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
        PclPairVisualizer::instance().spin_once();
    }
}

void on_get_hdl_cloud_for_line(const CloudIConstPtr& cloud){
    CloudPtr cloud_temp(new Cloud);
    CloudPtr cloud_tras(new Cloud);
    g_status._frame_count_total++;

    PointCloudHelper::cloudi2cloud(cloud, cloud_temp, 7, true, 40);
    //AddTracePoint(cloudTemp);
    Matrix4f rot;

    if (g_status._map_clouds_rot_stamps.size() != 0) {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot_stamps,
            cloud->header.stamp / 1000, cloud_temp, cloud_tras, rot)) {
            return;
        }
    } else {
        if (!PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot,
              g_status._frame_count_total, cloud_temp, cloud_tras, rot)) {
            return;
        }
    }
    g_status._optimize_rot.push_back(rot);
    *g_status._cloud_result = *g_status._cloud_result + *cloud_tras;

    cout << g_status._frame_count_total << ":" << g_status._cloud_result->size() << endl;
    cloud_temp = CloudPtr(new Cloud);
    if (g_status._is_write_stamp) {
        FileHelper::write_file_optimize_rot_stamp(g_status._frame_count_total,
            g_status._pcap_file, rot, g_status._optimize_rot.size() != 1,
            cloud->header.stamp / 1000);
    }
    if (g_status._optimize_rot.size() == 1) {
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    } else {
        //PclPairVisualizer::instance().update_cloud_left(gCloudResult);
    }
}

void process_trace(string pcapFile,Matrix4f mat_rot,bool is_output_trace){
    CloudPtr cloud_line_result(new Cloud);
    vector< map< int, int> > map_loops;
    list< int> list_frame_valid;
    vector< int> list_frame_changed;
    //jake 这里随时控制观看什么样的结果
//#if IS_OUTPUT_TRACE
//    FileHelper::load_file_optimize_rot_stamp(pcapFile, g_status._map_clouds_rot,
//        g_status._map_clouds_stamp);
//#else
    FileHelper::load_file_rot(pcapFile, g_status._map_clouds_rot,
        g_status._map_clouds_rot_stamps, g_status._map_clouds_stamp);
//#endif

    get_frame_idx_vector(g_status._map_clouds_rot, list_frame_valid);
    g_status._optimize_rot.clear();

    unordered_map< int, Matrix4f> rot_new;
    unordered_map< int, Matrix4f> rot_once;
    unordered_map< int, Matrix4f> rot_final;
    Matrix4f last_frame = Matrix4f::Identity();
    Matrix4f base_frame_once = Matrix4f::Identity();
    int i = 0;

//#if !IS_OUTPUT_TRACE
	if(is_output_trace==false)
	{
		for (list< int>::iterator it = list_frame_valid.begin(); it != list_frame_valid.end();
			it++, i++) {
			int frame = *it;
			if (i == 0) {
				rot_once[frame] = g_status._map_clouds_rot[*it];
			}
			else {
				Matrix4f& temp = g_status._map_clouds_rot[*(--it)];
				it++;
				rot_once[frame] = temp.inverse() * g_status._map_clouds_rot[*it];
				if (abs(rot_once[frame](0, 3)) > MAXTRACEDIS ||
					  abs(rot_once[frame](1, 3)) > MAXTRACEDIS ||
					  abs(rot_once[frame](2, 3)) > MAXTRACEDIS) {
					cout << rot_once[frame](0, 3) << endl;
					rot_once[frame] = Matrix4f::Identity();
					list_frame_changed.push_back(frame);
				}

				  //if (frame >= 17154 + 1 && frame < 17377 + 1+10)
				  //{
				  //  if (baseFrameOnce == Matrix4f::Identity())
				  //  {
				  //      baseFrameOnce << -0.968, -0.240, -0.078, -9.113,
				  //          0.247, -0.964, -0.095, 2.415,
				  //          -0.052, -0.111, 0.992, 0.250,
				  //          0, 0, 0, 1;
				  //      Matrix4f mat=baseFrameOnce;
				  //      baseFrameOnce = lastFrame*mat;
				  //  }
				  //  lastFrame = baseFrameOnce;
				  //  list_frame_changed.push_back(frame);
				  //}else if (frame >= 17377 + 1)
				  //{
				  //  //rotOnce[frame] = baseFrameOnce*rotOnce[frame];
				  //  list_frame_changed.push_back(frame);
				  //}
				  //cout << rotOnce[frame] << endl;
			}
			last_frame = last_frame * rot_once[frame];
			rot_new[frame] = last_frame;
			//cout << frame << endl << rot_new[frame] << endl;
		}
		string file_path_pos_new(pcapFile + "_posnew.txt");
		i = 0;
		stringstream sstream;
		for (list< int>::iterator it = list_frame_valid.begin(); it != list_frame_valid.end();
			it++, i++) {
			int frame = *it;
			sstream << frame;
			Matrix4f trans = rot_new[frame];
			for (int i = 0; i < 4.; i++) {
				for (int j = 0; j < 4; j++) {
					sstream << " " << trans(i, j);
				}
			}
			sstream << "\n";
			//FileHelper::write_file(file_path_pos_new, sstream.str(), i != 0);
		}
		FileHelper::write_file(file_path_pos_new, sstream.str(), false);
		//return 0;
		CloudPtr cloud_fix(new Cloud());
		//g_status._map_clouds_rot = rot_new;
		for(unordered_map<int , Matrix4f>::iterator it=rot_new.begin();it!= rot_new.end() ;it++ )
		{
			Matrix4f& rt=it->second;
			int r=255;
			int g=0;
			int b=0;
			CloudItem tempPoint;
			tempPoint.x = rt(0,3);
			tempPoint.y = rt(1,3);
			tempPoint.z = rt(2,3);
			uint32_t rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
				static_cast< uint32_t>(b);
			bool is_find = false;
			for (int j = 0; j < list_frame_changed.size(); j++) {
				if (list_frame_changed[j] == it->first) {
					is_find = true;
					break;
				}
			}
			tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
			cloud_fix->push_back(tempPoint);
		}
		PclPairVisualizer::instance().update_cloud_right(cloud_fix);
	}
//#endif
    //string hdlCalibration;
    //HDLViewerGrabberHelper hdlHelper(*gGrabber, OnGetHdlCloudForLoop);
    //hdlHelper.Run();
    i = 0;
    for (list< int>::iterator it = list_frame_valid.begin(); it != list_frame_valid.end();
        it++) {
        CloudPtr cloud(new Cloud);
        bool is_find = false;
        for (int j = 0; j < list_frame_changed.size(); j++) {
            if (list_frame_changed[j] == *it) {
                is_find = true;
                break;
            }
        }
        if (is_find) {
            PointCloudHelper::add_trace_point(cloud, 0, 255, 0);
		}
		else {
            PointCloudHelper::add_trace_point(cloud);
        }
		if (is_output_trace) {
			//jake 理论上应该输出所有的,这里暂时只输出未改动过的点,为了便于观看
			//cout << gMapCloudsStamp[*it] << endl << *it <<endl<< gMapCloudsStamp[*it] + ((*it) << 24)<<endl;
			cloud->points[0].rgba = g_status._map_clouds_stamp[*it];//+ (*it << 24);
			//cout << (cloud->points[0].rgba << 8 >> 8) << endl << (cloud->points[0].rgba >> 24) << endl;
		}

        cloud->header.stamp = g_status._map_clouds_stamp[*it] * 1000;
        on_get_hdl_cloud_for_loop_trace(cloud);
    }

#if !IS_OUTPUT_TRACE
    //DoLumElch(list_frame_valid, map_loops);
#endif
    CloudPtr cloud_temp(new Cloud);
    for (int i = 0; i < g_status._clouds_line.size(); i++) {
        CloudPtr item = g_status._clouds_line[i];
        *cloud_line_result += *item;
    }
    i = -1;
    if (mat_rot == Matrix4f::Identity()) {
        pcl::io::savePCDFile(pcapFile + "_trace.pcd", *cloud_line_result, false);
        PclPairVisualizer::instance().update_cloud_left(cloud_line_result);
		PclPairVisualizer::instance().spin();
/*        cout << "loop trace." << endl;
        PclPairVisualizer::instance().spin();
        while (true) {
            i++;
            if (i % 2 == 0) {
                PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
                cout << "src" << endl;
                PclPairVisualizer::instance().spin();
            } else {
                PclPairVisualizer::instance().update_cloud_left(cloud_line_result);
                cout << "loop" << endl;
                PclPairVisualizer::instance().spin();
            }
        }*/
    } else {
        if (cloud_line_result->size() != 0) {
            cout << "start to output trace." << endl;
			pcl::transformPointCloud(*cloud_line_result,*cloud_line_result,mat_rot);
            pcl::io::savePCDFile(pcapFile + "_trace_z.pcd", *cloud_line_result, false);
            PclPairVisualizer::instance().update_cloud_left(cloud_line_result);
        }
    }
}

bool process_fix_pos(){
	unordered_map< int, Matrix4f> map_clouds_rot_fix;
	unordered_map< uint64, Matrix4f> map_clouds_rot_stamps_fix;
	FileHelper::load_file_rot_fix(g_status._pcap_file, map_clouds_rot_fix,
		map_clouds_rot_stamps_fix);
	if (map_clouds_rot_stamps_fix.size() <= 0)
	{
		return false;
	}
	FileHelper::load_file_rot(g_status._pcap_file, g_status._map_clouds_rot
		,g_status._map_clouds_rot_stamps);
	if (g_status._map_clouds_rot_stamps.size() > 0) {
		namespace bf = boost::filesystem;
		bf::path path_rot(g_status._pcap_file + "_pos.txt");
		bf::path path_rot_old(g_status._pcap_file + "_pos_" +
			StringHelper::get_time_string() + ".txt");
		bf::copy_file(path_rot, path_rot_old);
		bf::remove(path_rot);
		for (unordered_map< uint64_t, Matrix4f>::iterator it =
			g_status._map_clouds_rot_stamps.begin();
			it != g_status._map_clouds_rot_stamps.end(); it++) {
				if (g_status._start_frame< it->first) {
					g_status._start_frame = it->first;
				}
		}
	}
	if (g_status._map_clouds_rot.size()<=0)
	{
		return false;
	}
	list<uint64> list_frame_valid;
    get_frame_stamps(g_status._map_clouds_rot_stamps ,list_frame_valid);
	list<uint64> list_frame_valid_fix;
    get_frame_stamps(map_clouds_rot_stamps_fix,list_frame_valid_fix);
	uint64 start_frame =list_frame_valid_fix.front();
	uint64 end_frame=list_frame_valid_fix.back();
	Matrix4f rot_base=g_status._map_clouds_rot_stamps[start_frame]
		*map_clouds_rot_stamps_fix[start_frame].inverse();
	Matrix4f rot=rot_base * map_clouds_rot_stamps_fix[end_frame]
	*g_status._map_clouds_rot_stamps[end_frame].inverse();
	int idx=0;
	for (list<uint64>::iterator it=list_frame_valid.begin();it != list_frame_valid.end(); it++)
	{
		uint64 stamp=*it;
		if (stamp<start_frame)
		{
			if (g_status._map_clouds_rot_stamps.find(stamp) != g_status._map_clouds_rot_stamps.end())
			{
				g_status._optimize_rot.push_back(g_status._map_clouds_rot_stamps[stamp]);
			}
		}else if (stamp<=end_frame)
		{
			g_status._optimize_rot.push_back(rot_base * map_clouds_rot_stamps_fix[stamp]);
		}else if (stamp>end_frame)
		{
			if (g_status._map_clouds_rot_stamps.find(stamp) != g_status._map_clouds_rot_stamps.end())
			{
				g_status._optimize_rot.push_back(rot * g_status._map_clouds_rot_stamps[stamp]);
			}
		}
		FileHelper::write_file_rot(idx,
			g_status._pcap_file, g_status._optimize_rot[idx], idx != 0,
			stamp);
		idx++;
	}
    return true;
}

void process_wall(string pcap_file, bool isBatch){
    PclPairVisualizer::instance()._pv->addCoordinateSystem(20, 0, 0, -5);
    CloudPtr cloud_line_result(new Cloud);
    CloudPtr cloud_src(new Cloud);
    int i = 0;

	FileHelper::load_file_rot(pcap_file, g_status._map_clouds_rot,
		g_status._map_clouds_rot_stamps, g_status._map_clouds_stamp);
	CloudPtr cloud_process(new Cloud);
	for (unordered_map< uint64_t, Eigen::Matrix4f>::iterator it =
		g_status._map_clouds_rot_stamps.begin();
		it != g_status._map_clouds_rot_stamps.end(); it++) {
			CloudItem item;
			item.x = it->second(0, 3);
			item.y = it->second(1, 3);
			item.z = it->second(2, 3);
			int r = 255;
			int g = 0;
			int b = 0;
			uint32_t rgb = 0;
			rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
				static_cast< uint32_t>
				(b);
			item.rgb = *reinterpret_cast< float*>(&rgb);
			cloud_process->push_back(item);
	}
	cout<<"trace size "<<cloud_process->size()<<endl;

    pcl::io::loadPCDFile(pcap_file + "_loopdst.pcd", *cloud_src);

    CloudPtr cloud_temp(new Cloud());
    pcl::copyPointCloud(*cloud_src, *cloud_temp);
    //cloudSrc->clear();
    //for (int i = 0; i < cloudTemp->size(); i++) {
    //    CloudItem item = cloudTemp->points[i];
    //    //if (item.rgb!=rgbBlack)
    //     {
    //        cloudSrc->points.push_back(item);
    //    }
    //}
    //pv.addCoordinateSystem(10, 0, 0, -5);
    PclPairVisualizer::instance().update_cloud_left(cloud_src);
    isBatch == true ? PclPairVisualizer::instance().spin_once() :
        PclPairVisualizer::instance().spin();

    CloudPtr cloud_result(new Cloud);
    //RemoveOutlier(cloudSrc);
    //pv.updatePointCloud(cloudSrc);
    //pv.spin();
    //RemoveOutlier(cloudSrc);
    pcl::copyPointCloud(*cloud_src, *cloud_result);
    cout << "start to fix z" << endl;
    //todo jake 暂时取消fixz

    PclPairVisualizer::instance().update_cloud_left(cloud_src);
    ProjectPlane plane(*cloud_src,*cloud_process);
    plane.get_cloud(*cloud_src);
    if (!isBatch)
    {
        PclPairVisualizer::instance().update_cloud_right(cloud_src);
    }
    isBatch == true ? PclPairVisualizer::instance().spin_once() :
        PclPairVisualizer::instance().spin();
    //GetCloudBase(cloudSrc, cloudResult);
    //pv.updatePointCloud(cloudResult);
    //pv.spin();

    CloudPtr cloud_result_output(new Cloud);
    vector< CloudPtr> cloud_list;
    for (int i = 0; i < 80; i++) {
        CloudPtr cloud_plane(new Cloud);
        PointCloudHelper::get_cloud_line(cloud_src, cloud_result, i);
        pcl::copyPointCloud(*cloud_result, *cloud_plane);
        cout << i << endl;
        cloud_list.push_back(cloud_plane);
        PclPairVisualizer::instance().update_cloud_left(cloud_result);
        isBatch == true ? PclPairVisualizer::instance().spin_once() :
            PclPairVisualizer::instance().spin();
    }
    if (isBatch) {
        int start = 40;
        int end = 50;
        cloud_result_output->clear();
        for (int i = start; i <= end; i++) {
            *cloud_result_output = *cloud_result_output + *cloud_list[i];
        }
		if (cloud_result_output->size()==0)
		{
			cout<<"点云输出错误,数据为0"<<endl;
			return;
		}
        process_trace(pcap_file,plane._mat_rot,true);
        cout << "start to save " + pcap_file + "_loopdst_z.pcd" << endl;
        pcl::io::savePCDFile(pcap_file + "_loopdst_z.pcd", *cloud_src, true);
        cout << pcap_file + "_loopdst_z.pcd" << endl;
        pcl::io::savePCDFile(pcap_file + "_wall_z.pcd", *cloud_result_output, true);
        cout << "start to save " + pcap_file + "_wall_z.pcd" << endl;
    } else {
        while (true) {
            int start = 28;
            int end = 51;
            cout << "please insert the start index of the frame:28" << endl;
            cin >> start;
            cout << "please insert the end index of the frame:51" << endl;
            cin >> end;
            cloud_result_output->clear();
            for (int i = start; i <= end; i++) {
                *cloud_result_output = *cloud_result_output + *cloud_list[i];
            }
            PclPairVisualizer::instance().update_cloud_left(cloud_result_output);
            PclPairVisualizer::instance().spin();
            char flag = 'n';
            cout << "save?(y/n default n)" << endl;
            cin >> flag;
            if (flag == 'y' || flag == 'Y') {
                //process_trace(pcap_file);
                cout << "start to save " + pcap_file + "_loopdst_z.pcd" << endl;
                pcl::io::savePCDFile(pcap_file + "_loopdst_z.pcd", *cloud_src, true);
                cout << pcap_file + "_loopdst_z.pcd" << endl;
                pcl::io::savePCDFile(pcap_file + "_wall_z.pcd", *cloud_result_output,
                    true);
                cout << "start to save " + pcap_file + "_wall_z.pcd" << endl;
                char is_exit = 'n';
                cout << "exit?(y/n default n)" << endl;
                cin >> is_exit;
                if (is_exit == 'y') {
                    return;
                }
            }
        }
    }
}
void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
    if (!event.keyDown()) {
        return;
    }
    switch (event.getKeyCode())
    {
    case 'd':
        g_status._is_debug_mode = true;
        break;
    case 'c':
        g_status._is_debug_mode = false;
        break;
    case 'p':
        g_status._is_step_mode = !g_status._is_step_mode;
        break;
    default:
        break;
    }
}

void test_fix_pos_plane()
{
	unordered_map< int, Matrix4f> map_rots_src; 
	unordered_map< int, int64_t> stamps; 
	string pcap_file= "C:\\Users\\jake\\Desktop\\问题数据\\新建文件夹\\";
	bool flag = FileHelper::load_file(pcap_file, "POINTCLOUD201507300D_20150730112121081.pcap_pos.txt", map_rots_src, stamps, true);
	CloudPtr cloud_src(new Cloud);
	Matrix4f first=map_rots_src.begin()->second;

	list< int> list_frame_valid;
	 get_frame_idx_vector(map_rots_src, list_frame_valid);
	for(unordered_map<int , Matrix4f>::iterator it=map_rots_src.begin();it!= map_rots_src.end() ;it++ )
	{
		Matrix4f& rt=it->second;
		int r=255;
		int g=0;
		int b=0;
		CloudItem tempPoint;
		tempPoint.x = rt(0,3)-first(0,3);
		tempPoint.y = rt(1,3)-first(1,3);
		tempPoint.z = rt(2,3)-first(2,3);
		uint32_t rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
			static_cast< uint32_t>(b);
		tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
		cloud_src->push_back(tempPoint);
	}
	//PclPairVisualizer::instance().spin();
	CloudPtr cloud_dst(new Cloud());
	Matrix4f rot_src = Matrix4f::Identity();
	Matrix4f rot_dst;
	Matrix4f rot;
	ProjectPlane plane(*cloud_src,*cloud_src);
	rot_src = plane._mat_rot;
	cout << rot_src<<endl;
	plane.get_cloud(*cloud_src);

	Vector3f zero;
	zero(0) = map_rots_src.begin()->second(0,3); 
	zero(1) = map_rots_src.begin()->second(1,3); 
	zero(2) = map_rots_src.begin()->second(2,3); 
	float rot_avg = 0;
	for(unordered_map<int , Matrix4f>::iterator it=map_rots_src.begin();it!= map_rots_src.end() ;it++ )
	{
		it->second = rot_src * it->second;
		it->second(0,3)-= map_rots_src.begin()->second(0,3);
		it->second(1,3)-= map_rots_src.begin()->second(1,3);
		it->second(2,3)-= map_rots_src.begin()->second(2,3);
		rot_avg += it->second(2,2);
	}
	rot_avg /= map_rots_src.size();
	//rot_avg =map_rots_src.begin()->second(2,2);
	
	PclPairVisualizer::instance().update_cloud_left(cloud_src);
	Matrix4f rot_last = Matrix4f::Identity();
	bool is_find=false;
	int idx = 0; 
	for(unordered_map<int , Matrix4f>::iterator it=map_rots_src.begin();it!= map_rots_src.end() ;it++ ,idx++)
	{
		Matrix4f& rt=it->second;
		int r=255;
		int g=0;
		int b=0;
		CloudItem tempPoint;
		float dz=abs(rt(2,3));
		rt=it->second;	
		tempPoint.x = rt(0,3);
		tempPoint.y = rt(1,3);
		tempPoint.z = rt(2,3);
		//cout << dz << endl;
		//if (abs(rt(2, 2) - rot_avg) > 0.01/* && !is_find*/)
		if (dz>2)
		{
			cout << idx << endl;
			is_find = true;
			r=0;
			b=255;
			g=0;
			tempPoint.z=0;
			rt(2,3)=0;
			//rt(2,3)=0;
			//Vector3f va;
			//Vector3f vb;
			//va(0) = rt(0,3) - rot_last(0,3);
			//va(1) = rt(1,3) - rot_last(1,3);
			//va(2) = 0;
			//vb(0) = rt(0,3) - rot_last(0,3);
			//vb(1) = rt(1,3) - rot_last(1,3);
			//vb(2) = rot_last(1,3);
			//Matrix4f rot_vb_va;
			////cout << va << endl;
			////cout << vb << endl;
			////cout << (vb.cross(va)).norm() << " " << vb.norm() << " " << va.norm() << endl;
			//float angle = (float)asin(vb.cross(va).norm()/vb.norm()/va.norm());
			//MatHelper::rotate_arbitrary_line(vb.cross(va),angle,rot_vb_va);
			//for (unordered_map<int,Matrix4f >::iterator itt=it;itt!= map_rots_src.end();itt++)
			//{
			//	Matrix4f& rt_new = itt->second;
			//	rt_new(0,3)-=rot_last(0,3);
			//	rt_new(1,3)-=rot_last(1,3);
			//	rt_new(2,3)-=rot_last(2,3);
			//	rt_new = rot_vb_va * rt_new;
			//	rt_new(0,3)+=rot_last(0,3);
			//	rt_new(1,3)+=rot_last(1,3);
			//	rt_new(2,3)+=rot_last(2,3);
			//}
		}
		
		uint32_t rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
			static_cast< uint32_t>(b);
		tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
		cloud_dst->push_back(tempPoint);
		rot_last = rt;
	}
	string file_path_pos_new(pcap_file + "1111_posnew.txt");
	int frame = 0;
	stringstream sstream;
	//for(unordered_map<int , Matrix4f>::iterator it=map_rots_src.begin();it!= map_rots_src.end() ;it++ ,frame++)
	for (list< int>::iterator it = list_frame_valid.begin(); it != list_frame_valid.end();
		it++,frame++) 
	{
			sstream << *it;
			Matrix4f trans = map_rots_src[*it];;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					sstream << " " << trans(i, j);
				}
			}
			sstream << " " << stamps[*it];
			sstream << " 100 ";
			sstream << "\n";
			//FileHelper::write_file(file_path_pos_new, sstream.str(), i != 0);
	}
	FileHelper::write_file(file_path_pos_new, sstream.str(), false);

	PclPairVisualizer::instance().update_cloud_right(cloud_dst);
	cout << rot_dst<<endl;
	PclPairVisualizer::instance().spin();
}
void test_fix_pos_part()
{
	unordered_map< int, Matrix4f> map_rots_src; 
	unordered_map< int, int64_t> stamps; 
	string pcap_file= "C:\\Users\\jake\\Desktop\\问题数据\\新建文件夹\\";
	bool flag = FileHelper::load_file(pcap_file, "1.txt", map_rots_src, stamps, true);
	CloudPtr cloud_src(new Cloud);
	Matrix4f first=map_rots_src.begin()->second;
	for(unordered_map<int , Matrix4f>::iterator it=map_rots_src.begin();it!= map_rots_src.end() ;it++ )
	{
		Matrix4f& rt=it->second;
		int r=255;
		int g=0;
		int b=0;
		CloudItem tempPoint;
		tempPoint.x = rt(0,3)-first(0,3);
		tempPoint.y = rt(1,3)-first(1,3);
		tempPoint.z = rt(2,3)-first(2,3);
		uint32_t rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
			static_cast< uint32_t>(b);
		tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
		cloud_src->push_back(tempPoint);
	}
	unordered_map< int, Matrix4f> map_rots_dst; 
	flag = FileHelper::load_file(pcap_file, "all.txt",map_rots_dst , stamps, true);
	CloudPtr cloud_dst(new Cloud);
	for(unordered_map<int , Matrix4f>::iterator it=map_rots_dst.begin();it!= map_rots_dst.end() ;it++ )
	{
		Matrix4f& rt=it->second;
		int r=0;
		int g=255;
		int b=0;
		CloudItem tempPoint;
		tempPoint.x = rt(0,3)-first(0,3);
		tempPoint.y = rt(1,3)-first(1,3);
		tempPoint.z = rt(2,3)-first(2,3);
		uint32_t rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
			static_cast< uint32_t>(b);
		tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
		cloud_dst->push_back(tempPoint);
	}
	CloudPtr cloud_src_all(new Cloud());
    *cloud_src_all = *cloud_src + *cloud_dst;
	PclPairVisualizer::instance().update_cloud_left(cloud_src_all);
	//PclPairVisualizer::instance().spin();
	CloudPtr cloud_out(new Cloud());
	Matrix4f rot_src;
	Matrix4f rot_dst;
	Matrix4f rot;
	ProjectPlane plane(*cloud_src,*cloud_src);
	rot_src=plane._mat_rot;
	cout << rot_src<<endl;
	ProjectPlane plane_dst(*cloud_dst,*cloud_dst);
	cout << rot_dst<<endl;
	rot_dst=plane_dst._mat_rot;
	rot = (rot_dst * rot_src.inverse()).inverse();
	//rot = rot_src * rot_dst.inverse();
	cout<< rot <<endl;
	
	
	CloudPtr cloud_out_all(new Cloud());
	pcl::transformPointCloud(*cloud_src,*cloud_out,rot);
    *cloud_out_all=  *cloud_dst + *cloud_out;
//	PclPairVisualizer::instance().update_cloud_left(cloud_out);
	PclPairVisualizer::instance().update_cloud_right(cloud_out_all);
	PclPairVisualizer::instance().spin();
}

bool loop_cmp(const pair< pair<uint64_t, uint64_t>, Matrix4f>& left, const pair< pair<uint64_t, uint64_t>, Matrix4f>& right) 
{ 
    return left.first.second < right.first.second;
}

void test_loop_detection(int argc, char** argv)
{
    if (argc != 3)
        return;
    string pcap_file(argv[1]);
    float eps = atof(argv[2]);
    // string pcap_file = "D:\\PointCloud\\回环检测\\POINTCLOUD201509160L_20150916193517735.pcap";
    unordered_map<int, Matrix4f> rot;
    unordered_map<uint64_t, Matrix4f> rot_stamp;
    FileHelper::load_file_rot(pcap_file, rot, rot_stamp);
    if (rot_stamp.size() <= 0) {
        cout << "pos empty";
        return;
    }
    vector<uint64_t> stamp_vec;
    stamp_vec.reserve(rot_stamp.size());
    unordered_map<uint64_t, Matrix4f>::iterator it, it_end;
    for (it = rot_stamp.begin(), it_end = rot_stamp.end(); it != it_end; ++it) {
        stamp_vec.push_back(it->first);
    }
    sort(stamp_vec.begin(), stamp_vec.end(), [](const uint64_t right, const uint64_t left) {return right < left; });
    PointCloud<PointXYZ>::Ptr track_cloud(new PointCloud<PointXYZ>());
    track_cloud->reserve(stamp_vec.size());
    CloudPtr cloud_ori(new Cloud);
    CloudPtr cloud_final(new Cloud);
    cloud_ori->reserve(stamp_vec.size());
    cloud_final->reserve(stamp_vec.size());

    // 生成轨迹点和时间戳的map映射
    vector<PointXYZ> point_vec;
    point_vec.reserve(rot_stamp.size());
    vector<uint32_t> rgb_vec;
    rgb_vec.reserve(rot_stamp.size());
    for (size_t i = 0; i < stamp_vec.size(); i++) {
        const Matrix4f& mat = rot_stamp[stamp_vec[i]];
        const PointXYZ pt(mat(0, 3), mat(1, 3), mat(2, 3));
        point_vec.push_back(pt);
        CloudItem p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        //r g b
		uint32_t rgb = static_cast< uint32_t>(255) << 16 | static_cast< uint32_t>(0) << 8 |
			static_cast< uint32_t>(0);
		p.rgb = *reinterpret_cast< float*>(&rgb);
        rgb_vec.push_back(rgb);
        cloud_ori->push_back(p);
    }

    // 筛选临近轨迹点
    vector<pair<uint64_t, uint64_t> > loop_stamp_pair_vec;
    uint64_t last_loop_timestamp_start = 0;
    uint64_t last_loop_timestamp_end = 0;
    uint64_t min_timestamp = 0;
    float min_distance = 10000.0f;

    float search_last_distance = 0.0f;
    uint64_t search_last_timestamp = 0;
    uint64_t last_timestamp = 0;
    bool is_search = false;
    for (size_t i = 0; i < stamp_vec.size(); i++) {
        const uint64_t current_timestamp = stamp_vec[i];
        // 当前轨迹点
        const PointXYZ& this_point = point_vec[i];

        // 每10帧作为一个关键帧
        if (i % 10 != 0) {
            continue;
        }

        track_cloud->push_back(this_point);
        // 利用KD树搜索最邻近点
        KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud(track_cloud);
        vector<int> pointIdxRadiusSearch;
        vector<float> pointRadiusSquaredDistance;
        kdtree.radiusSearch(this_point, eps, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        min_distance = 10000.0f;
        int64_t min_index = -1;
        min_timestamp = 0;
        for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
            const PointXYZ& pt_tmp = (*track_cloud)[pointIdxRadiusSearch[j]];
            float distance = pointRadiusSquaredDistance[j];
            uint64_t this_timestamp = stamp_vec[10 * pointIdxRadiusSearch[j]];
            uint64_t abs_timestamp =
                (current_timestamp > this_timestamp) ?
                (current_timestamp - this_timestamp) :
                (this_timestamp - current_timestamp);
            uint64_t timestamp_tmp_start = 
                (this_timestamp > last_loop_timestamp_start) ? 
                (this_timestamp - last_loop_timestamp_start) : 
                (last_loop_timestamp_start - this_timestamp);
            uint64_t timestamp_tmp_end = 
                (current_timestamp > last_loop_timestamp_end) ? 
                (current_timestamp - last_loop_timestamp_end) : 
                (last_loop_timestamp_end - current_timestamp);
            // 回环起止时间点必须大于 5min，回环时间间隔大于50s
            if (abs_timestamp > 5 * 60 * 1000
                && (timestamp_tmp_start >= 50 * 1000 || timestamp_tmp_end >= 50 * 1000)) {
                if (min_distance > distance) {
                    min_distance = distance;
                    min_index = j;
                    min_timestamp = this_timestamp;
                }
            }
        }
        if (min_index != -1) {
            if (is_search) {
                uint64_t abs_tmp_stamp = 
                    (search_last_timestamp > min_timestamp) ?
                    (search_last_timestamp - min_timestamp) :
                    (min_timestamp - search_last_timestamp);
                if (abs_tmp_stamp >= 50 * 1000 || search_last_distance < min_distance) { // 开始新的搜索
                    loop_stamp_pair_vec.push_back(std::make_pair(search_last_timestamp, last_timestamp));
                    last_loop_timestamp_start = search_last_timestamp;
                    last_loop_timestamp_end = last_timestamp;
                    is_search = false;
                    if (search_last_distance < min_distance) {
                        continue;
                    }
                }
            }
            search_last_timestamp = min_timestamp;
            search_last_distance = min_distance;
            is_search = true;
        }
        else {
            if (is_search) {
                loop_stamp_pair_vec.push_back(std::make_pair(search_last_timestamp, last_timestamp));
                last_loop_timestamp_start = search_last_timestamp;
                last_loop_timestamp_end = last_timestamp;
                is_search = false;
            }
        }
        last_timestamp = current_timestamp;
    }
    if (is_search) {
        loop_stamp_pair_vec.push_back(make_pair(search_last_timestamp, last_timestamp));
    }
    // cout << "最近起止点对数量:" << loop_point_pair_vec.size() << endl;
    // sort(loop_stamp_pair_vec.begin(), loop_stamp_pair_vec.end(), [](const pair<uint64_t, uint64_t>& left, const pair<uint64_t, uint64_t>& right) {return left.first < right.first; });
    for (size_t i = 0; i < loop_stamp_pair_vec.size(); ++i) {
        cout << loop_stamp_pair_vec[i].first << '\t' << loop_stamp_pair_vec[i].second << endl;
    }
    cout << endl;

    // 生成新图
    for (size_t i = 0; i < point_vec.size(); i++) {
        CloudItem p;
        p.x = point_vec[i].x;
        p.y = point_vec[i].y;
        p.z = point_vec[i].z;
        uint32_t rgb = rgb_vec[i];
        p.rgb = *reinterpret_cast<float*>(&rgb);
        cloud_final->push_back(p);
    }
    // PclPairVisualizer::instance().update_cloud_left(cloud_ori);
    // PclPairVisualizer::instance().update_cloud_right(cloud_final);
    // PclPairVisualizer::instance().spin();
}

// void test_loop_detection2()
// {
//     string pcd1 = "D:\\PointCloud\\回环检测\\pcd\\POINTCLOUD201507180J_20150718161258071.pcap_1_0.pcd";
//     string pcd2 = "D:\\PointCloud\\回环检测\\pcd\\POINTCLOUD201507180J_20150718161258071.pcap_1_1.pcd";
//     string pcd3 = "D:\\PointCloud\\回环检测\\pcd\\POINTCLOUD201507180J_20150718161258071.pcap_1_3.pcd";
//     CloudPtr cloud1(new Cloud);
//     CloudPtr cloud2(new Cloud);
//     CloudPtr cloud3(new Cloud);
//     pcl::io::loadPCDFile(pcd1, *cloud1);
//     pcl::io::loadPCDFile(pcd2, *cloud2);
//     lslgeneric::LazyGrid<CloudItem> lz(0.5);
//     lslgeneric::NDTMap<CloudItem> fixed(&lz);
//     fixed.loadPointCloud(*cloud1);
//     lslgeneric::NDTMap<CloudItem> moving(&lz);
//     moving.loadPointCloud(*cloud2);
// 
//     fixed.computeNDTCells();
//     moving.computeNDTCells();
//     lslgeneric::NDTHistogram<CloudItem> fixedH(fixed);
//     lslgeneric::NDTHistogram<CloudItem> movingH(moving);
// 
//     Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> T;
//     movingH.bestFitToHistogram(fixedH, T);
//     cout << T.translation() << endl;
//     cout << T.rotation() << endl;
// 
//     pcl::transformPointCloud(*cloud2, *cloud3, T.matrix().cast<float>());
//     pcl::io::savePCDFile(pcd3, (*cloud1) + (*cloud3));
// }

void test_loop_detection3()
{
    float arr1[16] = { -0.0188133, 0.993424, -0.0165416, 95.3173, -0.987716, -0.0169454, 0.107749, 11.9601, 0.107339, 0.0185723, 0.987708, 1.99183, 0, 0, 0, 1 };
    float arr2[16] = { 0.90629, -0.371276, -0.114133, 99.9866, 0.377852, 0.909683, 0.0399032, 11.2516, 0.0901464, -0.0803764, 0.978519, 2.54383, 0, 0, 0, 1 };
    Matrix<float, 4, 4, RowMajor> first(arr1);
    Matrix<float, 4, 4, RowMajor> last(arr2);
    first.block<3, 3>(0, 0) = -Matrix3f::Identity();
    last.block<3, 3>(0, 0) = -Matrix3f::Identity();

    CloudPtr cloud1(new Cloud);
    CloudPtr cloud2(new Cloud);
    pcl::io::loadPCDFile("\\\\172.18.54.16\\d\\天津市\\天津市\\新业广场_东丽二经路\\201510010L\\point_cloud_c2\\POINTCLOUD201510010L_20151001182215847.pcap_3_0.pcd", *cloud1);
    pcl::io::loadPCDFile("\\\\172.18.54.16\\d\\天津市\\天津市\\新业广场_东丽二经路\\201510010L\\point_cloud_c2\\POINTCLOUD201510010L_20151001182215847.pcap_3_1.pcd", *cloud2);
    pcl::transformPointCloud(*cloud1, *cloud1, -first);
    pcl::transformPointCloud(*cloud2, *cloud2, -last);
    // pcl::io::savePCDFile("D:\\0.pcd", *cloud);
    // {
    //     float sum_x = 0;
    //     float sum_y = 0;
    //     for (int i = 0; i < cloud1->points.size(); i++) {
    //         CloudItem &point = cloud1->points[i];
    //         point.z = 0;
    //         if (pcl::euclideanDistance(PointXYZ(0, 0, 0), point) < 5) {
    //             point.x = 0;
    //             point.y = 0;
    //             point.z = 0;
    //         }
    //         sum_x += point.x;
    //         sum_y += point.y;
    //     }
    //     cout << atan2(sum_y, sum_x) / M_PI * 180 << endl;
    // }
    // {
    //     float sum_x = 0;
    //     float sum_y = 0;
    //     for (int i = 0; i < cloud2->points.size(); i++) {
    //         CloudItem &point = cloud2->points[i];
    //         point.z = 0;
    //         if (pcl::euclideanDistance(PointXYZ(0, 0, 0), point) < 5) {
    //             point.x = 0;
    //             point.y = 0;
    //             point.z = 0;
    //         }
    //         sum_x += point.x;
    //         sum_y += point.y;
    //     }
    //     cout << atan2(sum_y, sum_x) / M_PI * 180 << endl;
    // }
    PclPairVisualizer::instance().update_cloud_left(cloud1);
    PclPairVisualizer::instance().update_cloud_right(cloud2);
    int total_points = 0;
    float distance_max = 0;
    float distance_avg = 0;
    {
        Vector3d tmp1;
        process_loop_detection_calc_feature(cloud1, total_points, distance_max, distance_avg, tmp1);
        // tmp.normalize();
        for (int i = 0; i < 3; i++) {
            cout << tmp1(i) << endl;
        }
        cout << atan2(tmp1(1), tmp1(0)) / M_PI * 180 << endl << endl;
        Vector3d tmp2;
        process_loop_detection_calc_feature(cloud2, total_points, distance_max, distance_avg, tmp2);
        // tmp.normalize();
        for (int i = 0; i < 3; i++) {
            cout << tmp2(i) << endl;
        }
        cout << atan2(tmp2(1), tmp2(0)) / M_PI * 180 << endl << endl;
        float cos_theta = tmp1.dot(tmp2) / tmp1.norm() / tmp2.norm();
        // acos returns [0,pi] radians
        float final_angle = acos(cos_theta) / M_PI * 180;
        cout << final_angle << endl;
    }
    PclPairVisualizer::instance()._pv->addCoordinateSystem(10, 0, 0, 0);

    PclPairVisualizer::instance().spin();
}

void test_loop_detection2()
{ 
    // string pcap1 = "\\\\172.18.54.80\\d\\山东省\\青岛市\\伟东·乐客城\\201510130P\\point_cloud\\POINTCLOUD201510130P_20151013110808017.pcap_loop_test_0_0.pcd";
    // string pcap2 = "\\\\172.18.54.80\\d\\山东省\\青岛市\\伟东·乐客城\\201510130P\\point_cloud\\POINTCLOUD201510130P_20151013110808017.pcap_loop_test_0_1.pcd";
    // string pcap1 = "\\\\172.18.54.16\\d\\天津市\\天津市\\新业广场_东丽二经路\\201510010L\\point_cloud_c2\\POINTCLOUD201510010L_20151001182215847.pcap_loop_test_4_0.pcd";
    // string pcap2 = "\\\\172.18.54.16\\d\\天津市\\天津市\\新业广场_东丽二经路\\201510010L\\point_cloud_c2\\POINTCLOUD201510010L_20151001182215847.pcap_loop_test_4_1.pcd";
    string pcap1 = "\\\\172.18.54.16\\d\\天津市\\天津市\\新业广场_东丽二经路\\201510010L\\point_cloud_c2\\POINTCLOUD201510010L_20151001182215847.pcap_loop_test_1_0.pcd";
    string pcap2 = "\\\\172.18.54.16\\d\\天津市\\天津市\\新业广场_东丽二经路\\201510010L\\point_cloud_c2\\POINTCLOUD201510010L_20151001182215847.pcap_loop_test_1_1.pcd";

    CloudPtr cloud1(new Cloud);
    CloudPtr cloud2(new Cloud);
    pcl::io::loadPCDFile(pcap1, *cloud1);
    pcl::io::loadPCDFile(pcap2, *cloud2);
    PointCloudHelper::remove_duplicate(cloud1, 0.15);
    PointCloudHelper::remove_duplicate(cloud2, 0.15);
    Matrix4f rot = Matrix4f::Identity();
    bool ret = FeatureAlign::do_align_pre(cloud1, cloud2, rot);
    pcl::transformPointCloud(*cloud2, *cloud2, rot);
	PclPairVisualizer::instance().update_cloud_left(cloud2);
	*cloud2 = *cloud1 + *cloud2;
	PclPairVisualizer::instance().update_cloud_right(cloud2);
    cout << ret << endl;
    cout << rot << endl;
	PclPairVisualizer::instance().spin();
}

int main(int argc, char** argv)
{
    // test_loop_detection2();
    // return 0;
    // test_loop_detection(argc, argv);
    // // test_loop_detection3();
    // return 0;

    if (argc != 5) {
		return 0;
    } else {
        g_status._frame_count_total = atoi(argv[1]);
        g_status._pcap_file = string(argv[2]);
        g_status._current_mode = (enumModeProcess) atoi(argv[3]);
        g_status._is_batch_mode = atoi(argv[4]) == 1 ? true : false;
    }

	if (g_status._pcap_file.find("_vlp16.pcap") != string::npos) {
		g_status._current_dev=ENUM_PTDEV_VLP_16;
    } else {
		g_status._current_dev=ENUM_PTDEV_HDL_64E;
	}

    g_grabber = new PcapHelper(string(""), g_status._pcap_file,g_status._current_dev);
    g_grabber->set_frame_count_ignore(g_status._frame_count_total - 1);
    CloudPtr cloud_line_result(new Cloud);

    PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    PclPairVisualizer::instance().update_cloud_right(g_status._cloud_result);
    PclPairVisualizer::instance()._pv->registerKeyboardCallback(keyboard_callback);
    if (g_status._current_mode == ENUM_MODE_PROCESS_POS) {
        PclPairVisualizer::instance()._pv->addCoordinateSystem(10, 0, 0, -5);
        std::ofstream log(g_status._pcap_file + ".log");
        if (!g_status._is_batch_mode) {
            //std::streambuf * oldbuf = std::cout.rdbuf(log.rdbuf());
        }
        string hdl_calibration;
        g_status._optimize_rot.clear();
        //加载历史rot
        FileHelper::load_file_rot(g_status._pcap_file, g_status._map_clouds_rot,
            g_status._map_clouds_rot_stamps);
        if (g_status._map_clouds_rot_stamps.size() > 0) {
            namespace bf = boost::filesystem;
            bf::path path_rot(g_status._pcap_file + "_pos.txt");
            bf::path path_rot_old(g_status._pcap_file + "_pos_" +
                    StringHelper::get_time_string() + ".txt");
            bf::copy_file(path_rot, path_rot_old);
            bf::remove(path_rot);
            for (unordered_map< uint64_t, Matrix4f>::iterator it =
                g_status._map_clouds_rot_stamps.begin();
                it != g_status._map_clouds_rot_stamps.end(); it++) {
                if (g_status._start_frame< it->first) {
                    g_status._start_frame = it->first;
                }
            }
        }
        //加载imu
        g_imu_trans = new ImuTras(g_status._pcap_file + "_mt.txt");
        if (g_imu_trans->is_valid() == false) {
            cout << "Imu load failed................................" << endl;
        }
        //加载loop
        // vector< map< uint64_t, uint64_t> > map_loops;
		vector< pair< pair<uint64_t, uint64_t>, Matrix4f> > map_loops;
        FileHelper::load_file_loop(g_status._pcap_file, map_loops);
        std::sort(map_loops.begin(), map_loops.end(), loop_cmp);
        g_status._map_loops = map_loops;
        g_status._optimize_rot.clear();
        //加载houseback
        FileHelper::load_file_house_back(g_status._pcap_file, g_status._house_back_stamp);
        for (int i= 0; i < map_loops.size(); i++) {
            const pair< uint64_t, uint64_t>& item = map_loops[i].first;
            uint64_t sec = item.second;
            uint64_t fir = item.first;
            g_status._house_back_stamp.push_back((sec - fir) / 2 + fir);
        }
        g_status._house_back_stamp.sort();
        //加载玻璃走廊
        //FileHelper::load_file_glass(g_status._pcap_file, g_status._glass_stamp);
        //加载忽略frame
        FileHelper::load_file_ignore_frame(g_status._pcap_file, g_status._ignore_stamp);

        // 加载忽略回环优化frame
        FileHelper::load_file_loop_ignore(g_status._pcap_file, g_status._map_ignore_loops);

        ImuTras imuFrame(*g_imu_trans);
        delete g_grabber;
        g_grabber = NULL; 
        g_grabber = new PcapHelper(string(""), g_status._pcap_file,g_status._current_dev, &imuFrame);
        g_grabber->set_frame_count_ignore(g_status._frame_count_total - 1);
        HDLViewerGrabberHelper hdl_helper(*g_grabber, on_get_hdl_cloud_for_reg);
        hdl_helper.Run();

		if (g_status._clouds_line.size() == 0)
		{
			cout << "pcap文件没有数据，疑似损坏" << endl;
			return 0;
		}
        // zhangyuzhi 20151028
        // 对整个点云结束后还正在搜索的回环点进行处理
        process_loop(g_status._clouds_line[g_status._clouds_line.size() - 1], true);

        pcl::io::savePCDFile(g_status._pcap_file + "_loopsrc.pcd",
            *g_status._cloud_result, false); 
        //CloudGrid::instance().GetResultCloud(gCloudResult);
        //pcl::io::savePCDFile(gPcapFile + "_loopsrcall.pcd", *gCloudResult, false);
        delete g_imu_trans;
        g_imu_trans = NULL;
    } else if (g_status._current_mode == ENUM_MODE_PROCESS_LOOP) {
          //std::ofstream log(gPcapFile + ".log");
        //std::streambuf * oldbuf = std::cout.rdbuf(log.rdbuf());
        //加载imu
        g_imu_trans = new ImuTras(g_status._pcap_file + "_mt.txt");
        if (g_imu_trans->is_valid() == false) {
            cout << "Imu load failed................................" << endl;
        }
        vector< pair< pair<uint64_t, uint64_t>, Matrix4f> > &map_loops = g_status._map_loops;
        FileHelper::load_file_loop2(g_status._pcap_file, map_loops);
        FileHelper::load_file_rot(g_status._pcap_file, g_status._map_clouds_rot,
              g_status._map_clouds_rot_stamps);
          //加载忽略frame
        FileHelper::load_file_ignore_frame(g_status._pcap_file, g_status._ignore_stamp);
        cout << g_status._map_clouds_rot[3] << endl;
        if (g_status._map_clouds_rot.size() == 0) {
            cout << "error ... rot empty" << endl;
            return 0;
        } else {
            //if (check_loop_valid(map_loops, g_status._map_clouds_rot_stamps) == false) {
            //    return -1;
            //}
        }
        if (map_loops.size() == 0) {
            cout << "error ... loop empty" << endl;
            //return 0;
        }
          //FileHelper::LoadFileRotNew(gPcapFile, gMapCloudsRot,gMapCloudsRotStamps);
          //GetFrameIdxVector(gMapCloudsRot, list_frame_valid);
        g_status._optimize_rot.clear();
        string hdl_calibration;
          //加载玻璃走廊
        FileHelper::load_file_glass(g_status._pcap_file, g_status._glass_stamp);
        ImuTras imuFrame(*g_imu_trans);
        delete g_grabber;
        g_grabber = NULL; 
        g_grabber = new PcapHelper(string(""), g_status._pcap_file,g_status._current_dev, &imuFrame);
        g_grabber->set_frame_count_ignore(g_status._frame_count_total - 1);
		uint64_t stamp_max = 0;
		for (unordered_map< uint64_t, Matrix4f>::iterator it =
			g_status._map_clouds_rot_stamps.begin();
			it != g_status._map_clouds_rot_stamps.end(); ++it) {
				uint64_t stamp = it->first;
				if (stamp_max < stamp) {
					stamp_max = stamp;
				}
		}
		g_grabber->set_stamp_end(stamp_max);
        //jake todo 
        //g_grabber->_is_split_frame_global = false;
        HDLViewerGrabberHelper hdl_helper(*g_grabber, on_get_hdl_cloud_for_loop);
        hdl_helper.Run();

          //for (int i = 0; i < gCloudsLine.size(); i++)
          //{
          //  CloudPtr item = gCloudsLine[i];
          //  *gCloudResult += *item;
          //}
          //RemoveDuplicate(gCloudResult, 0.2f);
          //pcl::io::savePCDFile(gPcapFile + "_loopSrcNew.pcd", *gCloudResult, false);

        //jake 暂时只在处理时做
        //PointCloudHelper::do_lum_elch(map_loops, g_status._clouds_line,
        //    g_status._optimize_rot);
        // zhangyuzhi
        if (g_status._loop_detection_status.is_perform_search) {
            pair<uint64_t, uint64_t> loop_stamp_pair;
            pair<uint64_t, uint64_t> loop_index_pair;
            loop_index_pair.first = g_status._loop_detection_status.search_last_timestamp;
            loop_index_pair.second = g_status._clouds_line.size() - 1;
            bool loop_check_ret = process_loop_detection_check(loop_index_pair, 10.0f, 0.30f, 0.8f);
            if (loop_check_ret) {
                loop_stamp_pair.first = g_status._clouds_line[loop_index_pair.first]->header.stamp / 1000;
                loop_stamp_pair.second = g_status._clouds_line[loop_index_pair.second]->header.stamp / 1000;
                map_loops.push_back(make_pair(loop_stamp_pair, Matrix4f::Identity()));
            }
        }
        std::sort(map_loops.begin(), map_loops.end(), loop_cmp);
        for (size_t i = 0; i < map_loops.size(); i++) {
            pair<uint64_t, uint64_t>& loop_stamp_pair = map_loops[i].first;
            pair<uint64_t, uint64_t> loop_stamp_pair_new;
            vector<CloudPtr>& clouds_line = g_status._clouds_line;
            int first = -1;
            int last = -1;
            bool do_end = true;
            size_t clouds_line_index = 0;
            for (clouds_line_index = 0; 
                clouds_line_index < clouds_line.size(); 
                clouds_line_index++) {
                if (clouds_line[clouds_line_index]->header.stamp / 1000 > loop_stamp_pair.first) {
                    size_t tmp_index = 0;
                    if (clouds_line_index == 0) {
                        tmp_index = clouds_line_index;
                    } else {
                        tmp_index = clouds_line_index - 1;
                    }
                    loop_stamp_pair_new.first = clouds_line[tmp_index]->header.stamp / 1000;
                    break;
                }
            }
            for (; clouds_line_index < clouds_line.size(); clouds_line_index++) {
                if (clouds_line[clouds_line_index]->header.stamp / 1000 >= loop_stamp_pair.second) {
                    loop_stamp_pair_new.second = clouds_line[clouds_line_index]->header.stamp / 1000;
                    do_end = false;
                    break;
                }
            }
            if (do_end) {
                loop_stamp_pair_new.second = clouds_line[clouds_line.size() - 1]->header.stamp / 1000;
            }
            if (loop_stamp_pair != loop_stamp_pair_new) {
                cout << "loop stamp change from ("
                    << loop_stamp_pair.first
                    << ", "
                    << loop_stamp_pair.second
                    << ") to ("
                    << loop_stamp_pair_new.first
                    << ", "
                    << loop_stamp_pair_new.second
                    << ")"
                    << endl;
                loop_stamp_pair = loop_stamp_pair_new;
            }
        }
        PointCloudHelper::do_lum_elch(
            map_loops, 
            g_status._clouds_line, 
            g_status._optimize_rot, 
            g_status._pcap_file, 
            -1, 
            0);
        CloudPtr cloud_temp(new Cloud);
          //string loopsrc = gPcapFile + "_loopsrc.pcd";
          //if ((_access(loopsrc.c_str(), 0)) != -1)
          //{
          //  pcl::io::loadPCDFile(loopsrc, *cloudTemp);
          //  pcl::copyPointCloud(*cloudTemp, *gCloudResult);
          //  cloudTemp->clear();
          //}
        if (map_loops.size() > 0) {
            namespace bf = boost::filesystem;
            bf::path path_rot(g_status._pcap_file + "_pos.txt");
            bf::path path_rot_old(g_status._pcap_file + "_pos_loop_src_loop_" +
                StringHelper::get_time_string() + ".txt");
            bf::copy_file(path_rot, path_rot_old);
            for (int i = 0; i < g_status._clouds_line.size(); i++) {
                CloudPtr item = g_status._clouds_line[i];
                FileHelper::write_file_rot(item->header.seq, g_status._pcap_file,
                    g_status._optimize_rot[i], i != 0, item->header.stamp / 1000);
                // if (g_status._is_write_stamp) {
                //     if (map_loops.size() == 0) {
                //         // write_file_stamp
                //         FileHelper::write_file_optimize_rot_stamp(item->header.seq,
                //             g_status._pcap_file, g_status._optimize_rot[i], i != 0,
                //             item->header.stamp / 1000);
                //     } else {
                //         FileHelper::write_file_optimize_rot_stamp(item->header.seq,
                //             g_status._pcap_file, g_status._optimize_rot[i], i != 0,
                //             item->header.stamp / 1000);
                //     }
                // } else {
                //       FileHelper::write_file_optimize_rot(item->header.seq,
                //           g_status._pcap_file, g_status._optimize_rot[i], i != 0);
                // }
                //RemoveDuplicate(item, RDUMPLICATE_VALUE);

                //*cloud_line_result += *item;
                //if (i % 5000 == 0) {
                //    PointCloudHelper::remove_duplicate(cloud_line_result, 0.1f);
                //}
				CloudDensity::instance().add_cloud(item);
            }
        } else {
            for (int i = 0; i < g_status._clouds_line.size(); i++) {
                CloudPtr item = g_status._clouds_line[i];
                //RemoveDuplicate(item, RDUMPLICATE_VALUE);
                //*cloud_line_result += *item;
                //pv.updatePointCloud(cloud_line_result);
                //pv.spinOnce();
                //if (i % 5000 == 0) {
                //    PointCloudHelper::remove_duplicate(cloud_line_result, 0.1f);
                //}
				CloudDensity::instance().add_cloud(item);
            }
        }
		uint64_t avg_hit_num = CloudDensity::instance().get_avg_hit_num();
		cout << "avg hit num " << avg_hit_num << endl;
		for(int i = avg_hit_num > 10?(avg_hit_num - 10):0; i < avg_hit_num + 10;i += 1)
		{
			CloudDensity::instance().get_cloud_hit_num(cloud_line_result,i);
            PclPairVisualizer::instance().update_cloud_left(cloud_line_result);
            PclPairVisualizer::instance().spin();
			cout << "hit num " << i << endl;
			std::stringstream file_result;
			file_result << g_status._pcap_file << "_loopdst_" << i << ".pcd";
			pcl::io::savePCDFile(file_result.str(), *cloud_line_result,
				false);
		}
		CloudDensity::instance().get_cloud_hit_num(cloud_line_result,avg_hit_num);
        //PointCloudHelper::remove_duplicate(cloud_line_result, 0.1f);
        pcl::io::savePCDFile(g_status._pcap_file + "_loopdst.pcd", *cloud_line_result,
            false);
    }else if (g_status._current_mode == ENUM_MODE_PROCESS_SHOW) {
        PclPairVisualizer::instance().show_single();
        PclPairVisualizer::instance()._pv->addCoordinateSystem(2, 0, 0, -5);
CloudItem item_start;
	CloudItem item_end;
	item_start.x=0;
	item_start.y=0;
	item_start.z=0;
	item_end.x=5;
	item_end.y=0;
	item_end.z=0;
		PclPairVisualizer::instance()._pv->addArrow(item_start,item_end,0.5,0.5,0,false);


		pcl::visualization::PCLVisualizer* viewer = PclPairVisualizer::instance()._pv;
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
		viewer->setBackgroundColor(255,255,255);
        //加载imu
        g_imu_trans = new ImuTras(g_status._pcap_file + "_mt.txt");
        if (g_imu_trans->is_valid() == false) {
            cout << "Imu load failed................................" << endl;
        }
        vector< pair< pair<uint64_t, uint64_t>, Matrix4f> > &map_loops = g_status._map_loops;
        FileHelper::load_file_loop2(g_status._pcap_file, map_loops);
        FileHelper::load_file_rot(g_status._pcap_file, g_status._map_clouds_rot,
              g_status._map_clouds_rot_stamps);
          //加载忽略frame
        FileHelper::load_file_ignore_frame(g_status._pcap_file, g_status._ignore_stamp);
        cout << g_status._map_clouds_rot[3] << endl;
        if (g_status._map_clouds_rot.size() == 0) {
            cout << "error ... rot empty" << endl;
            return 0;
        } else {
            //if (check_loop_valid(map_loops, g_status._map_clouds_rot_stamps) == false) {
            //    return -1;
            //}
        }
        if (map_loops.size() == 0) {
            cout << "error ... loop empty" << endl;
            //return 0;
        }
        g_status._optimize_rot.clear();
        string hdl_calibration;
        ImuTras imuFrame(*g_imu_trans);
        delete g_grabber;
        g_grabber = NULL; 
        if (g_status._is_batch_mode == false)
        {
            cout << "press any key to continue." << endl;
            getchar();
        }
        g_grabber = new PcapHelper(string(""), g_status._pcap_file,g_status._current_dev, &imuFrame);
        g_grabber->set_frame_count_ignore(g_status._frame_count_total - 1);
		uint64_t stamp_max = 0;
		for (unordered_map< uint64_t, Matrix4f>::iterator it =
			g_status._map_clouds_rot_stamps.begin();
			it != g_status._map_clouds_rot_stamps.end(); ++it) {
				uint64_t stamp = it->first;
				if (stamp_max < stamp) {
					stamp_max = stamp;
				}
		}
		g_grabber->set_stamp_end(stamp_max);
        HDLViewerGrabberHelper hdl_helper(*g_grabber, on_get_hdl_cloud_for_show);
        hdl_helper.Run();
		CloudDensity::instance().get_cloud_hit_num(cloud_line_result,10);
        //PointCloudHelper::remove_duplicate(cloud_line_result, 0.1f);
        pcl::io::savePCDFile(g_status._pcap_file + "_loopdst.pcd", *cloud_line_result,
            false);
    } else if (g_status._current_mode == ENUM_MODE_PROCESS_LINE) {
        if (!FileHelper::load_file_optimize_rot_stamp(g_status._pcap_file,
              g_status._map_clouds_rot, g_status._map_clouds_stamp)) {
            FileHelper::load_file_optimize_rot(g_status._pcap_file,
                g_status._map_clouds_rot);
            g_status._is_write_stamp = true;
        } else {
            g_status._is_write_stamp = false;
        }
          //FileHelper::LoadFileRot(gPcapFile, gMapCloudsRot,gMapCloudsRotStamps);
        g_status._optimize_rot.clear();
        string hdl_calibration;
        HDLViewerGrabberHelper hdlHelper(*g_grabber, on_get_hdl_cloud_for_line);
        hdlHelper.Run();
          pcl::io::savePCDFile(g_status._pcap_file + "_line.pcd", *g_status._cloud_result,
              false);
    } else if (g_status._current_mode == ENUM_MODE_PROCESS_LOOP_TRACE) {
        process_trace(g_status._pcap_file,Matrix4f::Identity(),false);
    } else if (g_status._current_mode == ENUM_MODE_PROCESS_OUT_PUT) {
        process_wall(g_status._pcap_file, g_status._is_batch_mode);
    } else if (g_status._current_mode == ENUM_MODE_PROCESS_FIXPOS) {
		process_fix_pos();
	}
    delete g_grabber;
    g_grabber = NULL;

    //DoLum();
    if (!g_status._is_batch_mode && g_status._current_mode != ENUM_MODE_PROCESS_LOOP) {
        PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
        PclPairVisualizer::instance().spin();
    }

    int i = 0;
    if (!g_status._is_batch_mode && g_status._current_mode == ENUM_MODE_PROCESS_LOOP) {
        if (cloud_line_result->size() == 0) {
            string loopsrc = g_status._pcap_file + "_loopsrc.pcd";
            if ((_access(loopsrc.c_str(), 0)) != -1) {
                pcl::io::loadPCDFile(loopsrc, *g_status._cloud_result);
            }
        }
        while (true) {
            i++;
            if (i % 2 == 0) {
                PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
                cout << "src" << endl;
                PclPairVisualizer::instance().spin();
            } else {
                PclPairVisualizer::instance().update_cloud_left(cloud_line_result);
                cout << "loop" << endl;
                PclPairVisualizer::instance().spin();
            }
        }
    }
    //pv.spin();
    return 0;
}
} // cloud_icp_reg

int main(int argc, char** argv){
    time_t tm_tmp = time(NULL);
    tm* p_time = localtime(&tm_tmp);
    if (p_time->tm_year + 1900 > 2016)// || p_time->tm_mon + 1 >= 8)
    {
        return 0;
    }
    cloud_icp_reg::main(argc, argv);
}
