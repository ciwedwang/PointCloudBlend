#include "stdafx.h"
#include "point_cloud_helper.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include "elch.hpp"
#include "gpc.h"
#include "mat_helper.h"

namespace cloud_icp_reg {
using Eigen::Matrix4f;
using boost::unordered_map;

const bool is_super_cache = true;

int g_cache_icp_num = 0;
uint32_t rgbBlack = static_cast< uint32_t>(0) << 16 | static_cast< uint32_t>(0) << 8 |
        static_cast< uint32_t>(0);
static TriMesh* g_trimesh_cache = NULL;
static KDtree* g_kdtree = NULL;
static int g_cache_num = 0;

PointCloudHelper::PointCloudHelper(){
}


PointCloudHelper::~PointCloudHelper(){
}

float PointCloudHelper::get_rot_icp_pl(CloudPtr cloud_src, CloudPtr cloud_temp,
    Matrix4f& mat_rot){
    int count = min(cloud_src->size(), cloud_temp->size());
    struct gpc_corr dummy;
    vector< gpc_corr> c(count, dummy);
    for (int i = 0; i < cloud_src->size() && i < cloud_temp->size(); i++) {
        c[i].p[0] = cloud_src->points[i].x;
        c[i].p[1] = cloud_src->points[i].y;
        c[i].q[0] = cloud_temp->points[i].x;
        c[i].q[1] = cloud_temp->points[i].y;
        c[i].C[0][0] = 1;
        c[i].C[0][1] = 0;
        c[i].C[1][0] = 0;
        c[i].C[1][1] = 1;
        c[i].valid = 1;
    }

    double x[3];
    double std = 1;
    const double inv_cov_x0[9] = {
        1 / (std* std), 0, 0, 0, 1 / (std* std), 0, 0, 0, 0
    };

    int ok = gpc_solve(count, c, 0, inv_cov_x0, x);
    if (!ok) {
        cout << "gpc_solve_valid failed\n" << endl;
        return 0;
    }

    mat_rot << cos(x[2]), sin(x[2]), 0, x[0], -sin(x[2]), cos(x[2]), 0, x[1],
    0, 0, 1, 0,
    0.0, 0.0, 0.0, 1.0;
    return sqrt(x[0] + x[1]);
}

void PointCloudHelper::remove_outlier(CloudPtr& cloud_src, int min_count){
    return;
    if (cloud_src->size() < min_count) {
        return;
    }

    double t_start = clock();
    Cloud cloud;
    //pcl::RadiusOutlierRemoval<CloudItem> outrem;// 创建滤波器    
    //outrem.setInputCloud(cloud_src);              //设置输入点云
    //outrem.setRadiusSearch(0.3);              //设置在0.8半径的范围内找邻近点
    //outrem.setMinNeighborsInRadius(3);       //设置查询点的邻近点集数小于2的删除
    //outrem.filter(cloud);
    //cout << "remove outlier" << clock() -
    //    t_start << " src " << cloud_src->points.size() << " dst " << cloud.points.size() << endl;
    t_start = clock();
    pcl::StatisticalOutlierRemoval< CloudItem> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(2);
    sor.setStddevMulThresh(0.3);
    sor.filter(cloud);
    if (cloud.size() >= min_count) {
        pcl::copyPointCloud(cloud, *cloud_src);
    }
}


bool PointCloudHelper::cloud_convert_with_rot(const unordered_map< uint64_t, Matrix4f>& rot_stamps,
    const uint64_t stamp, const CloudPtr& cloud_temp, CloudPtr& cloud_trans,
    Matrix4f& rot_out)
{
    if (rot_stamps.find(stamp) == rot_stamps.end()) {
        cout << "stamp rot not found :" << stamp;
        return false;
    } else {
        Matrix4f pair_transform = (rot_stamps.find(stamp))->second;
        rot_out = pair_transform;
        //cout << "final rot matrix:" << endl << pairTransform << endl << endl;
        pcl::transformPointCloud(*cloud_temp, *cloud_trans, pair_transform);
        return true;
    }
}

bool PointCloudHelper::cloud_convert_with_rot(const unordered_map< int, Matrix4f>& rot,
    const int frame_count, const CloudPtr& cloud_temp, CloudPtr& cloud_trans,
    Matrix4f& rot_out){
    if (rot.find(frame_count) == rot.end()) {
        cout << "frame rot not found :" << frame_count;
        return false;
    } else {
        Matrix4f pairTransform = (rot.find(frame_count))->second;
        rot_out = pairTransform;
        //cout << "final rot matrix:" << endl << pairTransform << endl << endl;
        pcl::transformPointCloud(*cloud_temp, *cloud_trans, pairTransform);
        return true;
    }
}

void PointCloudHelper::add_trace_point(CloudPtr& cloudTemp, int r, int g, int b){
    CloudItem tempPoint;
    tempPoint.x = 0;
    tempPoint.y = 0;
    tempPoint.z = 0.5;
    uint32_t rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
            static_cast< uint32_t>(b);
    tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
    cloudTemp->push_back(tempPoint);
}

void PointCloudHelper::add_trace_point(CloudPtr& cloud_temp, Matrix4f rot){
    CloudItem temp_point;
    //tempPoint.x = 0;
    //tempPoint.y = 0;
    //tempPoint.z = 0.5;
    temp_point.x = rot(0, 3);
    temp_point.x = rot(1, 3);
    temp_point.x = rot(2, 3);
    int r = 255;
    int g = 0;
    int b = 0;
    uint32_t rgb = 0;
    rgb = static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
        static_cast< uint32_t>
        (b);
    temp_point.rgb = *reinterpret_cast< float*>(&rgb);
    cloud_temp->push_back(temp_point);
}


void PointCloudHelper::do_lum_elch(vector< pair< pair<uint64_t, uint64_t>, Matrix4f> >& map_loops,
    vector< CloudPtr>& clouds_line, vector< Matrix4f>& optimize_rot, const string& pcap_file, const int output_pcd_index, const size_t add_point_cloud_start_index){
    if (map_loops.size() == 0) {
        return;
    }
    static pcl::registration::ELCH< CloudItem> elch;
    pcl::IterativeClosestPoint< CloudItem, CloudItem>::Ptr icp(
            new pcl::IterativeClosestPoint< CloudItem, CloudItem>);
    icp->setMaximumIterations(600);
    //icp->setMaxCorrespondenceDistance(0.003f);
    //icp->setRANSACOutlierRejectionThreshold(0.003f);
    icp->setMaxCorrespondenceDistance(30.0f);
    icp->setRANSACOutlierRejectionThreshold(30.0f);

    //jake test
    icp->setMaxCorrespondenceDistance(30.0f);
    icp->setRANSACOutlierRejectionThreshold(20.0f);

    elch.setReg(icp);


    int first = -1, last = -1;
    vector< pair< pair<int, int>, Matrix4f > > map_loops_idxs;

	for (int i = 0; i < map_loops.size(); i++) {
		const pair<uint64_t, uint64_t>& loop_stamp_pair = map_loops[i].first;
		first = last = -1;

		int idx = 0;
		bool isInverse = false;
		for (size_t clouds_line_index = 0; clouds_line_index < clouds_line.size(); clouds_line_index++) {
			if (abs((int)(clouds_line[clouds_line_index]->header.stamp / 1000 - loop_stamp_pair.first)) < 100) {
				first = clouds_line_index;
			}
			if (abs((int)(clouds_line[clouds_line_index]->header.stamp / 1000 - loop_stamp_pair.second)) < 100) {
				last = clouds_line_index;
			}
		}
		if (first == -1 || last == -1) {
			cout <<
				"error: loop idx not found in rots,please reset ....................................."
				<< loop_stamp_pair.first << endl;
			for (size_t i = 0; i < clouds_line.size(); i++) {
				cout << "stamp :" << clouds_line[i]->header.stamp / 1000 << endl;
			}
			cout <<
				"error: loop idx not found in rots,please reset ....................................."
				<< loop_stamp_pair.first << endl;
			//getchar();
			return;
		}

        if (abs(first) > (clouds_line.size() - 1) ||
			abs(last) > (clouds_line.size() - 1)) {
			cout << "loop idx unvalid ..." << loop_stamp_pair.first << "........."
				<< loop_stamp_pair.second << endl;
			continue;
		}
		pair< pair< int, int>, Matrix4f> item;
		item.first.first = first;
		item.first.second = last;
		item.second = map_loops[i].second;
		map_loops_idxs.push_back(item);
	}

	for (int i = 0; i < map_loops_idxs.size(); i++) {
		first = map_loops_idxs[i].first.first;
		last = map_loops_idxs[i].first.second;
		uint64_t stampFirst = clouds_line[first]->header.stamp;
		for (int j = -20; j < 20; j++) {
			if (j + first<0 || j + first>clouds_line.size() - 1)
				continue;
			*clouds_line[first] += *clouds_line[j + first];
		}
		clouds_line[first]->header.stamp = stampFirst;
		cout << "add lines to cloudline first... " << first << endl;
    }

	for (int i = 0; i < map_loops_idxs.size(); i++) {
		first = map_loops_idxs[i].first.first;
		last = map_loops_idxs[i].first.second;
		uint64_t stampLast = clouds_line[last]->header.stamp;
		for (int j = -20; j < 20; j++) {
			if (last + j<0 || last + j>clouds_line.size() - 1)
				continue;
			*clouds_line[last] += *clouds_line[last+j];
		}
        clouds_line[last]->header.stamp = stampLast;
		cout << "add lines to cloudline last... " << last << endl;
	}

    for (size_t i = add_point_cloud_start_index; i < clouds_line.size(); i++) {
        elch.addPointCloud(clouds_line[i]);
    }

    if (map_loops.size() != 0) {
        for (int i = 0; i < map_loops_idxs.size(); i++) {
            first = map_loops_idxs[i].first.first;
            last = map_loops_idxs[i].first.second;
            elch.setLoopStart(first);
            elch.setLoopEnd(last);
            cout << "loop start " << first << " end " << last << endl;

            Matrix4f loop_trans_mat = Matrix4f::Identity();
            Matrix4f icp_trans_mat = Matrix4f::Identity();
			float dis = -1;
			const Matrix4f& init_trans_mat = map_loops_idxs[i].second;
			cout << "use init loop transform matrix" << endl;
			//cout << init_trans_mat << endl;
			CloudPtr last_temp(new Cloud);
            pcl::transformPointCloud(*(clouds_line[last]), *last_temp, init_trans_mat);
			//cout << "transformPointCloud done" << endl;
			//cout << first << endl;
			//cout << "clouds_line[first]->size(): " << clouds_line[first]->size() << endl;
			//cout << "last_temp->size(): " << last_temp->size() << endl;
			//cout << "saving test_0.pcd" << endl;
			//pcl::io::savePCDFile("d:\\test_0.pcd", *clouds_line[first], true);
			//cout << "save test_0.pcd done" << endl;
			//cout << "saving test_1.pcd" << endl;
			//pcl::io::savePCDFile("d:\\test_1.pcd", *last_temp, true);
			//cout << "save test_1.pcd done" << endl;
			UINT32 size_uint32 = 3000000;
			if (clouds_line[first]->size() >= size_uint32)
			{
				PointCloudHelper::remove_duplicate(clouds_line[first], 0.3);
			}
			if (last_temp->size() >= size_uint32)
			{
				PointCloudHelper::remove_duplicate(last_temp, 0.3);
			}
			dis = PointCloudHelper::get_rot_icp(clouds_line[first], last_temp, icp_trans_mat, true);
			//cout << "get_rot_icp done" << endl;
			//cout << icp_trans_mat << endl;
			loop_trans_mat = icp_trans_mat * init_trans_mat;
			//cout << loop_trans_mat << endl;

            CloudPtr cloud_out(new Cloud());
			if (clouds_line[first]->size() > 0) {
                int tmp_index = (output_pcd_index == -1) ? i : output_pcd_index;
                if (pcap_file.empty()) {
                    pcl::io::savePCDFile("d:\\0.pcd", *clouds_line[first], true);
                } else {
                    stringstream ss;
                    string str_tmp;
                    ss << pcap_file << "_" << tmp_index << "_0.pcd";
                    pcl::io::savePCDFile(ss.str(), *clouds_line[first], true);
                }
			}
			if (clouds_line[last]->size() > 0) {
                int tmp_index = (output_pcd_index == -1) ? i : output_pcd_index;
                if (pcap_file.empty()) {
                    pcl::io::savePCDFile("d:\\1.pcd", *clouds_line[last], true);
                } else {
                    stringstream ss;
                    string str_tmp;
                    ss << pcap_file << "_" << tmp_index << "_1.pcd";
                    pcl::io::savePCDFile(ss.str(), *clouds_line[last], true);
                }
			}
            cout << "loop transform dis " << dis << endl;
            cout << "loop transform" << endl << loop_trans_mat << endl;
            elch.setLoopTransform(loop_trans_mat);
            elch.compute(optimize_rot);

            cloud_out->clear();
			// boost::shared_ptr< pcl::registration::ELCH< CloudItem>::LoopGraph > loop_graph_ = elch.getLoopGraph();
            cout << "cloudsline " << clouds_line.size() << endl;
            /*cloudsLine.clear();
                                                    for (size_t i = 0; i < num_vertices(*loop_graph_); i++)
                                                    {
                                                        cloudsLine.push_back((*loop_graph_)[i].cloud);
                                                    }
                                                    cout << "cloudsline2 " << cloudsLine.size() << endl;*/
            *cloud_out = *(clouds_line[last]) + *(clouds_line[first]);
            if (cloud_out->size() > 0) {
                int tmp_index = (output_pcd_index == -1) ? i : output_pcd_index;
                if (pcap_file.empty()) {
                    pcl::io::savePCDFile("d:\\2.pcd", *cloud_out, true);
                } else {
                    stringstream ss;
                    string str_tmp;
                    ss << pcap_file << "_" << tmp_index << "_2.pcd";
                    pcl::io::savePCDFile(ss.str(), *cloud_out, true);
                }
            }
        }
	}
	else{
		cout<< "map_loops为空" << endl;
	}
    //else {
    //    first = 1;
    //    last = clouds_line.size() - 5;
    //    elch.setLoopStart(first);
    //    elch.setLoopEnd(last);
    //    cout << "loop start " << first << " end " << last;
    //    //Matrix4f rotLoop;
    //    //GetRotIcp(cloudsLine[first], cloudsLine[last], rotLoop);
    //    //cout << "loop transform" << endl << rotLoop << endl;
    //    elch.compute(optimize_rot);
    //    //elch.compute();
    //}
}


bool PointCloudHelper::loop_detection(const int end, const vector< CloudPtr>& clouds,
    double dist, int& first, int& last){
    static double min_dist = -1;
    int state = 0;

    for (int i = end - 1; i > 0; i--) {
        Eigen::Vector4f cstart, cend;
        //TODO use pose of scan
        pcl::compute3DCentroid(*(clouds[i]), cstart);
        pcl::compute3DCentroid(*(clouds[end]), cend);
        Eigen::Vector4f diff = cend - cstart;

        double norm = diff.norm();

        //std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

        if (state == 0 && norm > dist) {
            state = 1;
            //std::cout << "state 1" << std::endl;
        }
        if (state > 0 && norm < dist) {
            state = 2;
            //std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
            if (min_dist < 0 || norm < min_dist) {
                min_dist = norm;
                first = i;
                last = end;
            }
        }
    }
    //std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
    if (min_dist > 0 && (state < 2 || end == int(clouds.size()) - 1)) //TODO
    {
        min_dist = -1;
        return true;
    }
    return false;
}

bool PointCloudHelper::loop_detection(const int end, map< int, Matrix4f> rots,
    double dist, int& first, int& last){
    static double min_dist = -1;
    int state = 0;

    for (int i = end - 1; i > 0; i--) {
        Eigen::Vector4f cstart, cend;
        //TODO use pose of scan
        //pcl::compute3DCentroid(*(clouds[i]), cstart);
        //pcl::compute3DCentroid(*(clouds[end]), cend);
        MatHelper::matrix42vector4((rots[i]), cstart);
        MatHelper::matrix42vector4((rots[end]), cend);

        Eigen::Vector4f diff = cend - cstart;

        double norm = diff.norm();

        //std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

        if (state == 0 && norm > dist) {
            state = 1;
            //std::cout << "state 1" << std::endl;
        }
        if (state > 0 && norm < dist) {
            state = 2;
            //std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
            if (min_dist < 0 || norm < min_dist) {
                min_dist = norm;
                first = i;
                last = end;
            }
        }
    }
    //std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
    if (min_dist > 0 && (state < 2 /*|| end == int(clouds.size()) - 1*/)) //TODO
    {
        min_dist = -1;
        return true;
    }
    return false;
}


void PointCloudHelper::find_nearest_point(CloudPtr cloud_src, CloudPtr cloud_dst,
    CloudPtr cloud_sim){
    cloud_sim->clear();
    pcl::KdTreeFLANN< CloudItem>kdtree;
    kdtree.setInputCloud(cloud_src);
    for (int i = 0; i < cloud_dst->size(); i++) {
        std::vector< int>pointIdxNKNSearch(1);
        std::vector< float>pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(cloud_dst->points[i], 1, pointIdxNKNSearch,
            pointNKNSquaredDistance) == 0)
            continue;
        cloud_sim->points.push_back(cloud_src->points[pointIdxNKNSearch[0]]);
    }
}

void PointCloudHelper::set_zero_z(CloudPtr cloud_src){
    return;
    for (int i = 0; i < cloud_src->size(); i++) {
        cloud_src->points[i].z = 0;
    }
}

void PointCloudHelper::build_face(CloudPtr cloud_src){
    return;
    CloudPtr cloudTemp2(new Cloud);
    for (int i = 0; i < cloud_src->size(); i++) {
        //point_cloud_ptr->points[i].z = 0;
        for (int j = 0; j < 10; j++) {
            CloudItem tempPoint;
            tempPoint.x = cloud_src->points[i].x;
            tempPoint.y = cloud_src->points[i].y;
            tempPoint.rgb = cloud_src->points[i].rgb;
            tempPoint.z = 0.1 * j;
            //tempPoint.z = 0;
            cloudTemp2->push_back(tempPoint);
        }
    }
    for (float theta = 0; theta <= 360; theta += 0.5) {
        for (float r = 0; r < 5; r += 0.5) {
            float theta2 = theta / 180. * M_PI;
            CloudItem tempPoint;
            tempPoint.x = r * cos(theta2);
            tempPoint.y = r * sin(theta2);
            tempPoint.z = 0;
            tempPoint.rgb = 0;
            cloudTemp2->push_back(tempPoint);
        }
    }

    pcl::copyPointCloud(*cloudTemp2, *cloud_src);
}

void PointCloudHelper::remove_duplicate(CloudPtr cloudSrc, const float voxel_grid_size){
    CloudPtr cloudTemp2(new Cloud);
    pcl::copyPointCloud(*cloudSrc, *cloudTemp2);
    pcl::VoxelGrid< CloudItem> vox_grid;
    vox_grid.setInputCloud(cloudTemp2);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*cloudSrc);
    //pcl::copyPointCloud(*cloudTemp2, *cloudSrc);
}

void PointCloudHelper::remove_duplicate(CloudPtr cloudSrc, const float voxel_grid_size, int min_num){
    CloudPtr cloudTemp2(new Cloud);
    //pcl::copyPointCloud(*cloudSrc, *cloudTemp2);
    pcl::VoxelGrid< CloudItem> vox_grid;
    vox_grid.setInputCloud(cloudSrc);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*cloudTemp2);
    if (cloudTemp2->size() >= min_num)
    {
        pcl::copyPointCloud(*cloudTemp2, *cloudSrc);
    }
}

float PointCloudHelper::cloudi2cloud(CloudIConstPtr cloud_src, CloudPtr cloud_out,
    int sample_num, bool is_line, float out_side, int min_ins){
    if (min_ins != -1) {
        cout << "min allowed intensity < " << min_ins << endl;
    }
    cloud_out->clear();
    int idx = 0;
    if (is_line) {
        sample_num = 1;
    }
    float max_dis = 0;
//#pragma omp parallel for
    for (int i = 0; i < cloud_src->points.size(); i++) {
        if (idx % sample_num == 0) {
            CloudItem tempPoint;
            tempPoint.x = cloud_src->points[i].x;
            tempPoint.y = cloud_src->points[i].y;
            tempPoint.z = cloud_src->points[i].z;
            if (is_line) {
                if (tempPoint.z != 0) {
                    continue;
                }
            }
            //if (tempPoint.x>0)
            //{
            //  continue;
            //}
            if (min_ins != -1 && cloud_src->points[i].intensity < min_ins) {
                continue;
            }

            uint32_t rgb = intensity2color(100, cloud_src->points[i].intensity);//(static_cast<uint32_t>(r) << 16 |
            tempPoint.rgb = *reinterpret_cast< float*>(&rgb);
            //if (tempPoint.z<0.5 /*|| tempPoint.z>1.3*/)
            //{
            //  continue;
            //}
            //if (tempPoint.z!=0)
            //{
            //  continue;
            //}
            //tempPoint.rgb = rgb;
            //if (Point2OriginDis(tempPoint.x, tempPoint.y, tempPoint.z) < 50
            //  && Point2OriginDis(tempPoint.x, tempPoint.y, tempPoint.z)>1)
            //  cloudOut->push_back(tempPoint);

            //if (tempPoint.z > 1.3) {
            //if (tempPoint.z < 0 ) {
            //    tempPoint.rgb = *reinterpret_cast< float*>(&rgbBlack);
            //}

            //jake todo 2 0.5
            float dis = point2origin_dis(tempPoint.x, tempPoint.y, tempPoint.z);
            if (dis > max_dis)
            {
                max_dis = dis;
            }
//#pragma omp critical
            if (dis > 1) {
                if (out_side <= 0) {
                    cloud_out->push_back(tempPoint);
                } else {
                    if (dis < out_side) {
                        cloud_out->push_back(tempPoint);
                    }
                }
            }
        }
        idx++;
    }
    cloud_out->header.stamp = cloud_src->header.stamp;
    return max_dis;
}

void PointCloudHelper::clear_trimesh_cache()
{
    g_cache_num = 0;
    if (g_trimesh_cache != NULL)
    {
        delete g_trimesh_cache;
        g_trimesh_cache = NULL;
    }
    if (g_kdtree != NULL)
    {
        delete g_kdtree;
        g_kdtree = NULL;
    }
}

float PointCloudHelper::get_rot_icp_cache(CloudPtr cloud_src, CloudPtr cloud_temp,
    Matrix4f& mat_rot){
        //if (cloud_src->size() < 100000)// || cloud_temp->size() < 4000)
        //{
        //    PointCloudHelper::clear_trimesh_cache();
        //}
		if (!is_super_cache)
		{
			if (g_cache_num >= 5)
			{
				clear_trimesh_cache();
			}
		}
        int verbose = 0;
        bool do_scale = false;
        bool do_affine = false;
        bool bulkmode = false;

        TriMesh::set_verbose(verbose);
        TriMesh* mesh1 = NULL;
        KDtree* kd1 =NULL;
        if (g_trimesh_cache != NULL)
        {
            mesh1 = g_trimesh_cache;
            kd1 = g_kdtree;
            g_cache_num++;
        }else{
            mesh1 = new TriMesh();
            mesh1->vertices.resize(cloud_src->size());
            for (int i = 0; i < cloud_src->size(); i++) {
                mesh1->vertices[i] = point(cloud_src->points[i].x, cloud_src->points[i].y,
                    cloud_src->points[i].z);
            }
            kd1 = new KDtree(mesh1->vertices);
            g_trimesh_cache = mesh1;
            g_kdtree = kd1;
        }

        TriMesh* mesh2 = new TriMesh();
        mesh2->vertices.resize(cloud_temp->size());
        for (int i = 0; i < cloud_temp->size(); i++) {
            mesh2->vertices[i] = point(cloud_temp->points[i].x, cloud_temp->points[i].y,
                cloud_temp->points[i].z);
        }


        xform xf1;
        xform xf2;

        KDtree* kd2 = new KDtree(mesh2->vertices);
        vector< float> weights1, weights2;

        if (bulkmode) {
            float area1 = mesh1->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
            float area2 = mesh2->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
            float overlap_area, overlap_dist;
            find_overlap(mesh1, mesh2, xf1, xf2, kd1, kd2, overlap_area, overlap_dist);
            float frac_overlap = overlap_area / min(area1, area2);
            if (frac_overlap < 0.1f) {
                TriMesh::eprintf("Insufficient overlap\n");
                exit(1);
            } else {
                TriMesh::dprintf("%.1f%% overlap\n", frac_overlap * 100.0);
            }
        }
        float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2, 0, verbose,
            do_scale, do_affine);
        //if (err >= 0.0f)
        //  err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
        //  verbose, do_scale, do_affine);

        if (err < 0.0f) {
            TriMesh::eprintf("ICP failed\n");
            //exit(1);
        }

        //TriMesh::eprintf("ICP succeeded - distance = %f\n", err);
        //delete kd1;
        delete kd2;
        //delete mesh1;
        delete mesh2;

        if (bulkmode) {
            xform xf12 = inv(xf2) * xf1;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    mat_rot(i, j) = xf12[i + 4 * j];
                }
            }
        } else {
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    mat_rot(i, j) = xf2[i + 4 * j];
                }
            }
        }

        if (err > 0.12 || err == -1)
        {
            PointCloudHelper::clear_trimesh_cache();
			if (is_super_cache)
			{
				return get_rot_icp(cloud_src,cloud_temp,mat_rot);
			}
        }
        return err;
}
float PointCloudHelper::get_rot_icp(CloudPtr cloud_src, CloudPtr cloud_temp,
    Matrix4f& mat_rot, bool do_scale, bool do_affine){
    int verbose = 0;
    bool bulkmode = false;

    TriMesh::set_verbose(verbose);
    TriMesh* mesh1 = new TriMesh();
    TriMesh* mesh2 = new TriMesh();
    mesh1->vertices.resize(cloud_src->size());
    mesh2->vertices.resize(cloud_temp->size());
    for (int i = 0; i < cloud_src->size(); i++) {
        mesh1->vertices[i] = point(cloud_src->points[i].x, cloud_src->points[i].y,
            cloud_src->points[i].z);
    }
    for (int i = 0; i < cloud_temp->size(); i++) {
        mesh2->vertices[i] = point(cloud_temp->points[i].x, cloud_temp->points[i].y,
            cloud_temp->points[i].z);
    }


    xform xf1;
    xform xf2;

    KDtree* kd1 = new KDtree(mesh1->vertices);
    KDtree* kd2 = new KDtree(mesh2->vertices);
    vector< float> weights1, weights2;

    if (bulkmode) {
        float area1 = mesh1->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
        float area2 = mesh2->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
        float overlap_area, overlap_dist;
        find_overlap(mesh1, mesh2, xf1, xf2, kd1, kd2, overlap_area, overlap_dist);
        float frac_overlap = overlap_area / min(area1, area2);
        if (frac_overlap < 0.1f) {
            TriMesh::eprintf("Insufficient overlap\n");
            exit(1);
        } else {
            TriMesh::dprintf("%.1f%% overlap\n", frac_overlap * 100.0);
        }
    }
    float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2, 0, verbose,
            do_scale, do_affine);
    //if (err >= 0.0f)
    //  err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
    //  verbose, do_scale, do_affine);

    if (err < 0.0f) {
        TriMesh::eprintf("ICP failed\n");
        //exit(1);
    }

    //TriMesh::eprintf("ICP succeeded - distance = %f\n", err);
    delete kd1;
    delete kd2;
    delete mesh1;
    delete mesh2;

    if (bulkmode) {
        xform xf12 = inv(xf2) * xf1;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat_rot(i, j) = xf12[i + 4 * j];
            }
        }
    } else {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat_rot(i, j) = xf2[i + 4 * j];
            }
        }
    }
    return err;
}
//float PointCloudHelper::get_rot_icp2(CloudPtr cloud_src, CloudPtr cloud_temp,
//    Matrix4f& mat_rot){
//    int verbose = 1;
//    bool do_scale = false;
//    bool do_affine = true;
//    bool bulkmode = false;
//
//    TriMesh::set_verbose(verbose);
//    g_cache_icp_num++;
//    TriMesh* mesh1 = NULL;
//    if (g_cache_icp_num > 5 && g_cache_icp_num % 5 != 0) {
//    } else {
//        delete g_cloud_icp_cache;
//        g_cloud_icp_cache = NULL;
//        g_cloud_icp_cache = new TriMesh();
//        g_cloud_icp_cache->vertices.resize(cloud_src->size());
//        for (int i = 0; i < cloud_src->size(); i++) {
//            g_cloud_icp_cache->vertices[i] = point(cloud_src->points[i].x,
//                cloud_src->points[i].y, cloud_src->points[i].z);
//        }
//    }
//    mesh1 = g_cloud_icp_cache;
//
//    TriMesh* mesh2 = new TriMesh();
//    mesh2->vertices.resize(cloud_temp->size());
//    for (int i = 0; i < cloud_temp->size(); i++) {
//        mesh2->vertices[i] = point(cloud_temp->points[i].x, cloud_temp->points[i].y,
//            cloud_temp->points[i].z);
//    }
//
//
//    xform xf1;
//    xform xf2;
//
//    KDtree* kd1 = new KDtree(mesh1->vertices);
//    KDtree* kd2 = new KDtree(mesh2->vertices);
//    vector< float> weights1, weights2;
//
//    if (bulkmode) {
//        float area1 = mesh1->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
//        float area2 = mesh2->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
//        float overlap_area, overlap_dist;
//        find_overlap(mesh1, mesh2, xf1, xf2, kd1, kd2, overlap_area, overlap_dist);
//        float frac_overlap = overlap_area / min(area1, area2);
//        if (frac_overlap < 0.1f) {
//            TriMesh::eprintf("Insufficient overlap\n");
//            exit(1);
//        } else {
//            TriMesh::dprintf("%.1f%% overlap\n", frac_overlap * 100.0);
//        }
//    }
//    float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2, verbose,
//            do_scale, do_affine);
//    if (err >= 0.0f)
//        err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2, verbose, do_scale,
//            do_affine);
//
//    if (err < 0.0f) {
//        TriMesh::eprintf("ICP failed\n");
//        //exit(1);
//    }
//
//    TriMesh::eprintf("ICP succeeded - distance = %f\n", err);
//    delete kd1;
//    delete kd2;
//    delete mesh2;
//
//    if (bulkmode) {
//        xform xf12 = inv(xf2) * xf1;
//        for (int i = 0; i < 4; i++) {
//            for (int j = 0; j < 4; j++) {
//                mat_rot(i, j) = xf12[i + 4 * j];
//            }
//        }
//    } else {
//        for (int i = 0; i < 4; i++) {
//            for (int j = 0; j < 4; j++) {
//                mat_rot(i, j) = xf2[i + 4 * j];
//            }
//        }
//    }
//    return err;
//}


/*
void ICPPackage(CloudPtr cloudSrc, CloudPtr cloudTemp, Matrix4f & pairTransform){
int32_t dim = 3;
Matrix R = Matrix::eye(dim);
Matrix t(dim, 1);
int32_t num1 = cloudSrc->points.size();
int32_t num2 = cloudTemp->points.size();
double* M = (double*)calloc(3 * num1, sizeof(double));
double* T = (double*)calloc(3 * num2, sizeof(double));
for (int i = 0; i < num1; i++){
M[3 * i + 0] = cloudSrc->points[i].x;
M[3 * i + 1] = cloudSrc->points[i].y;
M[3 * i + 2] = cloudSrc->points[i].z;
}
for (int i = 0; i < num2; i++){
T[3 * i + 0] = cloudTemp->points[i].x;
T[3 * i + 1] = cloudTemp->points[i].y;
T[3 * i + 2] = cloudTemp->points[i].z;
}
IcpPointToPoint icp(M, num1, dim);
icp.fit(T, num2, R, t, -1);

Matrix3to4(pairTransform, R, t);
free(M);
free(T);
}
*/

void PointCloudHelper::get_segment_plane(CloudPtr point_cloud_ptr, CloudPtr pcPlane){
    /*
    //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选
    seg.setOptimizeCoefficients(false);
    // 必选
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    // 创建滤波器对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    //// 当还有30%原始点云数据时
    //while (cloud_filtered->points.size() > 0.3 * nr_points)
    //{
        // 从余下的点云中分割最大平面组成部分
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // 分离内层
        extract.setInputCloud(point_cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*pcPlane);
        i++;
    //}
    */
    for (int i = 0; i < point_cloud_ptr->size(); i++) {
        float z = point_cloud_ptr->points[i].z;
        if (z > 0.5 && z < 1.3) {
            CloudItem tempPoint = point_cloud_ptr->points[i];
            /*int b = 0;
                                                    int g = 255;
                                                    int r = 0;
                                                    uint32_t rgb=(static_cast<uint32_t>(r) << 16 |
                                                        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                                                    tempPoint.rgb = rgb;*/
            pcPlane->push_back(tempPoint);
        }
    }
}

void PointCloudHelper::get_reduced_cloud(CloudPtr cloud_tras, Matrix4f cur_out){
    //int ignoreNum = 100000;
    //std::vector<CloudItem, Eigen::aligned_allocator<CloudItem> > vTemp;
    //vTemp.resize(ignoreNum);
    //memcpy(&(vTemp), &(cloudTras->points[cloudTras->points.size() - ignoreNum - 1]), ignoreNum*sizeof(CloudItem));
    //cloudTras->points.resize(ignoreNum);
    //memcpy(&cloudTras->points[0], &vTemp, ignoreNum*sizeof(CloudItem));
    vector< int> Indx;
    int irow = cur_out(0, 3);
    int icol = cur_out(1, 3);
    if (cloud_tras->points.size() > 100000) {
        for (int i = cloud_tras->points.size() / 3; i < cloud_tras->points.size(); i++) {
            //if (abs(cloudTras->points[i].x - irow)<40 && abs(cloudTras->points[i].y - icol)<40)
            //{
            Indx.push_back(i);
            //}
        }
        pcl::copyPointCloud(*cloud_tras, Indx, *cloud_tras);
    }
}


void PointCloudHelper::get_cloud_line(const CloudPtr cloud_src, CloudPtr cloud_out,
    int level){
    cloud_out->clear();
    cout << (-2 + 0.1 * level) << endl;
    //#pragma omp parallel for
    for (int i = 0; i < cloud_src->size(); i++) {
        CloudItem item = cloud_src->points[i];
        if (abs(item.z - (-4 + 0.1 * level)) < 0.1) {
            CloudItem itemTemp = item;
            //itemTemp.z = 0;
            cloud_out->points.push_back(itemTemp);
        }
    }
}


//点(x0,y0,z0),平面Ax+By+Cz+D=0
double PointCloudHelper::point2plane_dis(double x0, double y0, double z0){
    double A(-0.202),B(0.0044),C(0.3463),D(1.0);
    return abs(A * x0 + B * y0 + C * z0 + D) / sqrt((double) A * A + B * B + C * C);
}

//点(x0,y0,z0),到原点的距离
double PointCloudHelper::point2origin_dis(double x0, double y0, double z0){
    return sqrt(x0 * x0 + y0 * y0 + z0 * z0);
}
uint32_t PointCloudHelper::intensity2color(int max_weight, int intensity_data){
    //cout << intensityData << endl;
    uint8_t r(0),g(0),b(0);
    double CaseDistrib = (double) intensity_data / (double) max_weight;

    int max_reg = 220;
    if (CaseDistrib < 0) {
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
            static_cast< uint32_t>(b));
    } else if (CaseDistrib <= 0.2) {
        b = CaseDistrib / 0.2 * 255;
        g = 0;
        r = 0;
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
              static_cast< uint32_t>(b));
    } else if (CaseDistrib <= 0.4) {
        b = 255;
        g = (CaseDistrib - 0.2) / 0.2 * 255;
        r = 0;
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
              static_cast< uint32_t>(b));
    } else if (CaseDistrib <= 0.6) {
        b = (-5 * CaseDistrib + 3) * 255;
        g = (-2.5 * CaseDistrib + 2) * 255;
        r = 0;
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
              static_cast< uint32_t>(b));
    } else if (CaseDistrib <= 0.7) {
        b = (CaseDistrib - 0.6) / 0.2 * 255;
        g = (2.5 * CaseDistrib - 1) * 255;
        r = 0;
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
              static_cast< uint32_t>(b));
    } else if (CaseDistrib <= 1.0) {
        b = 0;
        g = (-5 * CaseDistrib + 5) * 255;
        r = max_reg;
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
              static_cast< uint32_t>(b));
    } else {
        return (static_cast< uint32_t>(r) << 16 | static_cast< uint32_t>(g) << 8 |
              static_cast< uint32_t>(b));
    }
}

float PointCloudHelper::get_cloud_max_width(Cloud& cloud){
		CloudItem pt_max;
		pt_max.x = 0; 
		pt_max.y = 0; 
		pt_max.z = 0; 
		
		for (int i = 0; i < cloud.points.size(); i++) {
			CloudItem pt_temp;
			pt_temp.x = cloud.points[i].x;
			pt_temp.y = cloud.points[i].y;
			pt_temp.z = cloud.points[i].z;
			pt_temp.rgba = cloud.points[i].rgba;
			if (fabs(pt_temp.x) >= pt_max.x){
				pt_max.x=pt_temp.x;
			}
			if (fabs(pt_temp.y) >= pt_max.y){
				pt_max.y=pt_temp.y;
			}
			if (fabs(pt_temp.z) >= pt_max.z){
				pt_max.z=pt_temp.z;
			}
	
		}
		return max(max(pt_max.x,pt_max.y),pt_max.z);
}


float PointCloudHelper::get_cloud_max_width_rate(Cloud& cloud,float width){
	CloudItem pt_max;
	pt_max.x = 0; 
	pt_max.y = 0; 
	pt_max.z = 0; 
	
	int num=0;
	for (int i = 0; i < cloud.points.size(); i++) {
		CloudItem pt_temp;
		float dis=0;
		pt_temp.x = cloud.points[i].x;
		pt_temp.y = cloud.points[i].y;
		pt_temp.z = cloud.points[i].z;
		pt_temp.rgba = cloud.points[i].rgba;
		dis=sqrt(pow(pt_temp.x,2)+pow(pt_temp.y,2)+pow(pt_temp.z,2));
		if (dis<=width)
		{
			num++;
		}
	}
	float rate = (float)num/cloud.points.size();
	cout << "short " << width << " rate " << rate << endl;
	return rate; 
}
void PointCloudHelper::fix_cloud_cache(Cloud& cloud_cache, float f_ins, 
    Matrix4f& local){
		Cloud cloud_cache_fix;
    cloud_cache_fix.clear();
    CloudItem pt_min;
    CloudItem pt_max;
    pt_min.x = 0; 
    pt_min.y = 0; 
    pt_min.z = 0; 
    pt_max.x = 0; 
    pt_max.y = 0; 
    pt_max.z = 0; 
	pt_min.x -= f_ins;
    pt_min.y -= f_ins;
    pt_min.z -= f_ins;
    pt_max.x += f_ins;
    pt_max.y += f_ins;
    pt_max.z += f_ins;
    pt_min.x += local(0, 3);
    pt_min.y += local(1, 3);
    pt_min.z += local(2, 3);
    pt_max.x += local(0, 3);
    pt_max.y += local(1, 3);
    pt_max.z += local(2, 3);
    for (int i = 0; i < cloud_cache.points.size(); i++) {
        CloudItem pt_temp;
        pt_temp.x = cloud_cache.points[i].x;
        pt_temp.y = cloud_cache.points[i].y;
        pt_temp.z = cloud_cache.points[i].z;
        pt_temp.rgba = cloud_cache.points[i].rgba;
        if (pt_temp.x >= pt_min.x && pt_temp.x <= pt_max.x && pt_temp.y >= pt_min.y &&
            pt_temp.y <= pt_max.y && pt_temp.z >= pt_min.z && pt_temp.z <= pt_max.z) {
            cloud_cache_fix.push_back(pt_temp);
        }
    }
    //cout << " fix cache min "<<pt_min.x <<" "<<pt_min.y <<" "<<pt_min.z<<endl;
    //cout << " fix cache max "<<pt_max.x <<" "<<pt_max.y <<" "<<pt_max.z<<endl;
    //cout << " fix cache cloud size "<<cloud_cache_fix.size()<<endl;
    if (cloud_cache_fix.size() > 2000) {
        pcl::copyPointCloud(cloud_cache_fix,cloud_cache);
    }else
	{
        cout << "cloud cache fix error size: " << cloud_cache_fix.size() << endl;
	}
}

void PointCloudHelper::reduce_cloud(Cloud& cloud, float f_ins){
	Cloud cloud_cache_fix;
	for (int i = 0; i < cloud.points.size(); i++) {
		CloudItem pt_temp;
		pt_temp.x = cloud.points[i].x;
		pt_temp.y = cloud.points[i].y;
		pt_temp.z = cloud.points[i].z;
		pt_temp.rgba = cloud.points[i].rgba;
		float dis = point2origin_dis(pt_temp.x, pt_temp.y, pt_temp.z);
		if (dis <= f_ins)
		{
			cloud_cache_fix.push_back(pt_temp);
		}
	}
	pcl::copyPointCloud(cloud_cache_fix,cloud);
}
} // cloud_icp_reg