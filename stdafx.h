#pragma once

//#undef _HAS_ITERATOR_DEBUGGING
//#define _HAS_ITERATOR_DEBUGGING 0
//#undef _ITERATOR_DEBUG_LEVEL
//#define _ITERATOR_DEBUG_LEVEL 0
//#undef _SECURE_SCL
//#define _SECURE_SCL 0

#include <cstdio>
#include <cstdlib>
//#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "boost.h"
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv/cv.h>

#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "ICP.h"

#include <string>


//register
#include "hdl_viewer_grabber_helper.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/common/centroid.h>


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>




#include "cmm_types.h"


#include "lum.h"
#include "hdl_viewer_grabber_helper.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <Eigen/Eigen>


