#include "stdafx.h"
#include "feature_align.h"
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
#include "sample_consensus_prerejective.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace cloud_icp_reg {
// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
double FeatureAlign::do_align_pre(const CloudPtr cloud_src , const CloudPtr cloud_dst, Eigen::Matrix4f& rot)
{
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  for (int i = 0;i <cloud_src->size();i++)
  {
	  CloudItem& item = cloud_src->points[i];
	  PointNT pt;
	  pt.x = item.x; 
	  pt.y = item.y; 
	  pt.z = item.z; 
	  scene->push_back(pt);
  }
  for (int i = 0;i <cloud_dst->size();i++)
  {
	  CloudItem& item = cloud_dst->points[i];
	  PointNT pt;
	  pt.x = item.x; 
	  pt.y = item.y; 
	  pt.z = item.z; 
	  object->push_back(pt);
  }
  
  // Get input object and scene
  //if (argc != 3)
  //{
  //  pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
  //  return (1);
  //}
  
  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  /*  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
  pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
  {
  pcl::console::print_error ("Error loading object/scene file!\n");
  return (1);
  }*/
//  pcl::io::loadPCDFile<PointNT> ("C:\\Users\\jake\\Desktop\\问题数据\\0.pcd", *object);
//  pcl::io::loadPCDFile<PointNT> ("C:\\Users\\jake\\Desktop\\问题数据\\1.pcd", *scene);
  
  // Downsample
  //pcl::console::print_highlight ("Downsampling...\n");
  //pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.15f;
  //grid.setLeafSize (leaf, leaf, leaf);
  //grid.setInputCloud (object);
  //grid.filter (*object);
  //grid.setInputCloud (scene);
  //grid.filter (*scene);
  
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;  
  nest.setRadiusSearch (0.5);
  nest.setInputCloud (object);
  nest.compute (*object);
  nest.setInputCloud (scene);
  nest.compute (*scene);
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (1.5);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputCloud (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  //align.setMaximumIterations (5000); // Number of RANSAC iterations
  align.setMaximumIterations (5000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (0.25); // Inlier threshold
  align.setInlierFraction (0.05f); // Required inlier fraction for accepting a pose hypothesis
  //align.setInlierFraction (0.95f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers().size(), object->size());
	pcl::console::print_info ("fitness: %i\n",align.getFitnessScore());
   
	pcl::console::print_info ("inliner fraction: %f\n",(double)align.getInliers().size()/ (double)object->size()); 
	rot = align.getFinalTransformation();
    // Show alignment
    //pcl::visualization::PCLVisualizer visu("Alignment");
    //visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    //visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    //visu.spin ();
	return (double)align.getInliers().size()/ (double)object->size();
  }
  else
  {
	pcl::console::print_info ("inliner fraction: %f\n",(double)align.getInliers().size()/ (double)object->size()); 
    pcl::console::print_error ("Alignment failed!\n");
	rot = Eigen::Matrix4f::Identity();
	return (double)align.getInliers().size()/ (double)object->size();
  }
  
  return (double)align.getInliers().size()/ (double)object->size();
}
}
