#include "StdAfx.h"
#include "pcl_pair_visualizer.h"


namespace cloud_icp_reg {
const bool ENABLE_DUAL = true;

void PclPairVisualizer::update_cloud_right(const CloudPtr cloud_source){
    if (!ENABLE_DUAL) {
        return;
    }
    _pv->removePointCloud("vp2_source");
    _pv->addPointCloud(cloud_source, "vp2_source", _n_pv2);
    _pv->spinOnce();
}

void PclPairVisualizer::update_cloud_left(const CloudPtr cloud_source){
    _pv->removePointCloud("vp1_source");
    _pv->addPointCloud(cloud_source, "vp1_source", _n_pv1);
    _pv->spinOnce();
}

PclPairVisualizer::PclPairVisualizer()
        : _n_pv1(0), _n_pv2(1),
    _pv(new pcl::visualization::PCLVisualizer("point cloud view"))
{
    _pv->setCameraPose(0, 0, 50, 0, 0, 0, 0, 0, 0, 0);
    _pv->addCoordinateSystem();
    if (ENABLE_DUAL) {
        _pv->createViewPort(0.0, 0, 0.5, 1.0, _n_pv1);
        _pv->createViewPort(0.5, 0, 1.0, 1.0, _n_pv2);
    } else {
        _pv->createViewPort(0.0, 0, 1, 1.0, _n_pv1);
    }
}
}//cloud_icp_reg
