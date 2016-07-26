#ifndef BEE_POINTCLOUD_POINTICPREG_PCL_PAIR_VISUALIZER_H
#define BEE_POINTCLOUD_POINTICPREG_PCL_PAIR_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "cmm_types.h"

namespace cloud_icp_reg {
class PclPairVisualizer {
public:
    void update_cloud_left(const CloudPtr cloud_source);
    void update_cloud_right(const CloudPtr cloud_source);
    void spin(){
        _pv->spin();
    }
    void spin_once(){
        _pv->spinOnce();
    }
    SINGLETON_CLASS(PclPairVisualizer);
    pcl::visualization::PCLVisualizer* _pv;
private:
    ~PclPairVisualizer(){
        if (_pv){
            delete _pv;
            _pv = NULL;
        }
    }
    DISALLOW_COPY_AND_ASSIGN(PclPairVisualizer);
    int _n_pv1;
    int _n_pv2;
};
}//cloud_icp_reg


#endif //BEE_POINTCLOUD_POINTICPREG_PCL_PAIR_VISUALIZER_H