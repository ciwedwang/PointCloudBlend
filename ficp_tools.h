#pragma once

#include "cmm_types.h"
#include <boost/unordered_map.hpp>

namespace cloud_icp_reg {
using namespace std;
using namespace trimesh;
using namespace Eigen;




void get_frame_idx_vector(boost::unordered_map< int, Matrix4f>& mapRots, list< int>& list);
void get_frame_stamps(boost::unordered_map< uint64, Matrix4f>& map_rot_stamps, list< uint64>& list);
int get_glass_min_ins(uint64_t stamp, map< uint64_t, uint64_t>& glassStamp);
bool check_ignore_frame_valid(uint64_t stamp, map< uint64_t, uint64_t>& ignoreStamp);
bool check_loop_valid(vector< map< uint64_t, uint64_t> >& mapLoops,
    boost::unordered_map< uint64_t, Matrix4f>& rotStamps);
} // cloud_icp_reg