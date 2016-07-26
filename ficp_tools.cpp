#include "stdafx.h"
#include "ficp_tools.h"
#include <pcl/io/pcd_io.h>
#include "gpc.h"

namespace cloud_icp_reg {
using boost::unordered_map;

#define LOOPDETECTTHRESHOLD 1.0

int getAllowedLoopIdx(int frame, int ignoreCount){
    if (abs(frame) - ignoreCount <= 0) {
        frame = 1;
    } else {
        frame = (abs(frame) - ignoreCount) * (frame / abs(frame));
    }
    return frame;
}


void get_frame_idx_vector(unordered_map< int, Matrix4f>& mapRots, list< int>& list){
    unordered_map< int, Matrix4f>::iterator it;
    for (it = mapRots.begin(); it != mapRots.end(); it++) {
        list.push_back(it->first);
    }
    list.sort();
}

void get_frame_stamps(unordered_map< uint64, Matrix4f>& map_rot_stamps, list< uint64>& list){
	unordered_map< uint64, Matrix4f>::iterator it;
	for (it = map_rot_stamps.begin(); it != map_rot_stamps.end(); it++) {
		list.push_back(it->first);
	}
	list.sort();
}

int get_glass_min_ins(uint64_t stamp, map< uint64_t, uint64_t>& glassStamp){
    for (map< uint64_t, uint64_t>::iterator it = glassStamp.begin(); it != glassStamp.end();
        it++) {
        if (stamp >= it->first && stamp<= it->second) {
            return 10;
        }
    }
    return -1;
}

bool check_ignore_frame_valid(uint64_t stamp, map< uint64_t, uint64_t>& ignoreStamp){
    for (map< uint64_t, uint64_t>::iterator it = ignoreStamp.begin(); it != ignoreStamp.end();
        it++) {
        if (stamp >= it->first && stamp <= it->second) {
            return false;
        }
    }
    return true;
}


bool check_loop_valid(vector< map< uint64_t, uint64_t> >& mapLoops,
    unordered_map< uint64_t, Matrix4f>& rotStamps) {
    //校验是不是rot中包含了loop对应的stamp
    for (int i = 0; i < mapLoops.size(); i++) {
        map< uint64_t, uint64_t> loop = mapLoops[i];
        bool isFindStart = false, isFindEnd = false;
        for (unordered_map< uint64_t, Matrix4f>::iterator it = rotStamps.begin(); 
            it != rotStamps.end(); it++) {
            if (it->first == loop.begin()->first) {
                isFindStart = true;
            }
            if (it->first == loop.begin()->second) {
                isFindEnd = true;
            }
            if (isFindStart && isFindEnd) {
                break;
            }
        }
        if (isFindStart && isFindEnd) {
        } else {
            cout << "error ... rot empty stamp not valid." << loop.begin()->first << " "
                  << loop.begin()->second << endl;
            return false;
        }
    }
    return true;
}
} // cloud_icp_reg