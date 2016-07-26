#pragma once
#include <Eigen/Eigen>
#include <boost/unordered_map.hpp>
#include "cmm_types.h"
#include "hdl_grabber.h"

namespace cloud_icp_reg {
typedef enum {
    ENUM_MODE_PROCESS_POS = 0,
    ENUM_MODE_PROCESS_LOOP,
    ENUM_MODE_PROCESS_LINE,
    ENUM_MODE_PROCESS_LOOP_TRACE,
    ENUM_MODE_PROCESS_LOOP_TRACE_NEW,
    ENUM_MODE_PROCESS_OUT_PUT = 9,
	ENUM_MODE_PROCESS_FIXPOS ,
}enumModeProcess;

class ProcessLoopDetectionStatus {
public:
    ProcessLoopDetectionStatus(void);
    ~ProcessLoopDetectionStatus(void);
    PointCloud<PointXYZ>::Ptr track_cloud;
    uint64_t last_timestamp_start;
    uint64_t last_timestamp_end;
    bool is_perform_search;
    float search_last_distance;
    uint64_t search_last_timestamp;
    uint64_t last_timestamp;
    int sleep_count;
    uint64_t min_cloudindex;
    vector<size_t> track_cloud_index;
private:
    DISALLOW_COPY_AND_ASSIGN(ProcessLoopDetectionStatus);
};

class ProcessStatus {
public:
    ProcessStatus(void);
    ~ProcessStatus(void);
    string _pcap_file;
    bool _is_batch_mode;
    bool _is_debug_mode;
    bool _is_step_mode;
    Eigen::Matrix4f _global_transform;
    int _frame_count_total; //所有处理的帧数,包含跳过的帧数
    int _frame_count_process; //处理的帧数
    int _skip_frame;
    uint64_t _start_frame;
    int _rotate_count;
    //是否写入时间戳
    bool _is_write_stamp;
    Eigen::Matrix4f _last_mt;

    //存放loop用的点云集合
    vector< CloudPtr> _clouds_line;
    boost::unordered_map< int, Eigen::Matrix4f> _map_clouds_rot;
    boost::unordered_map< uint64_t, Eigen::Matrix4f> _map_clouds_rot_stamps;
    boost::unordered_map< int, int64_t> _map_clouds_stamp;
    vector< Eigen::Matrix4f> _optimize_rot;
    queue< CloudPtr> _queue_cloud_tras;
    enumModeProcess _current_mode;
    list< uint64_t> _house_back_stamp;
    map< uint64_t, uint64_t> _glass_stamp;
    map< uint64_t, uint64_t> _ignore_stamp;
    vector< pair< pair<uint64_t, uint64_t>, Eigen::Matrix4f> > _map_loops;
    vector< pair<uint64_t, uint64_t> > _map_ignore_loops;
    //点云数据
    CloudPtr _cloud_src; //存放上一帧数据
    CloudPtr _cloud_result; //存放结果数据
    CloudPtr _cloud_cache;

	CloudPtr _cloud_temp_src_last; //存放上一帧的原始数据
	float _max_dis;
	int _num_short; 
	
	EnumPtDev _current_dev;
    set<uint64_t> _ignore_invalid_stamp;

    ProcessLoopDetectionStatus _loop_detection_status;
private:
    DISALLOW_COPY_AND_ASSIGN(ProcessStatus);
};
}//cloud_icp_reg

