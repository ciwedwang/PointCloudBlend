#include "StdAfx.h"
#include "process_status.h"


namespace cloud_icp_reg {
ProcessStatus::ProcessStatus(void)
        : _pcap_file(""), _global_transform(Eigen::Matrix4f::Identity()), _frame_count_total(0),
    _frame_count_process(0), _skip_frame(0), _start_frame(0), _rotate_count(0),
    _is_write_stamp(true), _last_mt(Eigen::Matrix4f::Identity()), _cloud_src(new Cloud),
    _cloud_result(new Cloud), _cloud_cache(new Cloud), _is_batch_mode(false),
	_cloud_temp_src_last(new Cloud),_max_dis(-1),_num_short(0)
{
}

ProcessStatus::~ProcessStatus(void){
}


ProcessLoopDetectionStatus::ProcessLoopDetectionStatus(void)
    :last_timestamp_start(0),
    last_timestamp_end(0),
    is_perform_search(false),
    search_last_distance(0),
    search_last_timestamp(0),
    track_cloud(new PointCloud<PointXYZ>()),
    sleep_count(0),
    min_cloudindex(0),
    last_timestamp(0)
{
}

ProcessLoopDetectionStatus::~ProcessLoopDetectionStatus(void)
{
}
}//cloud_icp_reg