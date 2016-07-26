#pragma once
#include <boost/unordered_map.hpp>

namespace cloud_icp_reg {
class FileHelper {
public:
    FileHelper();
    ~FileHelper();

    //static void ReadCSV_2_PointXYZRGBA(CloudPtr point_cloud_ptr, const char* pname);
    static void write_file_optimize_rot(const int frame, const string pcap_file,
        const Eigen::Matrix4f& trans, bool is_append);
    static void write_file_optimize_rot_stamp(const int frame, const string pcap_file,
        const Eigen::Matrix4f& trans, bool is_append, int64_t stamp = -1);
    static void write_file_no_nptimize_rot_stamp(const int frame, const string pcap_file,
        const Eigen::Matrix4f& trans, bool is_append, int64_t stamp);
    static bool load_file_optimize_rot_stamp(const string pcap_file,
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots, 
        boost::unordered_map< int, int64_t>& stamps);
    static bool load_file(const string pcap_file, const string suffix,
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots);
    static bool load_file_rot(const string pcap_file, 
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots,
        boost::unordered_map< uint64_t, Eigen::Matrix4f>& map_rots_stamp);
    static bool FileHelper::load_file_rot(const string pcap_file, 
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots,
        boost::unordered_map< uint64_t, Eigen::Matrix4f>& map_rots_stamp,
        boost::unordered_map< int, int64_t>& stamps);
    static bool load_file_glass(const string pcap_file, map< uint64_t, uint64_t>& intensity);
    static void write_file(const string filePath, const string content, bool isAppend);
    static bool load_file_rot_new(const string pcap_file, 
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots);
    static void write_file_rot(const int frame, const string pcap_file,
        const Eigen::Matrix4f& trans, bool isAppend, int64_t stamp = -1);
    static bool load_file(const string pcap_file, const string suffix,
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots, 
        boost::unordered_map< int, int64_t>& stamps, bool isContainStamp);
    static bool load_file_house_back(const string pcap_file, list< uint64_t>& stamps);
    static bool load_file_rot(const string pcap_file, 
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots);
	static bool FileHelper::load_file_rot_fix(const string pcap_file, 
		boost::unordered_map< int, Eigen::Matrix4f>& map_rots,
		boost::unordered_map< uint64_t,Eigen::Matrix4f>& map_rots_stamp);
	static bool FileHelper::load_file_rot_fix(const string pcap_file, 
		boost::unordered_map< int, Eigen::Matrix4f>& map_rots,
		boost::unordered_map< uint64_t, Eigen::Matrix4f>& map_rots_stamp,
		boost::unordered_map< int, int64_t>& stamps);
    static bool load_file_loop(const string& pcap_file,
        vector< pair< pair<uint64_t, uint64_t>, Eigen::Matrix4f> >& map_loops);
    static bool load_file_loop2(const string& pcap_file,
        vector< pair< pair<uint64_t, uint64_t>, Eigen::Matrix4f> >& map_loops);
    static bool load_file_loop_ignore(const string &pcap_file,
        vector<pair<uint64_t, uint64_t> >& map_ignore_loops);
    static bool load_file_optimize_rot(const string pcap_file, 
        boost::unordered_map< int, Eigen::Matrix4f>& map_rots);
    static bool load_file_ignore_frame(const string pcap_file, 
        map< uint64_t, uint64_t>& stamps);
private:
    static bool load_file_loop_core(const string &file_path,
        vector< pair< pair<uint64_t, uint64_t>, Eigen::Matrix4f> >& map_loops);
    static bool load_file_loop_core(const string &file_path,
        vector< pair<uint64_t, uint64_t> > &map_loops);
	static fstream _out_file;
	static string _last_write_path;
};

} // cloud_icp_reg