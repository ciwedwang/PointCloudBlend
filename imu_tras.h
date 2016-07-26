/*
*   ��������: imu��̬��ԭ��
*            ���ݼ�¼��imu����,����ǰ������̬��ԭΪ������һ֡����̬,�Ӷ��ų����ֶ�������
*            imu��Ƶ�ʷǳ���,��������,��֪���Ƶ�ʱ���,��Ҫ��imu���ҳ���ӽ����Ƶ�ʱ���
*            �Ƚ��д����С,�ٽ�������ӽ���ʱ���
*            Ϊ�˱���imu���ۼ����,�������������һ����֡����̬�仯
*/
#pragma once
#include <boost/unordered_map.hpp>
#include <set>
#include <Eigen/Eigen>
#include "cmm_types.h"

namespace cloud_icp_reg {
/*
*   ��������: imu��̬��ԭ��,��ԭ������̬,�ų�����
*/
class ImuTras {
public:
    /*
    *  ����˵��: str_path imu�ļ�·��
    *   ��������: ����һ��imu��̬��ԭ��
    */
    explicit ImuTras(std::string str_path);

    ImuTras()
    {

    }
    /*
     *  ��������: trans_cloud
     *  ����˵��: cloud_src�����������,
     *           cloud_out��������仯��ĵ�������
     *  ����ֵ: bool �������Ƿ��ҵ���Ӧ�ĵ�������
     *  ��������: ����һ֡��������,��������̬��ԭ
     */
    bool trans_cloud(const Cloud& cloud_src, Cloud& cloud_out);
    /*
    *  ��������: get_transaction_mx
    *  ����˵��: stamp_cloud �������ʱ���,
    *           mat_rot ����������һ֡����תƽ�ƾ���,
    *           d_yaw����������һ֡��yawֵ,
    *           d_pitch ����������һ֡��pitchֵ,
    *           d_roll ����������һ֡��rollֵ,
    *  ����ֵ: bool �����Ƿ��ҵ������imu��Ӧ
    *  ��������: ����ʱ���,���ػ�����һ֡����������̬
    */
    bool get_transaction_mx(const uint64_t stamp_cloud, Eigen::Matrix4f& mat_rot, double& d_yaw,
        double& d_pitch, double& d_roll);

	bool ImuTras::get_transaction_mx_mcs(const uint64_t stamp_cloud, Eigen::Matrix4f& mat_rot,
		double& d_yaw, double& d_pitch, double& d_roll);

    inline void get_mid_mt(cv::Point3d& pt_valid, uint64_t stamp, uint64_t stamp1, uint64_t stamp2);
    
    inline Eigen::Matrix4f pt2mat(const cv::Point3d& pt_gradient);

    static void get_matrix(float *cg, double a, double b, double r, double z, double y, double x);
    /*
     *  ��������: ����imu��̬��ԭ��
     *  ����˵��: ��
     *  ����ֵ: bool 
     *  ��������: ����imu�����Ƿ�����
     */
    bool is_valid() const{
        return _is_valid;
    };

private:

    /*
     *  ��������: get_hash_key
     *  ����˵��: stamp �����΢��ʱ���
     *  ����ֵ: uint64_t 
     *  ��������: ��1/10������Ϊmap��keyֵ,�����������ĵ��ƻ��ֳɶ������
     */
    uint64_t get_hash_key(uint64_t stamp) const{
        return stamp / 100000;
    }

    boost::unordered_map< uint64_t, std::set< uint64_t> > _map_mt_idx;//��imu���ݻ�������
    boost::unordered_map< uint64_t, cv::Point3d> _map_mt;//imu��̬���ݼ���
    Eigen::Matrix4f _last_mat;//��һ����ĵ���֡����̬
    bool _is_valid;//����imu�����Ƿ�����
    uint64_t _last_ms;//��¼��һ������ĺ�����,��������Խ1Сʱ���,��Ϊ���Ʋ��߱�Сʱʱ���
    int _last_hour;//��¼����Խ��Сʱ��
    cv::Point3d _last_ori;//��¼��һ֡����ľ�����̬
	cv::Point3d _last_read_point;//��¼��һ�ζ�ȡ��imu����
	bool _is_last_stamp_error;//��¼��һ֡�ǲ��ǳ����,��ֹ���������¼

    //DISALLOW_COPY_AND_ASSIGN(ImuTras);
};
} // cloud_icp_reg