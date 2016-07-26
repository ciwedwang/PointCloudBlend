/*
*   ��������: ���������ȡ��
*            ��������б��̬����ά���Ƴ���,����Ϊ���淨����ƽ��z�᷽��,
*            ����ƽ����XOYƽ��,������ָ���ĸ߶ȷ�Χ��ȡ��ͬ��������
*            
*/
#pragma once
#include <Eigen/Eigen>
#include <opencv/cv.h>
#include "cmm_types.h"

namespace cloud_icp_reg {
/*
*   ��������: ����rotate_arbitrary_line�����ȡ��
*/
class ProjectPlane {
public:
    /*
    *  ����˵��: cloud_src �����������
    *   ��������: ����һ�����������ȡ��
    */
    explicit ProjectPlane(const Cloud& cloud_src, const Cloud& cloud_trace);
    /*
     *  ��������: get_plane  
     *  ����˵��: height_min �����������С�߶�,
                    height_max ������������߶�,
                    cloud_out���صĵ�������
     *  ����ֵ: void
     *  ��������: �����ض��߶ȷ�Χ�ڵĵ�������
     */
    void get_plane(const float height_min, const float height_max, Cloud& cloud_out);
    /*
     *  ��������: get_cloud
     *  ����˵��: cloud_out���صĵ�������
     *  ����ֵ: void 
     *  ��������: ���ط�������z���غϵĵ�������
     */
    void get_cloud(Cloud& cloud_out);
	Eigen::Matrix4f _mat_rot;
private:
    /*
     *  ��������: get_rot
     *  ����˵��: mat_rot_out�õ����������z�᷽�����ת����
     *  ����ֵ: void 
     *  ��������: ��ý����淨��������Ϊz��,����Ҫ����ת����
     */
    void get_rot(Eigen::Matrix4f& mat_rot_out);
    /*
    *  ��������: rotate_arbitrary_line
    *  ����˵��: vec_z_normal��ת������, angle�Ƕ� ,mat_rot_out������ת����
    *  ����ֵ: bool
    *  ��������:������������ת�ض��ĽǶ�,�õ���ת����
    */
    bool rotate_arbitrary_line(CvMat* vec_z_normal, const float angle, Eigen::Matrix4f& mat_rot_out);

    Cloud _cloud;//�洢���������������ĵ�������
    Cloud _cloud_trace;

    DISALLOW_COPY_AND_ASSIGN(ProjectPlane);
};
} // cloud_icp_reg