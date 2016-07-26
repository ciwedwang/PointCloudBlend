#pragma once
#include "stdafx.h"
#include <Eigen/Eigen>
namespace cloud_icp_reg {
using namespace Eigen;
class MovementModel {
public:
    MovementModel();
    ~MovementModel();
    bool UpdataModel(Matrix4f& icptrans,float max_dis);
    bool UpdateModelLocal(Matrix4f& icptrans);
    bool SaveModelLocal(Matrix4f& icptrans);
    void SetEnable(bool enable){
        isEnabled = enable;
    };
    void clear(){
        _trans_list.clear();
    }
private:
    //�˶���б��
    float m_Heading;
    //�˶����ٶȣ�һ֡�ľ���
    float m_distence;
    //��ת���󻺴�
    vector< Matrix4f> _trans_list;
    void Matrix4f2RotateVector(Matrix4f& inputMx, double* rotatevc);
    void RotateVector2Matrix4(Matrix4f& outputMx, double* rotatevc);
    bool KalmanFilter(Matrix4f& inputMx);
    Matrix4f PrevMatrix4f;
    double PrevGradient[5];
    bool isEnabled;
	int num_error;
};
} // cloud_icp_reg