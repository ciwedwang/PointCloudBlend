#include "stdafx.h"
#include "movement_model.h"

namespace cloud_icp_reg {
MovementModel::MovementModel()
        : isEnabled(true),num_error(0)
{
    PrevMatrix4f = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 5; i++) {
        PrevGradient[i] = 0.0;
    }
}


MovementModel::~MovementModel(){
}

//bool MovementModel::UpdataModel(Matrix4f& icptrans){
//    if (isEnabled == false) {
//        return false;
//    }
//    bool flag = true;
//    if (_trans_list.size() == 5) {
//        float diffxx[5] = {
//            0
//        };
//        float diffyy[5] = {
//            0
//        };
//        float diffzz[5] = {
//            0
//        };
//        for (int i = 0; i < 4; i++) {
//            if (_trans_list[i + 1](0, 3) != _trans_list[i](0, 3)) {
//                diffxx[i] = (_trans_list[i + 1](0, 3) - _trans_list[i](0, 3));
//                diffyy[i] = (_trans_list[i + 1](1, 3) - _trans_list[i](1, 3));
//                diffzz[i] = (_trans_list[i + 1](2, 3) - _trans_list[i](2, 3));
//            }
//        }
//        diffxx[4] = (diffxx[0] + diffxx[1] * 2 + diffxx[2] * 3 + diffxx[3] * 7) / 13;
//        diffyy[4] = (diffyy[0] + diffyy[1] * 2 + diffyy[2] * 3 + diffyy[3] * 7) / 13;
//        diffzz[4] = (diffzz[0] + diffzz[1] * 2 + diffzz[2] * 3 + diffzz[3] * 7) / 13;
//
//        //jake no model 
//        //float xdiff = icptrans(0, 3) - _trans_list[4](0, 3);
//        //float ydiff = icptrans(1, 3) - _trans_list[4](1, 3);
//        //float zdiff = icptrans(2, 3) - _trans_list[4](2, 3);
//        //if (abs(xdiff) + abs(ydiff) + abs(ydiff) > 4) {
//        //    flag = false;
//        //    icptrans = _trans_list[4];
//        //    cout << "x y z dif " << sqrt(abs(xdiff) + abs(ydiff) + abs(ydiff)) << endl;
//        //}
//        float xdiff = icptrans(0, 3) - _trans_list[3](0, 3);
//        float ydiff = icptrans(1, 3) - _trans_list[3](1, 3);
//        float zdiff = icptrans(2, 3) - _trans_list[3](2, 3);
//        if (abs(xdiff) + abs(ydiff) + abs(ydiff) > 9) {
//            flag = false;
//            icptrans = _trans_list[3];
//            cout << "x y z dif " << sqrt(abs(xdiff) + abs(ydiff) + abs(ydiff)) << endl;
//            //jake 重大问题
//        //}
//            float moveWeight = 1.0 - (abs(xdiff) + abs(ydiff) + abs(ydiff)) / 2.0;
//            if (moveWeight < 0.0) {
//                moveWeight = 0.0;
//            }
//            icptrans(0, 3) = _trans_list[4](0, 3) + xdiff * moveWeight + diffxx[4] * (1.0 - moveWeight);
//            icptrans(1, 3) = _trans_list[4](1, 3) + ydiff * moveWeight + diffyy[4] * (1.0 - moveWeight);
//            icptrans(2, 3) = _trans_list[4](2, 3) + zdiff * moveWeight + diffzz[4] * (1.0 - moveWeight);
//            //jake 重大问题
//        }
//
//        //jake todo 背包暂时去掉
//        //if (abs(icptrans(2, 2) - 1) > 0.3) {
//        //    flag = false;
//        //    icptrans = _trans_list[4];
//        //}
//        //if (abs(icptrans(2, 2) - transList[4](2, 2)) > 0.3) {
//        //    flag = false;
//        //    icptrans = transList[4];
//        //}
//    }
//    if (_trans_list.size() < 5) {
//        _trans_list.push_back(icptrans);
//    } else {
//        _trans_list.erase(_trans_list.begin());
//        _trans_list.push_back(icptrans);
//    }
//    return flag;
//}

bool MovementModel::UpdataModel(Matrix4f& rot,float max_dis){
    if (isEnabled == false) {
        return false;
    }
	Matrix4f icptrans = rot;
	bool is_need_update = false;
    bool flag = true;
    if (_trans_list.size() == 5) {
        float diffxx[5] = {
            0
        };
        float diffyy[5] = {
            0
        };
        float diffzz[5] = {
            0
        };
        for (int i = 0; i < 4; i++) {
            if (_trans_list[i + 1](0, 3) != _trans_list[i](0, 3)) {
                diffxx[i] = (_trans_list[i + 1](0, 3) - _trans_list[i](0, 3));
                diffyy[i] = (_trans_list[i + 1](1, 3) - _trans_list[i](1, 3));
                diffzz[i] = (_trans_list[i + 1](2, 3) - _trans_list[i](2, 3));
            }
        }
        diffxx[4] = (diffxx[0] + diffxx[1] * 2 + diffxx[2] * 3 + diffxx[3] * 7) / 13;
        diffyy[4] = (diffyy[0] + diffyy[1] * 2 + diffyy[2] * 3 + diffyy[3] * 7) / 13;
        diffzz[4] = (diffzz[0] + diffzz[1] * 2 + diffzz[2] * 3 + diffzz[3] * 7) / 13;

		Matrix4f rot_diff=icptrans * _trans_list[4].inverse();
		float xdiff = icptrans(0, 3) - _trans_list[4](0, 3);
        float ydiff = icptrans(1, 3) - _trans_list[4](1, 3);
        float zdiff = icptrans(2, 3) - _trans_list[4](2, 3);
		float dis2=pow(xdiff,2) + pow(ydiff,2) + pow(zdiff,2);
		cout << "dis2 " << dis2 << endl;
        float dis_allowed = 16; //16
		//jake 取消距离限制
        if (num_error < 2) {
            dis_allowed = 1; //4
		}
        if (dis2 > dis_allowed) {
			cout << _trans_list[4] << endl << icptrans << endl;
			flag = false;
			float moveWeight = 1.0 - (abs(xdiff) + abs(ydiff) + abs(zdiff)) / 2.0;
			if (moveWeight < 0.0) {
				moveWeight = 0.0;
			}
			icptrans(0, 3) = _trans_list[4](0, 3) + xdiff * moveWeight + diffxx[4] * (1.0 - moveWeight);
			icptrans(1, 3) = _trans_list[4](1, 3) + ydiff * moveWeight + diffyy[4] * (1.0 - moveWeight);
			icptrans(2, 3) = _trans_list[4](2, 3) + zdiff * moveWeight + diffzz[4] * (1.0 - moveWeight);
            //icptrans = _trans_list[4];
            cout << "x y z dif " << sqrt(dis2)<< endl;
			is_need_update = true;
        }
        //jake todo 背包暂时去掉
        //if (abs(icptrans(2, 2) - 1) > 0.1) {
        //    flag = false;
        //    icptrans = _trans_list[4];
		//	  is_need_update = true;
        //}
        //if (abs(icptrans(2, 2) - _trans_list[4](2, 2)) > 0.1) {
		cout << "rot diff " << rot_diff(0, 2) << endl;
		//if(abs(rot_diff(0, 2)) > 0.1){
		//	cout << "rot diff failed" << endl;
		//	flag = false;
		//	icptrans = _trans_list[4];
		//	is_need_update = true;
		//}
    }
    if (_trans_list.size() < 5) {
        _trans_list.push_back(icptrans);
    } else {
        _trans_list.erase(_trans_list.begin());
        _trans_list.push_back(icptrans);
    }
	if (is_need_update)
	{
		rot = icptrans;
		num_error++;
	}else{
		num_error = 0;
	}
    return flag;
}

bool MovementModel::UpdateModelLocal(Matrix4f& icptrans){
    if (_trans_list.size() == 5) {
        float diffxx[5] = {
            0
        };
        float diffyy[5] = {
            0
        };
        float diffzz[5] = {
            0
        };
        for (int i = 0; i < 4; i++) {
            if (_trans_list[i + 1](0, 3) != _trans_list[i](0, 3)) {
                diffxx[i] = (_trans_list[i + 1](0, 3) - _trans_list[i](0, 3));
                diffyy[i] = (_trans_list[i + 1](1, 3) - _trans_list[i](1, 3));
                diffzz[i] = (_trans_list[i + 1](2, 3) - _trans_list[i](2, 3));
            }
        }
        diffxx[4] = (diffxx[0] + diffxx[1] * 2 + diffxx[2] * 3 + diffxx[3] * 7) / 13;
        diffyy[4] = (diffyy[0] + diffyy[1] * 2 + diffyy[2] * 3 + diffyy[3] * 7) / 13;
        diffzz[4] = (diffzz[0] + diffzz[1] * 2 + diffzz[2] * 3 + diffzz[3] * 7) / 13;

        //icptrans(0, 3) = transList[4](0, 3) + diffxx[4];
        //icptrans(1, 3) = transList[4](1, 3) + diffyy[4];
        //icptrans(2, 3) = transList[4](2, 3) + diffzz[4];
        icptrans(0, 3) = diffxx[4];
        icptrans(1, 3) = diffyy[4];
        icptrans(2, 3) = diffzz[4];
    }
    return true;
}

bool MovementModel::SaveModelLocal(Matrix4f& icptrans){
    if (_trans_list.size() < 5) {
        _trans_list.push_back(icptrans);
    } else {
        _trans_list.erase(_trans_list.begin());
        _trans_list.push_back(icptrans);
    }
    return true;
}

void MovementModel::Matrix4f2RotateVector(Matrix4f& inputMx, double* rotatevc){
    CvMat* cvMatd = cvCreateMat(3, 3, CV_64F);
    CvMat* cvMatRot = cvCreateMat(3, 1, CV_64F);
    cvMatd->data.db[0] = inputMx(0, 0);
    cvMatd->data.db[1] = inputMx(0, 1);
    cvMatd->data.db[2] = inputMx(0, 2);
    cvMatd->data.db[3] = inputMx(1, 0);
    cvMatd->data.db[4] = inputMx(1, 1);
    cvMatd->data.db[5] = inputMx(1, 2);
    cvMatd->data.db[6] = inputMx(2, 0);
    cvMatd->data.db[7] = inputMx(2, 1);
    cvMatd->data.db[8] = inputMx(2, 2);
    cvRodrigues2(cvMatd, cvMatRot);

    rotatevc[0] = cvMatRot->data.db[0];
    rotatevc[1] = cvMatRot->data.db[1];
    rotatevc[2] = cvMatRot->data.db[2];

    rotatevc[3] = inputMx(0, 3);
    rotatevc[4] = inputMx(1, 3);
    rotatevc[5] = inputMx(2, 3);
}


void MovementModel::RotateVector2Matrix4(Matrix4f& outputMx, double* rotatevc){
    CvMat* cvMatd = cvCreateMat(3, 3, CV_64F);
    CvMat* cvMatRot = cvCreateMat(3, 1, CV_64F);
    cvMatRot->data.db[0] = rotatevc[0];
    cvMatRot->data.db[1] = rotatevc[1];
    cvMatRot->data.db[2] = rotatevc[2];
    cvRodrigues2(cvMatd, cvMatRot);
    outputMx(0, 0) = cvMatd->data.db[0];
    outputMx(0, 1) = cvMatd->data.db[1];
    outputMx(0, 2) = cvMatd->data.db[2];
    outputMx(1, 0) = cvMatd->data.db[3];
    outputMx(1, 1) = cvMatd->data.db[4];
    outputMx(1, 2) = cvMatd->data.db[5];
    outputMx(2, 0) = cvMatd->data.db[6];
    outputMx(2, 1) = cvMatd->data.db[7];
    outputMx(2, 2) = cvMatd->data.db[8];
    outputMx(0, 3) = rotatevc[3];
    outputMx(1, 3) = rotatevc[4];
    outputMx(2, 3) = rotatevc[5];

    outputMx(3, 0) = 0;
    outputMx(3, 1) = 0;
    outputMx(3, 2) = 0;
    outputMx(3, 3) = 1;
}

bool MovementModel::KalmanFilter(Matrix4f& inputMx){
    if (PrevMatrix4f == Eigen::Matrix4f::Identity()) {
        PrevMatrix4f = inputMx;
        return true;
    } else if (PrevGradient[0] == 0.0) {
        double rotatevc_src[5] = {
            0
        };
        double rotatevc_dst[5] = {
            0
        };
        Matrix4f2RotateVector(PrevMatrix4f, rotatevc_src);
        Matrix4f2RotateVector(inputMx, rotatevc_dst);
        for (int i = 0; i < 5; i++) {
            PrevGradient[i] = rotatevc_dst[i] - rotatevc_src[i];
        }
    }
}
} // cloud_icp_reg