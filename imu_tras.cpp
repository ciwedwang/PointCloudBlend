#include "stdafx.h"
#include <sstream>
#include "imu_tras.h"
#include "string_helper.h"

namespace cloud_icp_reg {
    using cv::Point3d;
    using std::set;
    using boost::unordered_map;
    using Eigen::Matrix4f;

    const double INIT_MAP_SIZE = 7000000.0; //map��ʼ��С,��ֹ��Ϊ��������,map����Ӱ������
    const double MAX_HOUR_CHECK = 2000000000.0;//�����������ʱ���Ƿ��Խ1Сʱ
    const int COUNT_IMU_ITEM_SHORT = 5;//imu�ַ����ĵ�Ԫ��,��Ӧ��imu���ַ�����ʽ
    const int COUNT_IMU_ITEM_LONG = 14;//imu�ַ����ĵ�Ԫ��,��Ӧ��imu���ַ�����ʽ

    ImuTras::ImuTras(std::string str_path)
        : _last_mat(Matrix4f::Identity()), _last_ms(0), _last_hour(0),
        _last_ori(cv::Point3d(0, 0, 0)), _is_valid(false),_is_last_stamp_error(false)
    {
        _map_mt.rehash(INIT_MAP_SIZE);
        _map_mt_idx.rehash(INIT_MAP_SIZE);
        ifstream if_imu(str_path.c_str());
        if (!if_imu.good()) {
            _is_valid = false;
            return;
        } else {
            _is_valid = true;
        }
        string str_line;
        long idx = 0;
        //��ȡÿһ��
        while (!if_imu.eof()) {
            getline(if_imu, str_line, '\n');
            vector< string> arr_items;
            arr_items = StringHelper::split(std::string(str_line), " ");
            //����imu�������ּ�¼��ʽ
            if (arr_items.size() == 5 || arr_items.size() == 14) {
                cv::Point3d pt_tmp;
                pt_tmp.x = atof(arr_items[2].c_str());
                pt_tmp.y = atof(arr_items[3].c_str());
                pt_tmp.z = atof(arr_items[4].c_str());
                ///////////////jake todo ������ͬ�ļ�¼
                //if (pt_tmp == _last_read_point)
                //{
                //	continue;
                //}else
                //{
                //	_last_read_point = pt_tmp;
                //}
                ///////////////////
                uint64_t key_src = 0; 
                //����stringstream���string��uint64_t��ת��
                stringstream stream;
                stream << arr_items[0];
                stream >> key_src;
				//jake ʱ����ͬ�Ͳ�������
				if (_last_ms == key_src)
				{
					//jake 20151212
					continue;
					//return;
				}
                //keySrcû��Сʱ,��last_msͻȻԶ����key_src,˵����Խ1Сʱ,key_src���ܵ����ֵ3599999476
                if (_last_ms > key_src + MAX_HOUR_CHECK) {
                    _last_hour++;
                }else{
					//jake20150909����Ƿ���gps����1s
					if (_last_ms > key_src)
					{
						if (!_is_last_stamp_error)
						{
							uint64_t stamp_last = (uint64_t) _last_hour * 3600 * 1000 * 1000 + _last_ms;
							uint64_t stamp_local = (uint64_t) _last_hour * 3600 * 1000 * 1000 + key_src;
							cout << ".................gps time error " <<
								stamp_last << " " << stamp_local << endl;
							fstream outfile;
							string filePath = str_path + "_error.txt";
							outfile.open(filePath, ios::app);
							outfile << _last_ms << " " << key_src << " ʱ�䵹��" << endl;
							outfile.close();
							_is_last_stamp_error = true;
						}
						//jake 20151212
						continue;
						//return;
					}
				}
				_is_last_stamp_error = false;
                _last_ms = key_src;
                //ʱ�������Сʱ
                key_src = (uint64_t) _last_hour * 3600 * 1000 * 1000 + key_src;
                //�����С,imu���ݷ���
                uint64_t key = get_hash_key(key_src);
                _map_mt_idx[key].insert(key_src);
                //jake todo 
                _map_mt_idx[key + 1].insert(key_src);
                _map_mt_idx[key - 1].insert(key_src);

                _map_mt[key_src] = pt_tmp;
                idx++;
#ifdef _DEBUG
                if (idx > 80000) {
                    return;
                }
#endif
            }
        }
        cout << "imu load finished . count " << _map_mt.size() << endl;
    }

    void ImuTras::get_mid_mt(cv::Point3d& pt_valid, uint64_t stamp, uint64_t stamp1, uint64_t stamp2){
        cv::Point3d pt1 = _map_mt[stamp1];
        cv::Point3d pt2 = _map_mt[stamp2];
        cv::Point3d pt_mid;
        double radio = 0.0;
        if (stamp1 != stamp2) {
            radio = abs((long) (stamp - stamp1)) / (double) (abs((long) (stamp2 - stamp1)));
        }
		//jake �޶�������ת�� -178 *0.7 +178*0.3 �۽ǵ�����
		if (abs(pt1.x - pt2.x) > 180)
		{
			if (pt1.x < 0)
			{
				pt1.x += 360;
			}
			if (pt2.x < 0)
			{
				pt2.x += 360;
			}
		}
        pt_valid.x = pt1.x * radio + pt2.x * (1 - radio);
        pt_valid.y = pt1.y * radio + pt2.y * (1 - radio);
        pt_valid.z = pt1.z * radio + pt2.z * (1 - radio);
		if (pt_valid.x > 180)
		{
			pt_valid.x -= 360;
		}
    }

    bool ImuTras::get_transaction_mx(const uint64_t stamp_cloud, Matrix4f& mat_rot, double& d_yaw,
        double& d_pitch, double& d_roll){
            return get_transaction_mx_mcs(stamp_cloud * 1000 ,mat_rot, d_yaw, d_pitch, d_roll);
    }

    bool ImuTras::get_transaction_mx_mcs(const uint64_t stamp_cloud, Matrix4f& mat_rot, double& d_yaw,
        double& d_pitch, double& d_roll){
            mat_rot = Matrix4f::Identity();
            //uint64_t stamp = stamp_cloud / 1000;
            d_yaw = 0;
            d_pitch = 0;
            d_roll = 0;
            if (!_is_valid) {
                return false;
            }
            //stamp *= 1000;
            //���imu�Ĵ���
            uint64_t key = get_hash_key(stamp_cloud);
            if (_map_mt_idx.find(key) == _map_mt_idx.end()) {
                return false;
            }
            //�����ҵ�����imu��ӽ���ʱ���
            set< uint64_t>& idxs = _map_mt_idx[key];
            if (idxs.size() == 0) {
                return false;
            }
            set< uint64_t>::iterator it2 = idxs.lower_bound(stamp_cloud);
            if (it2 == idxs.end()) {
                return false;
            }
            set< uint64_t>::iterator it1 = it2;
            if (it1 != idxs.begin())
                it1--;

            cv::Point3d pt_valid;
            ////Ѱ����ֵ
            get_mid_mt(pt_valid, stamp_cloud, *it1, *it2);


            //fix 
            //pt_valid.x = 0 - pt_valid.x;

            //uint64_t stamp_valid = 0;
            //if (abs((long) (*it2 - stamp)) <= abs((long) (*it1 - stamp))) {
            //    stamp_valid = *it2;
            //} else {
            //    stamp_valid = *it1;
            //}
            //pt_valid=_map_mt[stamp_valid];

            if (_last_ori.x == 0 && _last_ori.y == 0 && _last_ori.z == 0) {
                //_last_ori = pt_valid;
            }


            //�����������һ֡����ĽǶȱ仯
            cv::Point3d pt_gradient = pt_valid - _last_ori;
            d_yaw = pt_gradient.x;
            d_pitch = pt_gradient.y;
            d_roll = pt_gradient.z;
            //fix yaw 360
            if (abs(d_yaw) > 180)
            {
                if (d_yaw<0)
                {
                    d_yaw += 360;
                }else{
                    d_yaw -= 360;
                }
            }

            Matrix4f mat_current = pt2mat(pt_valid);
            Matrix4f mat_last = pt2mat(_last_ori);
            mat_rot = mat_current.inverse() * mat_last;
            _last_ori = pt_valid;
            ////���ݵ��ư�װ��imu��λ�����������,��imu�ĸ��ǶȻ�ԭ����������ϵ��
            //Eigen::AngleAxisf angle_yaw(DEG2RAD(pt_gradient.x), Eigen::Vector3f::UnitZ());
            //Eigen::AngleAxisf angle_pitch(DEG2RAD(-pt_gradient.y), Eigen::Vector3f::UnitY());
            //Eigen::AngleAxisf angle_roll(DEG2RAD(-pt_gradient.z), Eigen::Vector3f::UnitX());
            //Eigen::Quaternion< float> qt = angle_roll* angle_yaw* angle_pitch;
            //mat_rot.block(0, 0, 3, 3) = qt.matrix();
            return true;
    }

    //void ImuTras::get_matrix(float *cg, double yaw, double pitch, double roll, double z, double y, double x)
	void ImuTras::get_matrix(float *cg, double a, double b, double r, double z, double y, double x)
	{
		cg[0 * 4 + 0] = cos(a) * cos(b);
		cg[0 * 4 + 1] = cos(a) * sin(b) * sin(r) - sin(a) * cos(r);
		cg[0 * 4 + 2] = cos(a) * sin(b)* cos(r) + sin(a) * sin(r);
		cg[0 * 4 + 3] = z;
		cg[1 * 4 + 0] = sin(a) * cos(b);
		cg[1 * 4 + 1] = sin(a) * sin(b)* sin(r) + cos(a) * cos(r);
		cg[1 * 4 + 2] = sin(a) * sin(b)* cos(r) - cos(a) * sin(r);
		cg[1 * 4 + 3] = y;
		cg[2 * 4 + 0] = -sin(b);
		cg[2 * 4 + 1] = cos(b) * sin(r);
		cg[2 * 4 + 2] = cos(b) * cos(r);
		cg[2 * 4 + 3] = x;
		cg[3 * 4 + 0] = 0;
		cg[3 * 4 + 1] = 0;
		cg[3 * 4 + 2] = 0;
		cg[3 * 4 + 3] = 1;
		/*cg[0 * 4 + 0] = cos(pitch)*cos(yaw);
		cg[0 * 4 + 1] = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
		cg[0 * 4 + 2] = sin(roll)*sin(yaw) + cos(yaw)*sin(pitch)*cos(roll);
		cg[0 * 4 + 3] = z;
		cg[1 * 4 + 0] = cos(yaw) * cos(roll);
		cg[1 * 4 + 1] = cos(yaw)*cos(roll) + sin(pitch)*sin(yaw)*sin(roll);
		cg[1 * 4 + 2] = -sin(roll)*cos(yaw) + sin(pitch)*sin(yaw)*cos(roll);
		cg[1 * 4 + 3] = y;
		cg[2 * 4 + 0] = -sin(pitch);
		cg[2 * 4 + 1] = cos(pitch)*sin(roll);
		cg[2 * 4 + 2] = cos(pitch)*cos(roll);
		cg[2 * 4 + 3] = x;
		cg[3 * 4 + 0] = 0;
		cg[3 * 4 + 1] = 0;
		cg[3 * 4 + 2] = 0;
		cg[3 * 4 + 3] = 1;*/
	}

	//Matrix4f ImuTras::pt2mat(cv::Point3d pt_gradient)
	//{
	//	Matrix4f mat_rot=Matrix4f::Identity();
	//	//���ݵ��ư�װ��imu��λ�����������,��imu�ĸ��ǶȻ�ԭ����������ϵ��
	//	Eigen::AngleAxisf angle_yaw(DEG2RAD(pt_gradient.x), Eigen::Vector3f::UnitZ());
	//	Eigen::AngleAxisf angle_pitch(DEG2RAD(pt_gradient.y), Eigen::Vector3f::UnitY());
	//	Eigen::AngleAxisf angle_roll(DEG2RAD(pt_gradient.z), Eigen::Vector3f::UnitX());
	//	Eigen::Quaternion< float> qt = angle_roll* angle_pitch*angle_yaw;
	//	//Eigen::Quaternion< float> qt = angle_yaw * angle_pitch * angle_roll;
	//	mat_rot.block(0, 0, 3, 3) = qt.matrix();
	//	return mat_rot;
	//}

    Matrix4f ImuTras::pt2mat(const cv::Point3d& pt_gradient)
	{
		Matrix4f mat_rot=Matrix4f::Identity();
		//Eigen::AngleAxisf angle_yaw(DEG2RAD(pt_gradient.x), Eigen::Vector3f::UnitZ());
		//Eigen::AngleAxisf angle_pitch(DEG2RAD(pt_gradient.y), Eigen::Vector3f::UnitY());
		//Eigen::AngleAxisf angle_roll(DEG2RAD(pt_gradient.z), Eigen::Vector3f::UnitX());
		//Eigen::Quaternion< float> qt = angle_roll* angle_pitch*angle_yaw;
		////Eigen::Quaternion< float> qt = angle_yaw * angle_pitch * angle_roll;
		//mat_rot.block(0, 0, 3, 3) = qt.matrix();

		//	cout<<mat_rot<<endl;

		//Matrix4lf
		float array_rot[16]={0};
		get_matrix(array_rot, DEG2RAD(pt_gradient.x),DEG2RAD(-pt_gradient.y),
			DEG2RAD(-pt_gradient.z),0,0,0);
		mat_rot = Eigen::Matrix4f(array_rot);
		//cout<<mat_rot<<endl<<endl;  
		return mat_rot;
	}


    bool ImuTras::trans_cloud(const Cloud& cloud_src, Cloud& cloud_out){
        Matrix4f mat_rot = Matrix4f::Identity();
        bool flag = false;
        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        //��ȡ��תƽ�ƾ���
        flag = get_transaction_mx_mcs(cloud_src.header.stamp, mat_rot, yaw, pitch, roll);
        //���е��Ʊ仯
        pcl::transformPointCloud(cloud_src, cloud_out, mat_rot);
        return flag;
    }
} // cloud_icp_reg