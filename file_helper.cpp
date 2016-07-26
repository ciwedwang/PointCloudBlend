#include "stdafx.h"
#include "file_helper.h"
#include <pcl/io/pcd_io.h>
#include "string_helper.h"
#include "ficp_tools.h"

namespace cloud_icp_reg {
using Eigen::Matrix4f;
using boost::unordered_map;

fstream FileHelper::_out_file;
string FileHelper::_last_write_path = "";

FileHelper::FileHelper(){
}

FileHelper::~FileHelper(){
	_out_file.close();
}

void FileHelper::write_file_optimize_rot(const int frame, const string pcap_file,
    const Matrix4f& trans, bool is_append){
    stringstream file_path;
    file_path << pcap_file << "_optimizepos.txt";
    stringstream sstream;
    sstream << frame;
    for (int i = 0; i < 4.; i++) {
        for (int j = 0; j < 4; j++) {
            sstream << " " << trans(i, j);
        }
    }
    FileHelper::write_file(file_path.str(), sstream.str(), is_append);
}

void FileHelper::write_file_optimize_rot_stamp(const int frame, const string pcap_file,
    const Matrix4f& trans, bool is_append, int64_t stamp){
    stringstream file_path;
    file_path << pcap_file << "_optimizepos_stamp.txt";
    stringstream sstream;
    sstream << frame;
    for (int i = 0; i < 4.; i++) {
        for (int j = 0; j < 4; j++) {
            sstream << " " << trans(i, j);
        }
    }
    sstream << " " << stamp << " " << 100;
    FileHelper::write_file(file_path.str(), sstream.str(), is_append);
}

void FileHelper::write_file_no_nptimize_rot_stamp(const int frame, const string pcap_file,
    const Matrix4f& trans, bool is_append, int64_t stamp){
    stringstream file_path;
    file_path << pcap_file << "_nooptimizepos_stamp.txt";
    stringstream sstream;
    sstream << frame;
    for (int i = 0; i < 4.; i++) {
        for (int j = 0; j < 4; j++) {
            sstream << " " << trans(i, j);
        }
    }
    sstream << " " << stamp << " " << 100;
    FileHelper::write_file(file_path.str(), sstream.str(), is_append);
}

void FileHelper::write_file(const string file_path, const string content, bool is_append){
    fstream outfile;
    if (is_append) {
        outfile.open(file_path, ios::app);
    } else {
        outfile.open(file_path, ios::out | ios::trunc);
    }
    outfile << content << endl;
    outfile.close();

	//if(_last_write_path.compare(file_path) != 0 || _last_write_path=="")
	//{
	//	if (is_append) {
	//		_out_file.close();
	//		_out_file.open(file_path, ios::app);
	//		_last_write_path = file_path;
	//	} else {
	//		_out_file.close();
	//		_out_file.open(file_path, ios::out | ios::trunc);
	//		_last_write_path = "";
	//	}
	//}
 //   _out_file << content << endl;
	/*char* file_path_str = new char[file_path.size() + 1];
	strcpy_s(file_path_str , sizeof(char) * (file_path.size() + 1),file_path.c_str());
	FILE* fp = fopen(file_path_str, is_append?"a":"w"); 
	free(file_path_str);
	file_path_str = NULL;
	if(fp == NULL)
	{
		cout << file_path << "打开失败" << endl;
		fclose(fp);
		return;
	}
	char* str = new char[content.size() + 2];
	strcpy_s(str , sizeof(char) * (content.size() + 2),content.c_str());
	str[content.size()]='\n';
	str[content.size()+1]='\0';
	size_t size = strlen(str);
    fwrite(str, sizeof(char), size, fp);  
	free(str);
	str = NULL;
	fclose(fp);*/
}

bool FileHelper::load_file_house_back(const string pcap_file, list< uint64_t>& stamps){
    string file_path = pcap_file + "_houseback.txt";
    fstream in_file(file_path);
    if (!in_file.good()) {
        cout << "file house back empty :" << file_path << endl;
        return false;
    }
    int pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector< string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
		if(in_file.fail())  
        {  
			in_file.clear(in_file.rdstate() & ~(ifstream::failbit));  
        }  
        stringstream stream;
        stream << str_line;
        stream >> stamp;
        if (stamp != 0) {
            stamps.push_back(stamp);
        }
    }
    in_file.close();

    return true;
}

bool FileHelper::load_file_glass(const string pcap_file, 
    map< uint64_t, uint64_t>& intensity){
    string file_path = pcap_file + "_glass.txt";
    fstream in_file(file_path);
    if (!in_file.good()) {
        cout << "file house back empty :" << file_path << endl;
        return false;
    }
    int pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector< string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
		if(in_file.fail())  
        {  
			in_file.clear(in_file.rdstate() & ~(ifstream::failbit));  
        }  
        arr_strs = StringHelper::split(std::string(str_line), " ");
        if (arr_strs.size() == 2) {
            stringstream stream;
            stream << arr_strs[0];
            stream >> stamp;
            if (stamp != 0) {
                stream.clear();
                stream << arr_strs[1];
                stream >> intensity[stamp];
            }
        }
    }
    in_file.close();

    return true;
}

bool FileHelper::load_file_ignore_frame(const string pcap_file, 
    map< uint64_t, uint64_t>& stamps){
    string file_path = pcap_file + "_ignore.txt";
    fstream in_file(file_path);
    if (!in_file.good()) {
        cout << "file house back empty :" << file_path << endl;
        return false;
    }
    int pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector< string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
		if(in_file.fail())  
        {  
			in_file.clear(in_file.rdstate() & ~(ifstream::failbit));  
        }  
        arr_strs = StringHelper::split(std::string(str_line), " ");
        if (arr_strs.size() >= 2) {
            stringstream stream;
            stream << arr_strs[0];
            stream >> stamp;
            if (stamp != 0) {
                stream.clear();
                stream << arr_strs[1];
                stream >> stamps[stamp];
            }
        }
    }
    in_file.close();

    return true;
}

bool FileHelper::load_file(const string pcap_file, const string suffix,
    unordered_map< int, Matrix4f>& map_rots, unordered_map< int, int64_t>& stamps, 
    bool is_contain_stamp){
    string file_path;
    file_path = pcap_file + suffix;

    fstream in_file(file_path);
    if (!in_file.good()) {
        cout << "file rot empty :" << file_path << endl;
        return false;
    }
    long pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector< string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline	(in_file, str_line, '\n');
		if(in_file.fail())  
        {  
			in_file.clear(in_file.rdstate() & ~(ifstream::failbit));  
        }  
        arr_strs = StringHelper::split(std::string(str_line), " ");
        if (arr_strs.size() == 18 || arr_strs.size() == 17 || arr_strs.size() == 19) {
            frame = atoi(arr_strs[0].c_str());
            Matrix4f trans = Matrix4f::Identity();
			pointcount++;
            trans << atof(arr_strs[1].c_str()), atof(arr_strs[2].c_str()),
                atof(arr_strs[3].c_str()), atof(arr_strs[4].c_str()),
                atof(arr_strs[5].c_str()), atof(arr_strs[6].c_str()), atof(arr_strs[7].c_str()),
                atof(arr_strs[8].c_str()),
                atof(arr_strs[9].c_str()), atof(arr_strs[10].c_str()), atof(arr_strs[11].c_str()),
                atof(arr_strs[12].c_str()),
                atof(arr_strs[13].c_str()), atof(arr_strs[14].c_str()), atof(arr_strs[15].c_str()),
				atof(arr_strs[16].c_str());
			if (arr_strs.size() == 18) {
				//旧数据
				stringstream stream;
				stream << arr_strs[17];
				stream >> stamp;
				stamp -= 100;
				stamps[frame] = stamp;
				if (pointcount % 1000 == 0)
				{
					cout << "pos载入旧数据 " << stamp << endl;
				}
			}else if (arr_strs.size() == 19 && atoi(arr_strs[18].c_str()) == 100) {
                stringstream stream;
                stream << arr_strs[17];
                stream >> stamp;
				//新数据
                stamps[frame] = stamp;
				if (pointcount % 1000 == 0)
				{
					cout << "pos载入新数据 " << stamp << endl;
				}
            }else
			{
				cout << "pos数据载入不正确" << str_line << endl;
				continue;
			}

            if (trans == Matrix4f::Zero()) {
                cout << "error.........................load rot ignore......" << endl;
                printf("error.........................load rot ignore......\n");
                continue;
            }
            map_rots[frame] = trans;
        } else {
            continue;
        }
    }
    in_file.close();

    return true;
    /*
    FILE *fp = fopen(file_path.c_str(), "r");
    if (fp == NULL)
    {
    cout << "file rot empty :" << file_path;
    return false;
    }
    int frame = 0;
    float r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15;
    r0 = r1 = r2 = r3 = r4 = r5 = r6 = r7 = r8 = r9 = r10 = r11 = r12 = r13 = r14 = r15 = 0;
    while (!feof(fp))
    {
    uint64_t stamp = 0;
    if (isContainStamp)
    {
    fscanf_s(fp, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %lld",
    &frame, &r0, &r1, &r2, &r3, &r4, &r5, &r6, &r7, &r8, &r9, &r10, &r11, &r12, &r13, &r14, &r15, &stamp);
    }
    else
    {
    fscanf_s(fp, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
    &frame, &r0, &r1, &r2, &r3, &r4, &r5, &r6, &r7, &r8, &r9, &r10, &r11, &r12, &r13, &r14, &r15);
    }
    Matrix4f trans = Matrix4f::Identity();
    trans << r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15;
    //cout << "load file rot :" << frame << endl <<trans<< endl << endl;
    if (trans == Matrix4f::Zero())
    {
    cout << "error.........................load rot ignore......" << endl;
    printf("error.........................load rot ignore......\n");
    continue;
    }
    map_rots[frame] = trans;
    stamps[frame] = stamp;
    //Vector4f ci;
    //Matrix42Vector4(trans, ci);
    }
    cout << "file rot load finished.rot size:" << map_rots.size() << file_path;
    fclose(fp);
    return true;
    */
}
bool FileHelper::load_file(const string pcap_file, const string suffix,
    unordered_map< int, Matrix4f>& map_rots){
    unordered_map< int, int64_t> stamps;
    return load_file(pcap_file, suffix, map_rots, stamps, false);
}

bool FileHelper::load_file_rot(const string pcap_file, unordered_map< int, Matrix4f>& map_rots){
    return load_file(pcap_file, "_pos.txt", map_rots);
}

bool FileHelper::load_file_rot_fix(const string pcap_file, unordered_map< int, Matrix4f>& map_rots,
	unordered_map< uint64_t,Matrix4f>& map_rots_stamp){
		unordered_map< int, int64_t> stamps;
		return load_file_rot_fix(pcap_file,map_rots,map_rots_stamp,stamps);
}

bool FileHelper::load_file_rot_fix(const string pcap_file, unordered_map< int, Matrix4f>& map_rots,
	unordered_map< uint64_t,Matrix4f>& map_rots_stamp,unordered_map< int, int64_t>& stamps){
		bool flag = load_file(pcap_file, "_pos_fix.txt", map_rots, stamps, true);
		for (unordered_map< int, int64_t>::iterator it = stamps.begin(); it != stamps.end(); it++) {
			map_rots_stamp[it->second] = map_rots[it->first];
		}
		return flag;
}

bool FileHelper::load_file_rot(const string pcap_file, unordered_map< int, Matrix4f>& map_rots,
    unordered_map< uint64_t,Matrix4f>& map_rots_stamp){
        unordered_map< int, int64_t> stamps;
        return load_file_rot(pcap_file,map_rots,map_rots_stamp,stamps);
}

bool FileHelper::load_file_rot(const string pcap_file, unordered_map< int, Matrix4f>& map_rots,
    unordered_map< uint64_t,Matrix4f>& map_rots_stamp,unordered_map< int, int64_t>& stamps){
    bool flag = load_file(pcap_file, "_pos.txt", map_rots, stamps, true);
    for (unordered_map< int, int64_t>::iterator it = stamps.begin(); it != stamps.end(); it++) {
        map_rots_stamp[it->second] = map_rots[it->first];
    }
    return flag;
}

bool FileHelper::load_file_rot_new(const string pcap_file, 
    unordered_map< int, Matrix4f>& map_rots){
    return load_file(pcap_file, "_posnew.txt", map_rots);
}

bool FileHelper::load_file_optimize_rot(const string pcap_file, 
    unordered_map< int, Matrix4f>& map_rots){
    return load_file(pcap_file, "_optimizepos.txt", map_rots);
}

bool FileHelper::load_file_optimize_rot_stamp(const string pcap_file, 
    unordered_map< int, Matrix4f>& map_rots,
    unordered_map< int, int64_t>& stamps){
    return load_file(pcap_file, "_pos.txt", map_rots, stamps, true);
}

bool FileHelper::load_file_loop_core(const string &file_path,
    vector< pair< pair<uint64_t, uint64_t>, Matrix4f> >& map_loops)
{
    fstream in_file(file_path);
    if (!in_file.good()) {
        return false;
    }
    int pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector< string> arr_strs;
    uint64_t stamp_start = 0;
    uint64_t stamp_end = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        if (in_file.fail()) {
            in_file.clear(in_file.rdstate() & ~(ifstream::failbit));
        }
        arr_strs = StringHelper::split(std::string(str_line), " ");
        if (arr_strs.size() == 2 || arr_strs.size() == 18) {
            stringstream stream;
            stream << arr_strs[0];
            stream >> stamp_start;
            stream.clear();
            stream << arr_strs[1];
            stream >> stamp_end;
            pair< pair<uint64_t, uint64_t>, Matrix4f> loop_item;
            //cout << "load file loop :" << getAllowedLoopIdx(frameStart, ignoreCount) << " " << getAllowedLoopIdx(frameEnd, ignoreCount) << endl;
            //loop_item[getAllowedLoopIdx(frameStart, ignoreCount)] = getAllowedLoopIdx(frameEnd, ignoreCount);
            loop_item.first.first = stamp_start;
            loop_item.first.second = stamp_end;
            if (arr_strs.size() == 2) {
                loop_item.second = Matrix4f::Identity();
            } else {
                float mat_arr[16] = { 0 };
                for (int i = 0; i < 16; i++) {
                    mat_arr[i] = atof(arr_strs[i + 2].c_str());
                }
                Eigen::Matrix<float, 4, 4, RowMajor> trans_mat(mat_arr);
                loop_item.second = trans_mat;
            }

            map_loops.push_back(loop_item);
        }
    }
    in_file.close();
    return true;
}

bool FileHelper::load_file_loop_core(const string &file_path,
    vector< pair<uint64_t, uint64_t> > &map_loops)
{
    fstream in_file(file_path);
    if (!in_file.good()) {
        return false;
    }
    int pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector< string> arr_strs;
    uint64_t stamp_start = 0;
    uint64_t stamp_end = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        if (in_file.fail()) {
            in_file.clear(in_file.rdstate() & ~(ifstream::failbit));
        }
        arr_strs = StringHelper::split(std::string(str_line), " ");
        if (arr_strs.size() >= 2) {
            stringstream stream;
            stream << arr_strs[0];
            stream >> stamp_start;
            stream.clear();
            stream << arr_strs[1];
            stream >> stamp_end;
            pair<uint64_t, uint64_t> loop_item;
            //cout << "load file loop :" << getAllowedLoopIdx(frameStart, ignoreCount) << " " << getAllowedLoopIdx(frameEnd, ignoreCount) << endl;
            //loop_item[getAllowedLoopIdx(frameStart, ignoreCount)] = getAllowedLoopIdx(frameEnd, ignoreCount);
            loop_item.first = stamp_start;
            loop_item.second = stamp_end;

            map_loops.push_back(loop_item);
        }
    }
    in_file.close();
    return true;
}

bool FileHelper::load_file_loop(const string& pcap_file,
    vector< pair< pair<uint64_t, uint64_t>, Matrix4f> >& map_loops)
{
    string file_path;
    file_path = pcap_file + "_loop2.txt";
    bool ret = FileHelper::load_file_loop_core(file_path, map_loops);
    if (!ret) {
        cout << "file loop2 empty" << endl;
    }
    else {
        for (int i = 0; i < map_loops.size(); i++) {
            cout << "load file loop2 :" << map_loops[i].first.first << " " << map_loops[i].first.second << endl;
        }
    }
    return ret;
}

bool FileHelper::load_file_loop2(const string& pcap_file, 
    vector< pair< pair<uint64_t, uint64_t>, Matrix4f> >& map_loops)
{
    string file_path;
    file_path = pcap_file + "_loop3.txt";
    bool ret = FileHelper::load_file_loop_core(file_path, map_loops);
    if (!ret) {
        cout << "file loop2 empty" << endl;
    }
    else {
        for (int i = 0; i < map_loops.size(); i++) {
            cout << "load file loop3 :" << map_loops[i].first.first << " " << map_loops[i].first.second << endl;
        }
    }
    return ret;
}

bool FileHelper::load_file_loop_ignore(const string &pcap_file,
    vector<pair<uint64_t, uint64_t> >& map_ignore_loops)
{
    string file_path = pcap_file + "_loop_ignore.txt";
    bool ret = FileHelper::load_file_loop_core(file_path, map_ignore_loops);
    if (!ret) {
        cout << "file loop ignore empty" << endl;
    }
    else {
        for (int i = 0; i < map_ignore_loops.size(); i++) {
            cout << "load file loop ignore:" << map_ignore_loops[i].first << " " << map_ignore_loops[i].second << endl;
        }
    }
    return ret;
}

void FileHelper::write_file_rot(const int frame, const string pcap_file, 
    const Matrix4f& trans,
    bool isAppend, int64_t stamp){
    stringstream file_path;
    file_path << pcap_file << "_pos.txt";
    stringstream sstream;
    sstream << frame;
    for (int i = 0; i < 4.; i++) {
        for (int j = 0; j < 4; j++) {
            sstream << " " << trans(i, j);
        }
    }
    if (stamp != -1) {
		sstream << " " << stamp << " " << 100;
    }
    write_file(file_path.str(), sstream.str(), isAppend);
}
/*
void FileHelper::ReadCSV_2_PointXYZRGBA(CloudPtr point_cloud_ptr, const char* pname){
    cout << pname << endl;
    //point_cloud_ptr->clear();
    ifstream in_file(pname);
    if (!in_file.good()) {
        return;
    }
    string str_line, TempStr;
    getline(in_file, str_line, '\n');
    //cout << StrLine << endl;
    int n = 0;

    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        vector< string> arr_strs;
        arr_strs = StringHelper::split(std::string(str_line), ",");
        //cout << StrLine << endl;
        if (arr_strs.size() == 8 && n % 10 == 0) {
            //if (atof(StrArray[3].c_str()) < 20)
            //{
            //  continue;
            //}

            //if (atof(StrArray[3].c_str()) > 60)
            //{
            //  continue;
            //}

            //if (atof(StrArray[6].c_str())>15)
            //{
            //  continue;
            //}

            CloudItem tempPoint;
            tempPoint.x = atof(arr_strs[0].c_str());
            tempPoint.y = atof(arr_strs[1].c_str());
            tempPoint.z = atof(arr_strs[2].c_str());
            //if (tempPoint.z <0)
            //{
            //continue;
            //}

            //if (tempPoint.z > 0.5)
            //{
            //continue;
            //}
            //if (tempPoint.z != 0)
            //{
            //  continue;
            //}
            //if (tempPoint.x>0)
            //{
            //  continue;
            //}

            //if (tempPoint.z >2)
            //{
            //  continue;
            //}

            uint32_t rgb = intensity2color(100, atoi(arr_strs[3].c_str()));//(static_cast<uint32_t>(r) << 16 |
            //tempPoint.label = *reinterpret_cast<float*>(&rgb);
            tempPoint.rgb = rgb;
            if (point2origindis(tempPoint.x, tempPoint.y, tempPoint.z) < 50)
                point_cloud_ptr->push_back(tempPoint);
        }
        n++;
        //////////////////////////////////////////////////////////////////////////
#ifdef _DEBUG

        if (n > 1000) {
            break;
        }
#endif // _DEBUG
    }
    in_file.close();
}
*/
} // cloud_icp_reg
