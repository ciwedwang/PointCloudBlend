#include "stdafx.h"


/*

void OnGetHdlCloudG(const CloudIConstPtr& cloud){
CloudPtr cloudTemp(new Cloud);
CloudPtr cloudTemp2(new Cloud);
CloudPtr cloudTempSrc(new Cloud);
CloudPtr cloudTras(new Cloud);
double dYaw, dPitch, dRoll;
g_status._ignore_frame_count++;

if (CheckIgnoreFrameValid(cloud->header.stamp / 1000, g_status._ignore_stamp) == false) {
return;
}
PointCloudHelper::cloudi2cloud(cloud, cloudTempSrc, 1, false, -1,
GetGlassMinIns(cloud->header.stamp / 1000, g_status._glass_stamp));
PointCloudHelper::remove_duplicate(cloudTempSrc, 0.13);

PointCloudHelper::remove_outlier(cloudTempSrc);
PointCloudHelper::add_trace_point(cloudTempSrc);

#ifdef _DEBUG
g_pv.updatePointCloud(cloudTempSrc);
g_pv.spinOnce();
return;
#else
//恢复历史记录
if (g_status._start_frame > 0) {
Matrix4f rotOld;
if (MatHelper::get_rot_with_stamp(g_status._map_clouds_rot_stamps, cloud->header.stamp / 1000, rotOld)) {
cout << "use old rot " << cloud->header.stamp / 1000 << endl;
g_status._global_transform = rotOld;
PointCloudHelper::cloud_convert_with_rot(g_status._map_clouds_rot_stamps, cloud->header.stamp / 1000, cloudTempSrc,
cloudTras, g_status._global_transform);
CloudGrid::instance().add_cloud(cloudTras);
CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache, g_status._global_transform);
g_status._process_count++;
goto processend;
}
if (cloud->header.stamp / 1000 < g_status._start_frame) {
return;
}
}
#endif

//1st frame
if (g_status._cloud_src->points.size() == 0) {
pcl::copyPointCloud(*cloudTempSrc, *g_status._cloud_src);
g_pv.updatePointCloud(g_status._cloud_src);
*g_status._cloud_cache = *g_status._cloud_src;
*g_status._cloud_result = *g_status._cloud_src;
////imu
//Matrix4f mtTmp = Matrix4f::Identity();
//gImuTrans->GetTransactionMx(gCloudSrc->header.stamp / 1000, mtTmp, dYaw, dPitch, dRoll);
//gMoveModel.UpdateModelLocal(mtTmp);
//cout << "Imu transform:" << endl;
//cout << mtTmp << endl;
FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, Matrix4f::Identity(), false,
cloud->header.stamp / 1000);
g_status._optimize_rot.push_back(Matrix4f::Identity());
return;
}

//imu
Matrix4f mtTmp = Matrix4f::Identity();
bool flag = gImuTrans->get_transaction_mx(cloudTempSrc->header.stamp / 1000, mtTmp, dYaw, dPitch,
dRoll);
//gMoveModel.UpdateModelLocal(mtTmp);
if (flag) {
mtTmp = mtTmp * g_status._last_mt;
g_status._last_mt = mtTmp;
pcl::transformPointCloud(*cloudTempSrc, *cloudTemp, mtTmp);
//cout << "Imu transform:" << endl;
//cout << mtTmp << endl;
} else {
g_status._last_mt = Matrix4f::Identity();
pcl::copyPointCloud(*cloudTempSrc, *cloudTemp);
cout << g_status._ignore_frame_count << " not find correspond imu stamp..................."
<< cloudTempSrc->header.stamp / 1000 << endl;
printf("%d not find correspond imu stamp...................%ld\n", g_status._ignore_frame_count,
cloudTempSrc->header.stamp / 1000);
}
g_status._process_count++;
cout << "process count "<<g_status._process_count <<"   count " << cloudTempSrc->points.size()<< endl;
processagain:
Matrix4f pairTransform;
float error1 = PointCloudHelper::get_rot_icp(g_status._cloud_src, cloudTemp, pairTransform);

Matrix4f GlobalTransform2 = pairTransform* mtTmp* g_status._global_transform;
pcl::transformPointCloud(*cloudTempSrc, *cloudTras, GlobalTransform2);

float error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloudTras, pairTransform);
cout << "error2:" << error2 << endl;
if (error2 > 0.15 || error2 < 0) {
if (g_status._queue_cloud_tras.size() > 0) {
CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
g_status._queue_cloud_tras.pop();
//GetReducedCloud(gCloudCache, true);
CloudGrid::instance().add_cloud(cloudCacheTrasTemp);
CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache, g_status._global_transform);
if (g_status._house_back_stamp.size() > 0 && g_status._house_back_stamp.front() >= cloud->header.stamp / 1000) {
CloudGrid::instance().add_cloud(g_status._cloud_cache);
g_status._house_back_stamp.pop();
}
goto processagain;
} else {
if (cloudTemp2->size() == 0) {
pcl::copyPointCloud(*cloudTras, *cloudTemp2);
g_status._rotate_count = 1;
}
Matrix4f adjustTransform = Matrix4f::Identity();
processrotate : 
MatHelper::get_adjust_tras(g_status._rotate_count, adjustTransform);
g_status._rotate_count++;
cout << ".........................." << g_status._rotate_count << endl;
printf("...........................%d\n", g_status._rotate_count);
if (g_status._rotate_count > 15) {
g_status._skip_frame++;
return;
}
pcl::transformPointCloud(*cloudTemp2, *cloudTras, adjustTransform);
error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloudTras, pairTransform);
cout << "error2: " << error2 << endl;
if (error2 > 0.15 + g_status._skip_frame / 20.0 || error2 < 0) {
goto processrotate;
} else {
GlobalTransform2 = adjustTransform * GlobalTransform2;
goto processcontinue;
}
}
}
processcontinue : g_status._global_transform = pairTransform * GlobalTransform2;
g_move_model_after.UpdataModel(g_status._global_transform);
//gMoveModel.SaveModelLocal(gGlobalTransform);
pcl::transformPointCloud(*cloudTempSrc, *cloudTras, g_status._global_transform);
//cout << gIgnoreFrameCount <<endl<< gGlobalTransform<<endl;
g_status._skip_frame = 0;
if (g_status._process_count < processCacheMinNum) {
CloudGrid::instance().add_cloud(cloudTras);
CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache, g_status._global_transform);
if (g_status._house_back_stamp.size() > 0 && g_status._house_back_stamp.front() >= cloud->header.stamp / 1000) {
CloudGrid::instance().add_cloud(g_status._cloud_cache);
g_status._house_back_stamp.pop();
}
} else {
CloudPtr cloudCacheTras(new Cloud);
pcl::copyPointCloud(*cloudTras, *cloudCacheTras);


g_status._queue_cloud_tras.push(cloudCacheTras);
if (g_status._queue_cloud_tras.size() >= processCacheMinNum) {
CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
g_status._queue_cloud_tras.pop();
CloudGrid::instance().add_cloud(cloudCacheTrasTemp);
CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache, g_status._global_transform);
if (g_status._house_back_stamp.size() > 0 && g_status._house_back_stamp.front() >= cloud->header.stamp / 1000) {
CloudGrid::instance().add_cloud(g_status._cloud_cache);
g_status._house_back_stamp.pop();
}
}
}
processend :
FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, g_status._global_transform, true,
cloud->header.stamp / 1000);
g_status._optimize_rot.push_back(g_status._global_transform);
cout << g_status._ignore_frame_count << endl;
printf("%d\n", g_status._ignore_frame_count);
g_status._cloud_src->clear();
g_status._cloud_src = cloudTempSrc;
cloudTempSrc = CloudPtr(new Cloud);
g_status._last_mt = Matrix4f::Identity();

if (g_status._process_count % 10 == 0) {
*g_status._cloud_result = *g_status._cloud_result + *cloudTras;
PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.3);
//CloudGrid::instance().GetResultCloud(gCloudResult);
g_pv.updatePointCloud(g_status._cloud_result);
}
g_pv.spinOnce();


//loop 优化
if (g_status._map_loops.size() > 0) {
g_status._clouds_line.push_back(cloudTras);
for (int i = 0; i < g_status._map_loops.size(); i++) {
map< uint64_t,uint64_t> loop = g_status._map_loops[i];
if (loop.begin()->second <= cloudTras->header.stamp / 1000) {
//gPv.spin();
//优化
vector< map< uint64_t,uint64_t> > loopIns;
loopIns.push_back(loop);
PointCloudHelper::do_lum_elch(loopIns, g_status._clouds_line, g_status._optimize_rot);
g_status._map_loops.erase(g_status._map_loops.begin());
g_status._start_frame = 0;
if (loop.size() == 0) {
g_move_model_after.SetEnable(false);
}

//恢复显示
g_status._cloud_result->clear();
int numReIcp = 100;
for (int i = 0; i < g_status._clouds_line.size(); i++) {
CloudPtr item = g_status._clouds_line[i];
FileHelper::WriteFileRot(i, g_status._pcap_file, g_status._optimize_rot[i],
i == 0 ? false : true, g_status._clouds_line[i]->header.stamp / 1000);
*g_status._cloud_result += *item;
}
PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.15f);
g_pv.updatePointCloud(g_status._cloud_result);
//恢复cache
g_status._global_transform = g_status._optimize_rot[g_status._optimize_rot.size() - 1];
CloudGrid::instance().add_cloud(g_status._cloud_result);
CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache, g_status._global_transform);
//重新icp一些最近的100个点
//for (int i = gCloudsLine.size() - 100; i < gCloudsLine.size(); i++)
//{
//  CloudPtr item = gCloudsLine[i];
//  GetRotIcp(meta_start, meta_end, loop_transform_);
//  CloudPtr cloudOut(new Cloud());
//  pcl::transformPointCloud(*meta_end, *cloudOut, loop_transform_);
//  FileHelper::WriteFileRot(i, gPcapFile, gOptimizeRot[i], i == 0 ? false : true, gCloudsLine[i]->header.stamp / 1000);
//  *gCloudResult += *item;
//}
break;
}
}
}
}




*/
/*

void OnGetHdlCloudImu(const CloudIConstPtr& cloud){
    CloudPtr cloudTemp(new Cloud);
    CloudPtr cloudTemp2(new Cloud);
    CloudPtr cloudTras(new Cloud);
    g_status._ignore_frame_count++;

    double dYaw, dPitch, dRoll;
    if (g_status._cloud_src->points.size() == 0) {
        PointCloudHelper::cloudi2cloud(cloud, g_status._cloud_src);
        g_pv.updatePointCloud(g_status._cloud_src);
        //imu
        Matrix4f mtTmp = Matrix4f::Identity();
        gImuTrans->get_transaction_mx(g_status._cloud_src->header.stamp / 1000, mtTmp, dYaw, dPitch, dRoll);
        cout << "Imu transform:" << endl;
        cout << mtTmp << endl;
        return;
    }
    PointCloudHelper::cloudi2cloud(cloud, cloudTemp2);
    PointCloudHelper::add_trace_point(cloudTemp2);
    //imu
    Matrix4f mtTmp = Matrix4f::Identity();
    bool flag = gImuTrans->get_transaction_mx(cloudTemp2->header.stamp / 1000, mtTmp, dYaw, dPitch, dRoll);
    if (flag) {
        mtTmp = mtTmp * g_status._last_mt;
        g_status._last_mt = mtTmp;
        pcl::transformPointCloud(*cloudTemp2, *cloudTemp, mtTmp);
        cout << "Imu transform:" << endl;
        cout << mtTmp << endl;
    } else {
        g_status._last_mt = Matrix4f::Identity();
        pcl::copyPointCloud(*cloudTemp2, *cloudTemp);
        cout << g_status._ignore_frame_count << " not find correspond imu stamp..................." << endl;
        printf("%d not find correspond imu stamp...................", g_status._ignore_frame_count);
    }

    //gGlobalTransform = mtTmp*gGlobalTransform;
    //pcl::transformPointCloud(*cloudTemp2, *cloudTras, gGlobalTransform);
    //gPv.updatePointCloud(cloudTras);
    //gPv.spinOnce();
    //return;

    cout << "src size :" << cloud->size() << "  input size: " << cloudTemp2->points.size() << endl;
    g_status._process_count++;
    bool isFirstProcess = true;

processagain:
    /////////////////////////////////////////////////////////////
    Matrix4f pairTransform;
    float error1 = PointCloudHelper::get_rot_icp(g_status._cloud_src, cloudTemp, pairTransform);
    if (dYaw == 0 && dPitch == 0 && dRoll == 0) {
        dYaw = atan2(pairTransform(1, 0), pairTransform(0, 0)) / M_PI * 180.0;
        dPitch = atan2(-pairTransform(2, 0),
            sqrt(pairTransform(2, 1) * pairTransform(2, 1) + pairTransform(2, 2) *
            pairTransform(2, 2))) / M_PI * 180.0;
        dRoll = atan2(pairTransform(1, 0), pairTransform(0, 0)) / M_PI * 180.0;
    }

    cout << pairTransform << endl;
    //if (MatHelper::CheckRotValid2(pairTransform) == false)
    //{
    //  cloudTemp = CloudPtr(new Cloud);
    //  return;
    //}
    Matrix4f GlobalTransform2 = pairTransform* mtTmp* g_status._global_transform;
    pcl::transformPointCloud(*cloudTemp2, *cloudTras, GlobalTransform2);

    if (g_status._cloud_result->size() == 0) {
        *g_status._cloud_cache = *g_status._cloud_src;
        *g_status._cloud_result = *g_status._cloud_src;
        FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, Matrix4f::Identity(), false,
            cloud->header.stamp / 1000);
    }


    float error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloudTras, pairTransform);
    //CloudPtr cloudTmp(new Cloud);
    //GetCloudLine(cloudTras, cloudTmp);
    //pcl::copyPointCloud(*cloudTmp, *cloudTras);
    //GetRotIcpPl(cloudCache, cloudTras, pairTransform);
    //GetRotIcp(cloudResult, cloudTras, pairTransform);
    cout << "...once icp dis " << error1 << endl;
    cout << "once rot matrix:" << endl << pairTransform << endl;


    //if (MatHelper::CheckRotValid(pairTransform) == false)
    //{
    //  cloudTemp2 = CloudPtr(new Cloud);
    //  return;
    //}

    Matrix4f pairTransformCache;
    pairTransformCache = g_status._global_transform.inverse() * pairTransform * GlobalTransform2;
    cout << "cloud cache:" << endl << pairTransformCache << endl;
    if (abs(pairTransformCache(0, 3)) > MAXTRACEDIS ||
        abs(pairTransformCache(1, 3)) > MAXTRACEDIS ||
        abs(pairTransformCache(2, 3)) > MAXTRACEDIS || abs(pairTransformCache(2, 2) - 1) > 0.1) {
            if (isFirstProcess) {
                printf("clear cache......................\n");
                cout << "clear cache......................" << endl;
                while (g_status._queue_cloud_tras.size() > 0) {
                    CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
                    g_status._queue_cloud_tras.pop();
                    *g_status._cloud_cache = *g_status._cloud_cache + *cloudCacheTrasTemp;
                }
                PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
                isFirstProcess = false;
                goto processagain;
            }
            //jake 暂时去掉位置纠正
            //else
            //{
            //  printf("stay......................\n");
            //  cout << "stay......................" << endl;
            //}
    } else {
        g_status._global_transform = pairTransform * GlobalTransform2;
    }

    pcl::transformPointCloud(*cloudTemp2, *cloudTras, g_status._global_transform);
    if (MatHelper::check_rot_valid(dYaw, dPitch, dRoll) == false) {
        g_status._global_transform = Matrix4f::Zero();
    }
    FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, g_status._global_transform, true,
        cloud->header.stamp / 1000);

    cout << ".....final icp dis " << error2 << endl;
    cout << "final rot matrix:" << endl << g_status._global_transform << endl << endl;

    //loop
    //CloudPtr cloudLine(new Cloud);
    //GetCloudLine(cloudTras, cloudLine);
    //gCloudsLine.push_back(cloudLine);

    if (g_status._process_count < processCacheMinNum) {
        *g_status._cloud_cache = *g_status._cloud_cache + *cloudTras;
        PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
    } else {
        CloudPtr cloudCacheTras(new Cloud);
        pcl::copyPointCloud(*cloudTras, *cloudCacheTras);
        cout << "DYaw : " << dYaw << " DPitch : " << dPitch << " DRoll : " << dRoll << endl;
        printf("DYaw : %f DPitch : %f DRoll : %f \n", dYaw, dPitch, dRoll);
        if (MatHelper::check_rot_valid(dYaw, dPitch, dRoll))
            g_status._queue_cloud_tras.push(cloudCacheTras); 
        if (g_status._queue_cloud_tras.size() >= processCacheMinNum) {
            CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
            g_status._queue_cloud_tras.pop();
            *g_status._cloud_cache = *g_status._cloud_cache + *cloudCacheTrasTemp;
            PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
        }
    }

    cout << g_status._ignore_frame_count << ":" << g_status._cloud_result->size() << endl;
    printf("gIgnoreFrameCount %d : %d", g_status._ignore_frame_count, g_status._cloud_result->size());

    g_status._cloud_src->clear();
    g_status._cloud_src = cloudTemp2;
    cloudTemp = CloudPtr(new Cloud);
    cloudTemp2 = CloudPtr(new Cloud);
    g_status._last_mt = Matrix4f::Identity();

    if (g_status._process_count % 10 == 0) {
        //RemoveDuplicate(cloudTras, RDUMPLICATE_VALUE);
        if (MatHelper::check_rot_valid(dYaw, dPitch, dRoll))
            *g_status._cloud_result = *g_status._cloud_result + *cloudTras;
        PointCloudHelper::remove_duplicate(g_status._cloud_result, RDUMPLICATE_VALUE);
        g_pv.updatePointCloud(g_status._cloud_result);
        g_pv.spinOnce();
    }
}
void OnGetHdlCloudBak(const CloudIConstPtr& cloud){
    CloudPtr cloudTemp(new Cloud);
    //CloudPtr cloudTemp2(new Cloud);
    CloudPtr cloudTras(new Cloud);
    g_status._ignore_frame_count++;

    if (g_status._cloud_src->points.size() == 0) {
        PointCloudHelper::cloudi2cloud(cloud, g_status._cloud_src);
        g_pv.updatePointCloud(g_status._cloud_src);
        return;
    }
    PointCloudHelper::cloudi2cloud(cloud, cloudTemp);

    cout << "src size :" << cloud->size() << "  input size: " << cloudTemp->points.size() << endl;
    g_status._process_count++;
    bool isFirstProcess = true;

processagain:
    /////////////////////////////////////////////////////////////
    Matrix4f pairTransform;
    float error1 = PointCloudHelper::get_rot_icp(g_status._cloud_src, cloudTemp, pairTransform);

    cout << pairTransform << endl;
    //if (MatHelper::CheckRotValid(pairTransform) == false)
    //{
    //  cloudTemp = CloudPtr(new Cloud);
    //  return;
    //}
    PointCloudHelper::add_trace_point(cloudTemp);
    Matrix4f GlobalTransform2 = pairTransform* g_status._global_transform;
    pcl::transformPointCloud(*cloudTemp, *cloudTras, GlobalTransform2);

    if (g_status._cloud_result->size() == 0) {
        *g_status._cloud_cache = *g_status._cloud_src;
        *g_status._cloud_result = *g_status._cloud_src;
        FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, Matrix4f::Identity(), false,
            cloud->header.stamp / 1000);
    }


    float error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloudTras, pairTransform);
    //CloudPtr cloudTmp(new Cloud);
    //GetCloudLine(cloudTras, cloudTmp);
    //pcl::copyPointCloud(*cloudTmp, *cloudTras);
    //GetRotIcpPl(cloudCache, cloudTras, pairTransform);
    //GetRotIcp(cloudResult, cloudTras, pairTransform);
    cout << "...once icp dis " << error1 << endl;
    cout << "once rot matrix:" << endl << pairTransform << endl;




    //if (MatHelper::CheckRotValid(pairTransform) == false)
    //{
    //  cloudTemp = CloudPtr(new Cloud);
    //  return;
    //}

    Matrix4f pairTransformCache;
    pairTransformCache = g_status._global_transform.inverse() * pairTransform * GlobalTransform2;
    cout << "cloud cache:" << endl << pairTransformCache << endl;
    if (abs(pairTransformCache(0, 3)) > MAXTRACEDIS ||
        abs(pairTransformCache(1, 3)) > MAXTRACEDIS ||
        abs(pairTransformCache(2, 3)) > MAXTRACEDIS || abs(pairTransformCache(2, 2) - 1) > 0.1) {
            if (isFirstProcess) {
                printf("clear cache......................\n");
                cout << "clear cache......................" << endl;
                while (g_status._queue_cloud_tras.size() > 0) {
                    CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
                    g_status._queue_cloud_tras.pop();
                    *g_status._cloud_cache = *g_status._cloud_cache + *cloudCacheTrasTemp;
                }
                PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
                isFirstProcess = false;
                goto processagain;
            }
            //jake 暂时去掉位置纠正
            //else
            //{
            //  printf("stay......................\n");
            //  cout << "stay......................" << endl;
            //}
    } else {
        g_status._global_transform = pairTransform * GlobalTransform2;
    }

    pcl::transformPointCloud(*cloudTemp, *cloudTras, g_status._global_transform);
    FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, g_status._global_transform, true,
        cloud->header.stamp / 1000);

    cout << ".....final icp dis " << error2 << endl;
    cout << "final rot matrix:" << endl << g_status._global_transform << endl << endl;

    //loop
    //CloudPtr cloudLine(new Cloud);
    //GetCloudLine(cloudTras, cloudLine);
    //gCloudsLine.push_back(cloudLine);

    if (g_status._process_count < processCacheMinNum) {
        *g_status._cloud_cache = *g_status._cloud_cache + *cloudTras;
        PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
    } else {
        CloudPtr cloudCacheTras(new Cloud);
        pcl::copyPointCloud(*cloudTras, *cloudCacheTras);
        g_status._queue_cloud_tras.push(cloudCacheTras);
        if (g_status._queue_cloud_tras.size() >= processCacheMinNum) {
            CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
            g_status._queue_cloud_tras.pop();
            *g_status._cloud_cache = *g_status._cloud_cache + *cloudCacheTrasTemp;
            PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
        }
    }

    cout << g_status._ignore_frame_count << ":" << g_status._cloud_result->size() << endl;
    printf("gIgnoreFrameCount %d : %d", g_status._ignore_frame_count, g_status._cloud_result->size());

    g_status._cloud_src->clear();
    g_status._cloud_src = cloudTemp;
    cloudTemp = CloudPtr(new Cloud);
    //cloudTemp2 = CloudPtr(new Cloud);


    if (g_status._process_count % 10 == 0) {
        //RemoveDuplicate(cloudTras, RDUMPLICATE_VALUE);
        *g_status._cloud_result = *g_status._cloud_result + *cloudTras;
        PointCloudHelper::remove_duplicate(g_status._cloud_result, RDUMPLICATE_VALUE);
        g_pv.updatePointCloud(g_status._cloud_result);
        g_pv.spinOnce();
    }
}

void OnGetHdlCloudG2(const CloudIConstPtr& cloud){
    CloudPtr cloudTemp(new Cloud);
    CloudPtr cloudTemp2(new Cloud);
    CloudPtr cloudTempSrc(new Cloud);
    CloudPtr cloudTras(new Cloud);
    double dYaw, dPitch, dRoll;
    g_status._ignore_frame_count++;
    if (g_status._cloud_src->points.size() == 0) {
        PointCloudHelper::cloudi2cloud(cloud, g_status._cloud_src, 5, false, 40);
        g_pv.updatePointCloud(g_status._cloud_src);
        *g_status._cloud_cache = *g_status._cloud_src;
        *g_status._cloud_result = *g_status._cloud_src;
        ////imu
        //Matrix4f mtTmp = Matrix4f::Identity();
        //gImuTrans->GetTransactionMx(gCloudSrc->header.stamp / 1000, mtTmp, dYaw, dPitch, dRoll);
        //gMoveModel.UpdateModelLocal(mtTmp);
        //cout << "Imu transform:" << endl;
        //cout << mtTmp << endl;
        FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, Matrix4f::Identity(), false,
            cloud->header.stamp / 1000);
        return;
    }
    PointCloudHelper::cloudi2cloud(cloud, cloudTempSrc, 5, false, 40);
    PointCloudHelper::add_trace_point(cloudTempSrc);
    //imu
    Matrix4f mtTmp = Matrix4f::Identity();
    bool flag = gImuTrans->get_transaction_mx(cloudTempSrc->header.stamp / 1000, mtTmp, dYaw, dPitch,
            dRoll);
    //gMoveModel.UpdateModelLocal(mtTmp);
    if (flag) {
        mtTmp = mtTmp * g_status._last_mt;
        g_status._last_mt = mtTmp;
        pcl::transformPointCloud(*cloudTempSrc, *cloudTemp, mtTmp);
        //cout << "Imu transform:" << endl;
        //cout << mtTmp << endl;
    } else {
        g_status._last_mt = Matrix4f::Identity();
        pcl::copyPointCloud(*cloudTempSrc, *cloudTemp);
        cout << g_status._ignore_frame_count << " not find correspond imu stamp..................."
              << cloudTempSrc->header.stamp / 1000 << endl;
          printf("%d not find correspond imu stamp...................%ld\n", g_status._ignore_frame_count,
              cloudTempSrc->header.stamp / 1000);
    }
    g_status._process_count++;
    cout << g_status._process_count << endl;
    processagain:
    Matrix4f pairTransform;
    float error1 = PointCloudHelper::get_rot_icp(g_status._cloud_src, cloudTemp, pairTransform);

    Matrix4f GlobalTransform2 = pairTransform* mtTmp* g_status._global_transform;
    pcl::transformPointCloud(*cloudTempSrc, *cloudTras, GlobalTransform2);

    float error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloudTras, pairTransform);
    int minError = error2;
    Matrix4f minErrorPairTransform = pairTransform;
    Matrix4f minErrorAdjustTransform = Matrix4f::Identity();
    cout << "error2:" << error2 << endl;
    if (error2 > 0.1 || error2 < 0) {
        if (g_status._queue_cloud_tras.size() > 0) {
            CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
            g_status._queue_cloud_tras.pop();
            //CloudGrid::instance().UpdataGlobalTree(cloudCacheTrasTemp);
            //CloudGrid::instance().SeekCurrentCache(gCloudCache, gGlobalTransform);
            *g_status._cloud_cache = *g_status._cloud_cache + *cloudCacheTrasTemp;
            PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
            goto processagain;
        } else {
            if (cloudTemp2->size() == 0) {
                pcl::copyPointCloud(*cloudTras, *cloudTemp2);
                g_status._rotate_count = 1;
            }
            Matrix4f adjustTransform = Matrix4f::Identity();
processrotate : 
            MatHelper::get_adjust_tras(g_status._rotate_count, adjustTransform);
            g_status._rotate_count++;
            cout << ".........................." << g_status._rotate_count << endl;
            printf("...........................%d\n", g_status._rotate_count);
            if (g_status._rotate_count > 150) {
                g_status._skip_frame++;
                //if (minError<1.5)
                //{
                //  pairTransform = minErrorPairTransform;
                //  adjustTransform = minErrorAdjustTransform;
                //  GlobalTransform2 = adjustTransform*GlobalTransform2;
                //  goto processcontinue;
                //}
                stringstream ignoreframe;
                ignoreframe << cloud->header.stamp / 1000;
                FileHelper::WriteFile(g_status._pcap_file + string("_ignoreframe.txt"), ignoreframe.str(),
                    true);
                //jake补充运动模型跳帧问题
                //gMoveModelAfter.UpdateModelLocal(mtTmp);
                return;
            }
            pcl::transformPointCloud(*cloudTemp2, *cloudTras, adjustTransform);
            error2 = PointCloudHelper::get_rot_icp(g_status._cloud_cache, cloudTras, pairTransform);
            cout << "error2: " << error2 << endl;
            if (error2 > 0.1 + g_status._skip_frame / 20.0 || error2 < 0) {
                goto processrotate;
            } else {
                GlobalTransform2 = adjustTransform * GlobalTransform2;
                goto processcontinue;
            }
            if (minError > error2) {
                minError = error2;
                minErrorPairTransform = pairTransform;
                minErrorAdjustTransform = adjustTransform;
            }
        }
    }
    processcontinue:
    //jake 检测是否过远
    //Matrix4f transNew = pairTransform*GlobalTransform2;
    //if (abs(transNew(0, 3) - gGlobalTransform(0, 3)) > 0.5*(gSkipFrame + 1) ||
    //  abs(transNew(1, 3) - gGlobalTransform(1, 3)) > 0.5*(gSkipFrame + 1) ||
    //  abs(transNew(2, 3) - gGlobalTransform(2, 3)) > 0.5*(gSkipFrame + 1))
    //{
    //  stringstream ignoreframe;
    //  ignoreframe << cloud->header.stamp / 1000;
    //  FileHelper::WriteFile(gPcapFile + string("_ignoreframe.txt"), ignoreframe.str(), true);
    //  return;
    //}

    g_status._global_transform = pairTransform * GlobalTransform2;
    g_move_model_after.UpdataModel(g_status._global_transform);
    //gMoveModel.SaveModelLocal(gGlobalTransform);
    pcl::transformPointCloud(*cloudTempSrc, *cloudTras, g_status._global_transform);
    //cout << gIgnoreFrameCount <<endl<< gGlobalTransform<<endl;
    FileHelper::WriteFileRot(g_status._ignore_frame_count, g_status._pcap_file, g_status._global_transform, true,
        cloud->header.stamp / 1000);
    g_status._skip_frame = 0;
    if (g_status._process_count < processCacheMinNum) {
        //CloudGrid::instance().UpdataGlobalTree(cloudTras);
        //CloudGrid::instance().SeekCurrentCache(gCloudCache, gGlobalTransform);
        *g_status._cloud_cache = *g_status._cloud_cache + *cloudTras;
        PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
    } else {
        CloudPtr cloudCacheTras(new Cloud);
        pcl::copyPointCloud(*cloudTras, *cloudCacheTras);


        g_status._queue_cloud_tras.push(cloudCacheTras);
        if (g_status._queue_cloud_tras.size() >= processCacheMinNum) {
            CloudPtr cloudCacheTrasTemp = g_status._queue_cloud_tras.front();
            g_status._queue_cloud_tras.pop();
            //CloudGrid::instance().UpdataGlobalTree(cloudCacheTrasTemp);
            //CloudGrid::instance().SeekCurrentCache(gCloudCache, gGlobalTransform);
            *g_status._cloud_cache = *g_status._cloud_cache + *cloudTras;
            PointCloudHelper::get_reduced_cloud(g_status._cloud_cache, g_status._global_transform);
        }
    }
    //*gCloudResult = *gCloudResult + *cloudTras;
    cout << g_status._ignore_frame_count << endl;
    printf("%d\n", g_status._ignore_frame_count);
    g_status._cloud_src->clear();
    g_status._cloud_src = cloudTempSrc;
    cloudTempSrc = CloudPtr(new Cloud);
    g_status._last_mt = Matrix4f::Identity();

    if (g_status._process_count % 10 == 0) {
        *g_status._cloud_result = *g_status._cloud_result + *cloudTras;
        PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.3);
        //CloudGrid::instance().GetResultCloud(gCloudResult);
        g_pv.updatePointCloud(g_status._cloud_result);
    }
    g_pv.spinOnce();
}


*/


/*




//pcl::registration::LUM<CloudItem> lum;
//void DoLum()
//{
//  float dist = 100;
//  pcl::registration::LUM<CloudItem> lum;
//  lum.setMaxIterations(1000);
//  lum.setConvergenceThreshold(3.0f);
//
//  for (size_t i = 0; i < cloudsLine.size(); i++)
//  {
//      lum.addPointCloud(cloudsLine[i]);
//  }
//
//  //for (size_t i = 1; i < cloudsLine.size(); i++)
//  //  for (size_t j = 0; j < i; j++)
//  //  {
//  //  Eigen::Vector4f ci, cj;
//  //  //pcl::compute3DCentroid(*(cloudsLine[i]), ci);
//  //  //pcl::compute3DCentroid(*(cloudsLine[j]), cj);
//  //  Matrix42Vector4(mapCloudsRot[i], ci);
//  //  Matrix42Vector4(mapCloudsRot[j], cj);
//  //  Eigen::Vector4f diff = ci - cj;
//
//  //  //std::cout << ci << std::endl;
//  //  //std::cout << cj << std::endl;
//  //  std::cout << i << " " << j << " " << diff.norm () << std::endl;
//
//  //  //if (diff.norm() < 5.0 && (i - j == 1 || i - j > 20))
//  //  int validFrameCount = 90;
//  //  if (diff.norm() < 1 && (i - j == 1 || i - j > validFrameCount))
//  //  {
//  //      if (i - j > validFrameCount)
//  //          std::cout << "add connection between " << i << " (" << "clouds[i].first" << ") and " << j << " (" << "clouds[j].first" << ")" << std::endl;
//  //      pcl::registration::CorrespondenceEstimation<CloudItem, CloudItem> ce;
//  //      ce.setInputTarget(cloudsLine[i]);
//  //      ce.setInputCloud(cloudsLine[j]);
//  //      pcl::CorrespondencesPtr corr(new pcl::Correspondences);
//  //      ce.determineCorrespondences(*corr, dist);
//  //      if (corr->size() > 2)
//  //          lum.setCorrespondences(j, i, corr);
//  //  }
//  //  }
//
//  int start, end;
//  start = 1;
//  end = 755;
//  std::cout << "add connection between " << start << " (" << "clouds[i].first" << ") and " << end << " (" << "clouds[j].first" << ")" << std::endl;
//  pcl::registration::CorrespondenceEstimation<CloudItem, CloudItem> ce;
//  ce.setInputTarget(cloudsLine[end]);
//  ce.setInputCloud(cloudsLine[start]);
//  pcl::CorrespondencesPtr corr(new pcl::Correspondences);
//  ce.determineCorrespondences(*corr, dist);
//  //if (corr->size() > 2)
//  lum.setCorrespondences(start, end, corr);
//
//  lum.compute();
//
//  for (int i = 0; i < lum.getNumVertices(); i++)
//  {
//      std::cout << i << ": " << lum.getTransformation(i) (0, 3) << " " << lum.getTransformation(i) (1, 3) << " " << lum.getTransformation(i) (2, 3) << std::endl;
//      cloudsLine[i] = lum.getTransformedCloud(i);
//  }
//
//}


*/


/*
void DoLumElch(list<int> &listFrameValid, vector<map<uint64_t, uint64_t>>  &mapLoops,
    vector<CloudPtr> &cloudsLine, map<int, Matrix4f> &mapCloudsRot, vector<Matrix4f> &optimizeRot)
{
    if (mapLoops.size()==0)
    {
        return;
    }
    pcl::registration::ELCH<CloudItem> elch;
    pcl::IterativeClosestPoint<CloudItem, CloudItem>::Ptr icp(new pcl::IterativeClosestPoint<CloudItem, CloudItem>);
    icp->setMaximumIterations(600);
    //icp->setMaxCorrespondenceDistance(0.003f);
    //icp->setRANSACOutlierRejectionThreshold(0.003f);
    icp->setMaxCorrespondenceDistance(30.0f);
    icp->setRANSACOutlierRejectionThreshold(30.0f);

    //jake test
    icp->setMaxCorrespondenceDistance(300.0f);
    icp->setRANSACOutlierRejectionThreshold(300.0f);

    elch.setReg(icp);

    for (size_t i = 0; i < cloudsLine.size(); i++)
    {
        elch.addPointCloud(cloudsLine[i]);
    }

    int first = -1, last = -1;
    if (mapLoops.size() != 0)
    {
        for (int i = 0; i < mapLoops.size(); i++)
        {
            map<int, int>::iterator it;
            for (it = mapLoops[i].begin(); it != mapLoops[i].end(); it++)
            {
                first = last = -1;

                int idx = 0;
                list<int>::iterator itList;
                bool isInverse = false;
                for (itList = listFrameValid.begin(); itList != listFrameValid.end(); itList++)
                {
                    if (*itList == abs(it->first))
                    {
                        first = idx;
                    }
                    if (*itList == abs(it->second))
                    {
                        last = idx;
                        if (it->second < 0)
                        {
                            isInverse = true;
                        }
                    }
                    idx++;
                }
                if (first == -1 || last == -1)
                {
                    cout << "error: loop idx not found in rots,please reset ....................................." << it->first << endl;
                    getchar();
                    return;
                }

                if (abs(first) > (cloudsLine.size() - 1) || abs(last) > (cloudsLine.size() - 1))
                {
                    cout << "loop idx unvalid ..." << it->first << "........." << it->second << endl;
                    continue;
                }

                elch.setLoopStart(first);
                elch.setLoopEnd(last);
                cout << "loop start " << first << " end " << last << endl;
                Matrix4f rotLoop = Matrix4f::Identity();
                if (cloudsLine[first]->size() == 1)
                {
                    //Matrix4f dTrans = (gMapCloudsRot.find(it->second))->second-(gMapCloudsRot.find(it->first))->second;
                    //rotLoop(0, 3) = dTrans(0, 3);
                    //rotLoop(1, 3) = dTrans(1, 3);
                    //rotLoop(2, 3) = dTrans(2, 3);
                    rotLoop = (mapCloudsRot.find(it->first))->second * (mapCloudsRot.find(it->second))->second.inverse();
                    cout << "loop transform" << endl << rotLoop << endl;
                }
                else
                {
                    float dis = GetRotIcp(cloudsLine[last], cloudsLine[first], rotLoop);
                    cout << "loop transform dis " << dis << endl;
                    cout << "loop transform" << endl << rotLoop << endl;
                }
                //if (isInverse)
                //{
                //  Matrix4f rotTemp=Matrix4f::Zero()-Matrix4f::Identity();
                //  rotLoop = rotTemp*rotLoop;
                //  rotLoop(3, 3) = 1;
                //  //rotLoop(0, 1) = 0 - rotLoop(0, 1);
                //  //rotLoop(1, 0) = 0 - rotLoop(1, 0);
                //  //rotLoop(1, 2) = 0 - rotLoop(1, 2);
                //  //rotLoop(2, 1) = 0 - rotLoop(2, 1);
                //  //rotLoop(0, 3) = 0 - rotLoop(0, 3);
                //  //rotLoop(1, 3) = 0 - rotLoop(1, 3);
                //  //rotLoop(2, 3) = 0 - rotLoop(2, 3);
                //  cout << "loop transform inv" << endl << rotLoop << endl;
                //}
                rotLoop = Matrix4f::Identity();
                elch.compute(optimizeRot, rotLoop);
            }
        }
    }
    else
    {
        first = 1;
        last = cloudsLine.size() - 5;
        elch.setLoopStart(first);
        elch.setLoopEnd(last);
        cout << "loop start " << first << " end " << last;
        Matrix4f rotLoop;
        GetRotIcp(cloudsLine[first], cloudsLine[last], rotLoop);
        cout << "loop transform" << endl << rotLoop << endl;
        elch.compute(optimizeRot, rotLoop);
    }

    //for (size_t i = 0; i < cloudsLine.size(); i++)
    //{
    //  if (loopDetection(int(i), cloudsLine, LOOPDETECTTHRESHOLD, first, last))
    //  {
    //      std::cout << "Loop between " << first << " (" << i << ") and " << last << " (" << i<< ")" << std::endl;
    //      elch.setLoopStart(first);
    //      elch.setLoopEnd(last);
    //      elch.compute();
    //  }
    //}

}
*/