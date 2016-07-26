#pragma once
#include <vector>

namespace cloud_icp_reg {
typedef __int64 int64_t;
typedef int int32_t;
typedef unsigned int uint32_t;

#define LINECOUNT 32
#define INFOLEN 4000
#ifndef CONST_PI
#define CONST_PI (3.14159265358979324)
#endif


#pragma pack (push)
#pragma pack (1)

struct FileHead {
    char head[3 * 8];
};

/* pcap头 */
struct PcapHeader {
    unsigned int Timestamp_h;
    unsigned int Timestamp_l;
    unsigned int Captured_len;
    unsigned int Packet_len;
};

/* 报文头数据 */
struct Ethernet {
    char data[42];
};

//data packet 1248

struct Laser {
    unsigned short Distance;
    unsigned char Intensity;
};

struct FireData {
    unsigned short LaserBlockID;
    unsigned short Rotational;
    Laser Lasers[LINECOUNT];
};

struct FireDatas {
    FireData FireDataBlock[12];
};

struct Gps {
    unsigned int Timestamp;
    unsigned char status_Type;
    unsigned char status_Value;
};

struct LidarData {
    Ethernet Head;
    FireDatas Firing;
    Gps GpsData;
};


//position packet

struct AttitudeData {
    char unusedData0[14];

    unsigned short gyro1;
    unsigned short temp1;
    unsigned short accel1x;
    unsigned short accel1y;

    unsigned short gyro2;
    unsigned short temp2;
    unsigned short accel2x;
    unsigned short accel2y;

    unsigned short gyro3;
    unsigned short temp3;
    unsigned short accel3x;
    unsigned short accel3y;

    char unusedData1[160];
};

struct GpsData {
    unsigned int GpsTimestamp;
    unsigned int nUnused;
    char NMEA[72];

    char unusedData[234];
};


struct PostionData {
    Ethernet Head;
    AttitudeData attData;
    GpsData gpsData;
};

struct posIdxForSort {
    unsigned int GpsTimestamp;
    int nContainIdx;
};

struct stLadarPoint {
    double x;   /* 激光器本身局部坐标系下的坐标值 */
    double y;
    double z;
    double distanse;   /* 采集点距离激光器中心距离 sqrt(x^2 + y^2 + z^) */
    double azimuth;       /* 扫描仪方位角*/
    unsigned char intensity;     /* 反射强度：范围0-255 */
    int laser_id;      /* 32E扫描仪内每线激光编号 */
    unsigned int timestamp;   /* 当前扫描点的ms级时间戳 */
};

/* IMU DATA header */
struct stIMULogHeader {
    char Sync[3];
    unsigned char MessageLength;
    unsigned short MessageID;
    unsigned short WeekNumber;
    uint32_t Milliseconds;
};
//C.2.26 RAWIMUS Short Raw IMU Data 325
struct stIMURaw325 {
    uint32_t Week;
    double SecondsIntoWeek;
    int32_t IMUstatus;
    int32_t ZAccelOutput;
    int32_t MinusYAccel;
    int32_t XAccel;
    int32_t ZGyro;
    int32_t MinusYGyro;
    int32_t XGyro;
};

struct stIMU {
    stIMULogHeader stHeader;
    stIMURaw325 stIMURaw;
};

struct stGPSheader {
    char acSync[3];
    unsigned char ucHeaderLen;
    unsigned short usMessageID;
    unsigned char cMessageType;
    unsigned char ucPortAddress;
    unsigned short usMessageLen;
    unsigned short usSequence;
    unsigned char ucIdleTime;
    unsigned char ucTimeStatus;
    unsigned short usWeek;
    uint32_t ulMilliseconds;
    uint32_t ulReceiverStatus;
    unsigned short usReceived;
    unsigned short usReceiverSWVersion;
};

/* Message ID: 423 */
struct stMessageID423 {
    uint32_t ulSolStatus;
    uint32_t ulPosType;
    double dLat;
    double dLon;
    double dHgt;
    float fUndulation;
    uint32_t ulDatumID;
    float fLats;
    float fLons;
    float fHgts;
    unsigned char acStnID[4];
    float fDiffage;
    float fSolage;
    unsigned char ucObs;
    unsigned char ucGPSL1;
    unsigned char ucL1;
    unsigned char ucL2;
    unsigned char aucReserved[4];
};
/* message id:101 */
struct stMessageID101 {
    uint32_t ulClockStatus;
    double dOffset;
    double dOffsetstd;
    double dUtcOffset;
    uint32_t ulUtcYear;
    unsigned char ucUtcMonth;
    unsigned char ucUtcDay;
    unsigned char ucUtcHour;
    unsigned char ucUtcMin;
    uint32_t ulUtcms;
    uint32_t ulUtcStatus;
};

/* message id:43 */
struct stMessageID43 {
    int32_t lObs;
    unsigned short usPRN;
    unsigned short usGlofreq;
    double dPsr;
    float fPsrstd;
    double dAdr;
    float fAdrstd;
    float fDopp;
    float fCNO;
    float fLocktime;
    uint32_t ulChtrstatus;
};

/* message id:42 */
struct stMessageID42 {
    uint32_t ulSolStat;
    uint32_t ulPosType;
    double dLat;
    double dLon;
    double dHgt;
    float fUndulation;
    uint32_t ulDatumID;
    float fLatstd;
    float fLonstd;
    float fHgtstd;
    char acStnid[4];
    float fDiffage;
    float fSolage;
    unsigned char ucSVs;
    unsigned char ucSolnSVs;
    unsigned char ucSolnL1SVs;
    unsigned char ucSolnMultiSVs;
    unsigned char unReserved;
    char cExtSolStat;
    char cReserved1;
    char cSigMask;
};

struct hardmap_file_head {
    char szFlag[4];
    int nHeadSize;
    int nVer;
    int nType;
    int nItemSize;
    int nItemCount;
    int nReserved;
};

struct hardmap_pcimu_pc {
    float x;
    float y;
    float z;
    unsigned char intensity;
    unsigned char laser_id;
    unsigned short Rotational;
};
#define  MAX_LADAR_BLOCK_ITEMCOUNT (32*12)
struct hardmap_pcimu {
    unsigned short nCount;
    double utc_timestamp;
    double fLongitude;
    double fLatitude;
    double fHEll;
    float fHeading;
    float fPitch;
    float fRoll;
    unsigned char Q;
    int ieoutid;
    unsigned int pc_time;
    double ieout_time;
    hardmap_pcimu_pc pc[MAX_LADAR_BLOCK_ITEMCOUNT];
};


#pragma pack (pop)


struct IMURawItem {
    uint32_t Week;
    double SecondsIntoWeek;
    double ax;
    double aym;
    double az;
    double gx;
    double gym;
    double gz;
};

struct IMURawGPS {
    uint32_t Week;
    double SecondsIntoWeek;
    double fLon;
    double fLat;
};

struct IeoutItem {
    __int64 gpstime_us;
    double fTimeUTC;
    double fLongitude;
    double fLatitude;
    double fHEll;
    double fHeading;
    double fPitch;
    double fRoll;
    double fVNorth;
    double fVEast;
    double fVUp;
    int nQ;
};


struct GPSInfo {
    double utc_timestamp;
    double pc_timestamp_fromhead;
    double fLongitude;
    double fLatitude;
    double fHEll;
};


struct LadarInfoData {
    double x;   /* 激光器本身局部坐标系下的坐标值 */
    double y;
    double z;
    double distanse;   /* 采集点距离激光器中心距离 sqrt(x^2 + y^2 + z^) */
    double azimuth;       /* 扫描仪方位角*/
    unsigned short Rotational;
    unsigned char intensity;     /* 反射强度：范围0-255 */
    unsigned char laser_id;      /* 32E扫描仪内每线激光编号 */
};
struct LadarInfo {
    double pc_timestamp_fromhead;   /* 当前扫描点的ms级时间戳 */
    double utc_timestamp;
    unsigned int pc_rawtimestamp;
    int nCount;
    LadarInfoData Data[12 * 32];
};


struct TimeIndexPc {
    __int64 nID;
    GPSInfo Info;
};
} // cloud_icp_reg