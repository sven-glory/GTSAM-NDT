//
//   The interface of class "CRawScan".
//

#pragma once

#include <vector>
#include <memory>
#include "Geometry.h"
#include "ScannerParam.h"

namespace sensor {

class CRawPointCloud
{
public:
    // Distance data in polar form in millimeter
    std::vector<unsigned short> distance;

    // Intensity data
    std::vector<unsigned int> intensity;

    unsigned long long timestamp_raw;   // �������ݵ�ԭʼʱ��
    unsigned long long timestamp_sync;  // �������ݵ�ͬ��ʱ��
    short              laser_type;      //0 - pf, 1 - hokuyo, 2 - sick
    short              laser_id;        // ɨ������ʶ
    unsigned int       num_points;      // ɨ������
    float              start_angle;     // ɨ����ʼ��(��)
    float              end_angle;       // ɨ����ֹ��(��)

public:
    CRawPointCloud()
    {
        Clear();
        timestamp_raw = 0;
        timestamp_sync = 0;
        laser_type = 0;
        laser_id = 0;
        num_points = 0;
        start_angle = 0.0;
        end_angle = 0.0;
    }

    ~CRawPointCloud()
    {
        Clear();
    }

    void Clear()
    {
        distance.clear();
        intensity.clear();
    }

    CRawPointCloud(const CRawPointCloud& other)
    {
        Clear();
        this->distance = other.distance;
        this->intensity = other.intensity;
        this->timestamp_raw = other.timestamp_raw;
        this->timestamp_sync = other.timestamp_sync;
        this->laser_type = other.laser_type;
        this->laser_id = other.laser_id;
        this->num_points = other.num_points;
        this->start_angle = other.start_angle;
        this->end_angle = other.end_angle;
    }

    CRawPointCloud(CRawPointCloud&& other)
    {
        Clear();
        this->distance = other.distance;
        this->intensity = other.intensity;
        this->timestamp_raw = other.timestamp_raw;
        this->timestamp_sync = other.timestamp_sync;
        this->laser_type = other.laser_type;
        this->laser_id = other.laser_id;
        this->num_points = other.num_points;
        this->start_angle = other.start_angle;
        this->end_angle = other.end_angle;
    }

    CRawPointCloud& operator = (const CRawPointCloud& Obj)
    {
        Clear();
        this->distance = Obj.distance;
        this->intensity = Obj.intensity;
        this->timestamp_raw = Obj.timestamp_raw;
        this->timestamp_sync = Obj.timestamp_sync;
        this->laser_type = Obj.laser_type;
        this->laser_id = Obj.laser_id;
        this->num_points = Obj.num_points;
        this->start_angle = Obj.start_angle;
        this->end_angle = Obj.end_angle;
        return *this;
    }

    CRawPointCloud& operator = (CRawPointCloud&& Obj)
    {
        Clear();
        this->distance = Obj.distance;
        this->intensity = Obj.intensity;
        this->timestamp_raw = Obj.timestamp_raw;
        this->timestamp_sync = Obj.timestamp_sync;
        this->laser_type = Obj.laser_type;
        this->laser_id = Obj.laser_id;
        this->num_points = Obj.num_points;
        this->start_angle = Obj.start_angle;
        this->end_angle = Obj.end_angle;
        return *this;
    }

    // �Ӷ������ļ��ж�ȡɨ������
    bool LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount, int nFileVersion);

    // ��ɨ�����ݱ��浽�������ļ�
    bool SaveBinary(FILE* fp, int nFileVersion);
};

class COdometryData
{
public:
    unsigned int odom_flag; // 4:����������̬���ݺ��ٶ���������
    unsigned long long time_stamp;
    CVelocity velocity;
    CPosture local_pst;
    CPosture global_pst;

public:
    COdometryData()
    {
        odom_flag = 4;
        time_stamp = 0;
    }

    ~COdometryData()
    {
        Clear();
    }

    void Clear()
    {
        odom_flag = 0;
        time_stamp = 0;
    }

    COdometryData(const COdometryData& other)
    {
        this->odom_flag = other.odom_flag;
        this->time_stamp = other.time_stamp;
        this->velocity = other.velocity;
        this->local_pst = other.local_pst;
        this->global_pst = other.global_pst;
    }

    COdometryData& operator = (const COdometryData& Obj)
    {
        this->odom_flag = Obj.odom_flag;
        this->time_stamp = Obj.time_stamp;
        this->velocity = Obj.velocity;
        this->local_pst = Obj.local_pst;
        this->global_pst = Obj.global_pst;
        return *this;
    }
};

using RawPointCloudVect = std::vector<std::shared_ptr<CRawPointCloud>>;
class CRawScan
{
public:
    RawPointCloudVect point_cloud;
    COdometryData odom_data;

public:
    CRawScan()
    {
        point_cloud.clear();
        odom_data.Clear();
    }

    ~CRawScan()
    {
        Clear();
    }

    CRawScan(const CRawScan& other)
    {
        Clear();
        this->point_cloud = other.point_cloud;
        //this->point_cloud.assign(other.point_cloud.begin(), other.point_cloud.end());
        this->odom_data = other.odom_data;
    }

    CRawScan& operator = (const CRawScan& Obj)
    {
        Clear();
        this->point_cloud = Obj.point_cloud;
        //this->point_cloud.assign(Obj.point_cloud.begin(), Obj.point_cloud.end());
        this->odom_data = Obj.odom_data;
        return *this;
    }

    void Clear()
    {
        for (unsigned int i = 0; i < point_cloud.size(); i++) {
            if(point_cloud[i]) {
                point_cloud[i].reset();
                point_cloud[i] = nullptr;
            }
        }
        point_cloud.clear();
    }

    bool SetOdometry(COdometryData& odom)
    {
        odom_data = odom;
        return true;
    }

    bool SetPointCloud(RawPointCloudVect& pCloud)
    {
        point_cloud = pCloud;
        return true;
    }

    bool PushBackPointCloud(std::shared_ptr<CRawPointCloud>& pCloud)
    {
        point_cloud.push_back(pCloud);
        return true;
    }

    // �Ӷ������ļ��ж�ȡɨ������
    bool LoadBinary(FILE* fp, const CPosture& pstRobot, const CScannerGroupParam& ScannerParam, int nFileVersion);

    // ��ɨ�����ݱ��浽�������ļ�
    bool SaveBinary(FILE* fp, int nFileVersion);
};

} // namespace sensor

