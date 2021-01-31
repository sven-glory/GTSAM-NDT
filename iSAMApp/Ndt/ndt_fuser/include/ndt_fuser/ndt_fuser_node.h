#pragma once

#include <ndt_map/ndt_map.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_map/pointcloud_utils.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <cstdio>
#include <fstream>
#include <vector>
#include "HistoryPoses.h"

using namespace std;

#define MAX_SENSOR_RANGE                 40                // ��������Զ��Ч����
#define MIN_SENSOR_RANGE                 0.5               // �����������Ч����

#define LOCALIZE_NEWTON                  1                 // 1-��˹ţ�ٷ�
//#define LOCALIZE_NDT_MCL                 2                 // 2-NDT-MCL

#ifdef _MSC_VER
class CDC;
class CScreenReference;
#endif

//Eigen::Affine3d getAsAffine(float x, float y, float yaw)
//{
//	Eigen::Matrix3d m;
//	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
//		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
//		* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
//	Eigen::Translation3d v(x, y, 0);
//	Eigen::Affine3d T = v*m;
//
//	return T;
//}

class CMapFuser
{
private:
	unsigned int count_clouds;

	// ��ͼ��Χ���ֱ���
	double map_size_x;
	double map_size_y;
	double map_size_z;
	double resolution;
	Eigen::Vector3d localMapSize;

	// ������������Χ
	double sensor_max_range;         // ɨ����Զ����
	double sensor_min_range;         // ɨ���������

	// �����˳�ʼλ��
	double pose_init_x;
	double pose_init_y;
	double pose_init_z;
	double pose_init_r;
	double pose_init_p;
	double pose_init_t;

	// ���о�����װλ��
	double sensor_pose_x;
	double sensor_pose_y;
	double sensor_pose_z;
	double sensor_pose_r;
	double sensor_pose_p;
	double sensor_pose_t;

	// �����������̬
	Eigen::Affine3d last_odom;
	Eigen::Affine3d sensor_pose;
	Eigen::Affine3d pose_;

	bool isInit;
	double translation_fuse_delta;
	double rotation_fuse_delta;
	double resolution_local_factor;

public:
	perception_oru::NDTMatcherD2D_2D matcher2D;      // NDTƥ����

public:
	Eigen::Affine3d Tnow, Tlast_fuse, Todom; // current pose
	perception_oru::NDTMap *map;		        // da map
	bool checkConsistency;			           // perform a check for consistency against initial estimate
	double max_translation_norm;
	double max_rotation_norm;
	CHistoryPoses historyPoses;
	bool  localizeOnly;                              // ��־������λ������ģ

	/*CCriticalSection         m_crit;*/
	
public:
	// ������ĵ��ƽ��д����˳������ĵ�	
	void FilterCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_filtered);

public:
	CMapFuser(double map_size_x = DEFAULT_MAP_SIZE_X, double map_size_y = DEFAULT_MAP_SIZE_Y, double map_reso = DEFAULT_MAP_RESO);

	~CMapFuser();

	// ����/ȡ��������λ��ģʽ
	void SetLocalizationMode(bool _localizeOnly);

	// �����µ�һ֡����
	Eigen::Affine3d processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d Tmotion, BOOL& bLocateState, int& nCountGood,
		int& nSourceNDT);

	// ���������̬��ɨ����ƽ��ж�λ
	bool Localize(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d& odometry, int& nCountGood,
		int& nSourceNDT);

	bool LocalizeMCL(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d& Tmotion, Eigen::Affine3d& estimate);

	// �����µ����ݣ������ж�λ����
	Eigen::Affine3d JustLocalize(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d Tmotion);

	// ���յ���̺ͼ��⴫�������ݺ�Ļص�����
	Eigen::Affine3d laserOdomCallback(Eigen::Affine3d& odometry, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_unfiltered, BOOL& bLocateState, int& nCountGood,
		int& nSourceNDT);

	// Ϊ��ͼ���ó�ʼ��̬����һ֡����
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud);

	// ���ݻ�����λ�˱仯�ͽ��յ��ĵ��ƣ�������Ӧ�ĸ��´���
	Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, BOOL& bLocationState, int& nCountGood,
		int& nSourceNDT);

	// ���ļ���װ���ͼ
	bool LoadMap(const char* strFileName);

	// ����ͼ���浽�ļ�
	bool SaveMap(const char* strFileName);

	// ����м������״̬
	void ClearTemp();

#ifdef _MSC_VER
	void PlotModelMap(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder, bool bShowMatched = false);
	void PlotSourceMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder, bool bShowMatched = false);
	void PlotPoses(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrTraj,
		unsigned long clrPoses, unsigned long clrSelected);
#endif

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
