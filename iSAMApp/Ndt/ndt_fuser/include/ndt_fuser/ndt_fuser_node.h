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

#define MAX_SENSOR_RANGE                 40                // 传感器最远有效距离
#define MIN_SENSOR_RANGE                 0.5               // 传感器最近有效距离

#define LOCALIZE_NEWTON                  1                 // 1-高斯牛顿法
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

	// 地图范围及分辨率
	double map_size_x;
	double map_size_y;
	double map_size_z;
	double resolution;
	Eigen::Vector3d localMapSize;

	// 传感器工作范围
	double sensor_max_range;         // 扫描最远距离
	double sensor_min_range;         // 扫描最近距离

	// 机器人初始位姿
	double pose_init_x;
	double pose_init_y;
	double pose_init_z;
	double pose_init_r;
	double pose_init_p;
	double pose_init_t;

	// 传感觉器安装位姿
	double sensor_pose_x;
	double sensor_pose_y;
	double sensor_pose_z;
	double sensor_pose_r;
	double sensor_pose_p;
	double sensor_pose_t;

	// 机器人里程姿态
	Eigen::Affine3d last_odom;
	Eigen::Affine3d sensor_pose;
	Eigen::Affine3d pose_;

	bool isInit;
	double translation_fuse_delta;
	double rotation_fuse_delta;
	double resolution_local_factor;

public:
	perception_oru::NDTMatcherD2D_2D matcher2D;      // NDT匹配器

public:
	Eigen::Affine3d Tnow, Tlast_fuse, Todom; // current pose
	perception_oru::NDTMap *map;		        // da map
	bool checkConsistency;			           // perform a check for consistency against initial estimate
	double max_translation_norm;
	double max_rotation_norm;
	CHistoryPoses historyPoses;
	bool  localizeOnly;                              // 标志：仅定位，不建模

	/*CCriticalSection         m_crit;*/
	
public:
	// 对输入的点云进行处理，滤除过近的点	
	void FilterCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_filtered);

public:
	CMapFuser(double map_size_x = DEFAULT_MAP_SIZE_X, double map_size_y = DEFAULT_MAP_SIZE_Y, double map_reso = DEFAULT_MAP_RESO);

	~CMapFuser();

	// 设置/取消“仅定位”模式
	void SetLocalizationMode(bool _localizeOnly);

	// 处理新的一帧数据
	Eigen::Affine3d processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d Tmotion, BOOL& bLocateState, int& nCountGood,
		int& nSourceNDT);

	// 根据里程姿态和扫描点云进行定位
	bool Localize(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d& odometry, int& nCountGood,
		int& nSourceNDT);

	bool LocalizeMCL(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d& Tmotion, Eigen::Affine3d& estimate);

	// 根据新的数据，仅进行定位处理
	Eigen::Affine3d JustLocalize(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d Tmotion);

	// 接收到里程和激光传感器数据后的回调函数
	Eigen::Affine3d laserOdomCallback(Eigen::Affine3d& odometry, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_unfiltered, BOOL& bLocateState, int& nCountGood,
		int& nSourceNDT);

	// 为地图设置初始姿态及第一帧点云
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud);

	// 根据机器人位姿变化和接收到的点云，进行相应的更新处理
	Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, BOOL& bLocationState, int& nCountGood,
		int& nSourceNDT);

	// 从文件中装入地图
	bool LoadMap(const char* strFileName);

	// 将地图保存到文件
	bool SaveMap(const char* strFileName);

	// 清除中间变量及状态
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
