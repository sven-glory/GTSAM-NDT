#include <stdafx.h>

#undef max
#undef min

#include "ndt_fuser/ndt_fuser_node.h"
#include <functional>
#include <algorithm>
#include "time_patch.h"

#define MAX_TRANSLATION_DELTA            0.5
#define MAX_ROTATION_DELTA               0.5



//////////////////////////////////////////////////////////////////////////////

CMapFuser::CMapFuser(double map_size_x_, double map_size_y_, double map_reso)
{
	map = NULL;

	sensor_max_range = MAX_SENSOR_RANGE;
	sensor_min_range = MIN_SENSOR_RANGE;

	map_size_x = map_size_x_;
	map_size_y = map_size_y_;
	map_size_z = 0.4;
	localMapSize << sensor_max_range, sensor_max_range, map_size_z;

	resolution = map_reso;
	resolution_local_factor = 1;


	matcher2D.ITR_MAX = 30;
	matcher2D.n_neighbours = 2;

	// 设置机器人初始姿态
	pose_ = Eigen::Translation<double, 3>(pose_init_x, pose_init_y, pose_init_z)*
		Eigen::AngleAxis<double>(pose_init_r, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pose_init_p, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(pose_init_t, Eigen::Vector3d::UnitZ());

	pose_.setIdentity();

	// 设置传感器初始姿态
	sensor_pose = Eigen::Translation<double, 3>(sensor_pose_x, sensor_pose_y, sensor_pose_z)*
		Eigen::AngleAxis<double>(sensor_pose_r, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(sensor_pose_p, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(sensor_pose_t, Eigen::Vector3d::UnitZ());
	sensor_pose.setIdentity();

	translation_fuse_delta = 0.0;
	rotation_fuse_delta = 0.0;
	max_translation_norm = 1.;
	max_rotation_norm = M_PI / 4;
	checkConsistency = false;

	isInit = false;
	count_clouds = 0;
	localizeOnly = false;
}

CMapFuser::~CMapFuser()
{
	if (map != NULL)
		delete map;
}

//
//   设置/取消“仅定位”模式。
//
void CMapFuser::SetLocalizationMode(bool _localizeOnly)
{
	Tnow = pose_;
	localizeOnly = _localizeOnly;
	count_clouds = 0;
	historyPoses.clear();
}

//
//   处理新的一帧数据。
//
Eigen::Affine3d CMapFuser::processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion, BOOL& bLocateState, int& nCountGood,
	int& nSourceNDT)
{
	// 如果这是第一帧，需要特殊处理
	if (count_clouds++ == 0)
	{
		initialize(pose_, cloud);

		// 记录新姿态
		historyPoses.push_back(pose_);
	}
		
	// 如果不是第一帧
	else 
	{
		// 如果里程计位姿变化太小，不处理
		/*if ((Tmotion.translation().norm() < 0.01 && Tmotion.rotation().eulerAngles(0, 1, 2)(2) < 0.01))
			return;*/

		// 如果里程计位移变化过大，或转角变化过大，忽略该帧
		/*if (Tmotion.translation().norm() > MAX_TRANSLATION_DELTA || 
			 Tmotion.rotation().eulerAngles(0, 1, 2)(2) > MAX_ROTATION_DELTA)
			Tmotion.setIdentity();*/

		pose_ = update(Tmotion, cloud, bLocateState, nCountGood,
			nSourceNDT);
		
		// 记录新姿态
		historyPoses.push_back(pose_);
	}
	return pose_;
}

//
//   对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化。
//
void CMapFuser::FilterCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, 
	pcl::PointCloud<pcl::PointXYZ> &cloud_filtered)
{
	const double varz = 0.05;

	cloud_filtered.clear();
	pcl::PointXYZ pt;
	for (int i = 0; i < cloud_in.points.size(); i++)
	{
		pt = cloud_in.points[i];
		double d = sqrt(pt.x*pt.x + pt.y*pt.y);

		// 滤除距离太近的点
		if (d > sensor_min_range)
		{
			pt.z += varz*((double)rand()) / (double)INT_MAX;
			cloud_filtered.points.push_back(pt);
		}
	}
}

//
//   根据里程姿态和扫描点云进行定位。
//
bool CMapFuser::Localize(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d& odometry, int& nCountGood,
	int& nSourceNDT)
{
	// 先复制点云，同时滤除过近的点
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	FilterCloud(cloud_in, cloud);

	// 将基于传感觉器坐标系的点云数据转换到基于机器人坐标系内
	perception_oru::transformPointCloudInPlace(sensor_pose, cloud);

	// 生成局部地图
	perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution * resolution_local_factor));
	ndlocal.guessSize(0, 0, 0, sensor_max_range, sensor_max_range, map_size_z);
	ndlocal.loadPointCloud(cloud, sensor_max_range);
	ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	// 测试将局部图与模型地图进行匹配，如果成功，匹配结果姿态保存于odometry中
	return matcher2D.match(*map, ndlocal, odometry, nCountGood, nSourceNDT, true);
}

//
//   根据里程姿态和扫描点云进行定位(NDT-MCL)。
//
bool CMapFuser::LocalizeMCL(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d& Tmotion, Eigen::Affine3d& estimate)
{
#if 0
	// 取得位姿变化量
//	Eigen::Affine3d Tmotion = last_odom.inverse() * odometry;

	// 将基于传感觉器坐标系的点云数据转换到基于机器人坐标系内
	perception_oru::transformPointCloudInPlace(sensor_pose, cloud);

	ndtmcl->updateAndPredict(Tmotion, cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)

	Eigen::Vector3d dm = ndtmcl->getMean(); ///Maximum aposteriori pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///Pose covariance

	// 测试将局部图与模型地图进行匹配，如果成功，匹配结果姿态保存于odometry中
	estimate = Eigen::Translation<double, 3>(dm(0), dm(1), 0) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(dm(2), Eigen::Vector3d::UnitZ());
	
//	last_odom = odometry;
#endif
	return true;
}

// 
//   根据新的数据，仅进行定位处理。
//
Eigen::Affine3d CMapFuser::JustLocalize(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion)
{
	// 根据姿态变化更新里程姿态 (we track this only for display purposes!)
	Todom = Todom * Tmotion;
	Eigen::Affine3d Tinit = Tnow * Tmotion;

	// 测试将局部图与模型地图进行匹配，如果成功，匹配结果姿态保存于odometry中
#ifdef LOCALIZE_NEWTON
	int nCountGood = 0;
	int nSourceNDT = 0;
	if (Localize(cloud, Tinit, nCountGood,
		nSourceNDT))
#elif defined LOCALIZE_NDT_MCL 
	if (LocalizeMCL(cloud, Tmotion, Tinit))
#endif
	{
		Tnow = Tinit;

		// 记录新姿态
		historyPoses.push_back(Tnow);
	}
	count_clouds++;
	return Tnow;
}

//
//   接收到里程和激光传感器数据后的回调函数。
//
Eigen::Affine3d CMapFuser::laserOdomCallback(Eigen::Affine3d& odometry, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_unfiltered, BOOL& bLocateState, int& nCountGood,
	int& nSourceNDT)
{
	const double varz = 0.05;

	Eigen::Affine3d Tm;
	if (count_clouds == 0)
		Tm.setIdentity();
	else 
		Tm = last_odom.inverse() * odometry;

	last_odom = odometry;

	// add some variance on z (为什么需要这样处理?)
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	FilterCloud(pcl_cloud_unfiltered, pcl_cloud);

	Eigen::Affine3d retpos;

	// 如果处于“仅定位”模式
	if (localizeOnly)
	{
#if 0
		Tm.translation()(0) = 0; // -0.2;
		Tm.translation()(1) = 0;
//		Tm.rotation().eulerAngles(0, 1, 2)(2) = 0;
#endif
		retpos = JustLocalize(pcl_cloud, Tm);
	}

	// 如果处于“建图”模式
	else
		retpos = processFrame(pcl_cloud, Tm, bLocateState, nCountGood,
		nSourceNDT);

	return retpos;
}

//
// 为地图设置初始姿态及第一帧点云
//
void CMapFuser::initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	// Set the cloud to sensor frame with respect to base
	// 将基于传感觉器坐标系的点云数据转换到基于机器人坐标系内
	perception_oru::transformPointCloudInPlace(sensor_pose, cloud);
	perception_oru::transformPointCloudInPlace(initPos, cloud);
	Tnow = initPos;
	//Tnow.setIdentity();

	// 生成地图对象，然后对其初始化
	map = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	map->initialize(Tnow.translation()(0), Tnow.translation()(1), Tnow.translation()(2), map_size_x, map_size_y, map_size_z);

	// 加入第一帧点云
	map->addPointCloud(Tnow.translation(), cloud, 0.1, 100.0, 0.1);

	// 生成NDT单元
	map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);

	isInit = true;
	Tlast_fuse = Tnow;
	Todom = Tnow;
}

//
//   根据机器人位姿变化和接收到的点云，进行相应的更新处理。
//
Eigen::Affine3d CMapFuser::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, BOOL& bLocationState, int& nCountGood,
	int& nSourceNDT)
{
	if (!isInit) 
		return Tnow;

	// 根据姿态变化更新里程姿态 (we track this only for display purposes!)
	Todom = Todom * Tmotion;
	Eigen::Affine3d Tinit = Tnow * Tmotion;

	double d1 = getDoubleTime();

	// 测试将局部图与模型地图进行匹配，如果成功，匹配结果姿态保存于Tinit中
	if (Localize(cloud, Tinit, nCountGood,
		nSourceNDT))
	{
		bLocationState = TRUE;
		double d2 = getDoubleTime() - d1;

		// 计算估测姿态与配准姿态之间的差值
		Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;

		// 如果上面的差值太大，则认为配准结果有误
		if ((diff.translation().norm() > max_translation_norm ||
			diff.rotation().eulerAngles(0, 1, 2).norm() > max_rotation_norm) && checkConsistency) 
		{
			// 配准姿态不可信，仍以里程估计姿态作为结果姿态
			Tnow = Tnow * Tmotion;
		}

		// 否则，配准结果可信
		else
		{
			// 采信刚刚得到的配准姿态
			Tnow = Tinit;
			double x = Tnow.translation().transpose()(0);
			double y = Tnow.translation().transpose()(1);
			double theta = Tnow.rotation().eulerAngles(0, 1, 2)(2);

			perception_oru::transformPointCloudInPlace(Tnow, cloud);

			// 得到传感器的姿态
			Eigen::Affine3d spose = Tnow * sensor_pose;

			// 计算姿态的变化量
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse() * Tnow;

			if (diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0, 1, 2).norm() > rotation_fuse_delta)
			{
				// 把局部点云按其均值添加到图中
				map->addPointCloudMeanUpdate(spose.translation(), cloud, localMapSize, 1e5, 5, 2 * map_size_z, 0.06);

				/////////////FIXME: check if this works for error beams //////////////////
				// 收集超出距离的激光点
				pcl::PointCloud<pcl::PointXYZ> max_beams;
				for (int i = 0; i < cloud.points.size(); i++) 
				{
					pcl::PointXYZ pt = cloud.points[i];
					double d = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
					if (d >= sensor_max_range) 
						max_beams.points.push_back(pt);
				}

				map->addPointCloud(spose.translation(), max_beams, 0.06, 100, 0.25);
				Tlast_fuse = Tnow;
			}
		}
	}
	else
	{
		bLocationState = FALSE;
		Tnow = Tnow * Tmotion;
	}
		

	return Tnow;
}

//
//   从文件中装入地图。
//
bool CMapFuser::LoadMap(const char* strFileName)
{
	if (map != NULL)
		delete map;

	map = new perception_oru::NDTMap(new perception_oru::LazyGrid);
	if (map == NULL)
		return false;

	if (!map->LoadFromFile(strFileName))
		return false;

#if 0
	// 如果地图装入成功，在此初始化ndt-mcl
	ndtmcl = new NDTMCL(resolution, *map, -0.5);
	if (ndtmcl == NULL)
		return false;

	int NOP = 100;     // 粒子数量

	ndtmcl->initializeFilter(0, 0, 0, 0.5, 0.2, 2.0*M_PI / 180.0, NOP);
	Tnow = last_odom = getAsAffine(0, 0, 0);
#endif

	return true;
}

//
//   将地图保存到文件。
//
bool CMapFuser::SaveMap(const char* strFileName)
{
	if (!isInit || map == NULL)
		return false;

	return (map->SaveToFile(strFileName) == 0);
}

void CMapFuser::ClearTemp()
{
	matcher2D.ClearTemp();
	map->ClearMatchStatus();
}

#ifdef _MSC_VER

//
//   绘制模型图。
//
void CMapFuser::PlotModelMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder, bool bShowMatched)
{
	map->Plot(pDC, ScrnRef, clrCellFill, clrCellBorder);
}


void CMapFuser::PlotSourceMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder, bool bShowMatched)
{

}

//
//   绘制位姿图。
//
void CMapFuser::PlotPoses(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected)
{
	historyPoses.Plot(pDC, ScrnRef, clrTraj, clrPoses, clrSelected);
}

#endif
