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

	// ���û����˳�ʼ��̬
	pose_ = Eigen::Translation<double, 3>(pose_init_x, pose_init_y, pose_init_z)*
		Eigen::AngleAxis<double>(pose_init_r, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pose_init_p, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(pose_init_t, Eigen::Vector3d::UnitZ());

	pose_.setIdentity();

	// ���ô�������ʼ��̬
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
//   ����/ȡ��������λ��ģʽ��
//
void CMapFuser::SetLocalizationMode(bool _localizeOnly)
{
	Tnow = pose_;
	localizeOnly = _localizeOnly;
	count_clouds = 0;
	historyPoses.clear();
}

//
//   �����µ�һ֡���ݡ�
//
Eigen::Affine3d CMapFuser::processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion, BOOL& bLocateState, int& nCountGood,
	int& nSourceNDT)
{
	// ������ǵ�һ֡����Ҫ���⴦��
	if (count_clouds++ == 0)
	{
		initialize(pose_, cloud);

		// ��¼����̬
		historyPoses.push_back(pose_);
	}
		
	// ������ǵ�һ֡
	else 
	{
		// �����̼�λ�˱仯̫С��������
		/*if ((Tmotion.translation().norm() < 0.01 && Tmotion.rotation().eulerAngles(0, 1, 2)(2) < 0.01))
			return;*/

		// �����̼�λ�Ʊ仯���󣬻�ת�Ǳ仯���󣬺��Ը�֡
		/*if (Tmotion.translation().norm() > MAX_TRANSLATION_DELTA || 
			 Tmotion.rotation().eulerAngles(0, 1, 2)(2) > MAX_ROTATION_DELTA)
			Tmotion.setIdentity();*/

		pose_ = update(Tmotion, cloud, bLocateState, nCountGood,
			nSourceNDT);
		
		// ��¼����̬
		historyPoses.push_back(pose_);
	}
	return pose_;
}

//
//   ������ĵ��ƽ��д����˳������ĵ㣬����Z������г�ʼ����
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

		// �˳�����̫���ĵ�
		if (d > sensor_min_range)
		{
			pt.z += varz*((double)rand()) / (double)INT_MAX;
			cloud_filtered.points.push_back(pt);
		}
	}
}

//
//   ���������̬��ɨ����ƽ��ж�λ��
//
bool CMapFuser::Localize(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d& odometry, int& nCountGood,
	int& nSourceNDT)
{
	// �ȸ��Ƶ��ƣ�ͬʱ�˳������ĵ�
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	FilterCloud(cloud_in, cloud);

	// �����ڴ��о�������ϵ�ĵ�������ת�������ڻ���������ϵ��
	perception_oru::transformPointCloudInPlace(sensor_pose, cloud);

	// ���ɾֲ���ͼ
	perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution * resolution_local_factor));
	ndlocal.guessSize(0, 0, 0, sensor_max_range, sensor_max_range, map_size_z);
	ndlocal.loadPointCloud(cloud, sensor_max_range);
	ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	// ���Խ��ֲ�ͼ��ģ�͵�ͼ����ƥ�䣬����ɹ���ƥ������̬������odometry��
	return matcher2D.match(*map, ndlocal, odometry, nCountGood, nSourceNDT, true);
}

//
//   ���������̬��ɨ����ƽ��ж�λ(NDT-MCL)��
//
bool CMapFuser::LocalizeMCL(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d& Tmotion, Eigen::Affine3d& estimate)
{
#if 0
	// ȡ��λ�˱仯��
//	Eigen::Affine3d Tmotion = last_odom.inverse() * odometry;

	// �����ڴ��о�������ϵ�ĵ�������ת�������ڻ���������ϵ��
	perception_oru::transformPointCloudInPlace(sensor_pose, cloud);

	ndtmcl->updateAndPredict(Tmotion, cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)

	Eigen::Vector3d dm = ndtmcl->getMean(); ///Maximum aposteriori pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///Pose covariance

	// ���Խ��ֲ�ͼ��ģ�͵�ͼ����ƥ�䣬����ɹ���ƥ������̬������odometry��
	estimate = Eigen::Translation<double, 3>(dm(0), dm(1), 0) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(dm(2), Eigen::Vector3d::UnitZ());
	
//	last_odom = odometry;
#endif
	return true;
}

// 
//   �����µ����ݣ������ж�λ����
//
Eigen::Affine3d CMapFuser::JustLocalize(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion)
{
	// ������̬�仯���������̬ (we track this only for display purposes!)
	Todom = Todom * Tmotion;
	Eigen::Affine3d Tinit = Tnow * Tmotion;

	// ���Խ��ֲ�ͼ��ģ�͵�ͼ����ƥ�䣬����ɹ���ƥ������̬������odometry��
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

		// ��¼����̬
		historyPoses.push_back(Tnow);
	}
	count_clouds++;
	return Tnow;
}

//
//   ���յ���̺ͼ��⴫�������ݺ�Ļص�������
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

	// add some variance on z (Ϊʲô��Ҫ��������?)
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	FilterCloud(pcl_cloud_unfiltered, pcl_cloud);

	Eigen::Affine3d retpos;

	// ������ڡ�����λ��ģʽ
	if (localizeOnly)
	{
#if 0
		Tm.translation()(0) = 0; // -0.2;
		Tm.translation()(1) = 0;
//		Tm.rotation().eulerAngles(0, 1, 2)(2) = 0;
#endif
		retpos = JustLocalize(pcl_cloud, Tm);
	}

	// ������ڡ���ͼ��ģʽ
	else
		retpos = processFrame(pcl_cloud, Tm, bLocateState, nCountGood,
		nSourceNDT);

	return retpos;
}

//
// Ϊ��ͼ���ó�ʼ��̬����һ֡����
//
void CMapFuser::initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	// Set the cloud to sensor frame with respect to base
	// �����ڴ��о�������ϵ�ĵ�������ת�������ڻ���������ϵ��
	perception_oru::transformPointCloudInPlace(sensor_pose, cloud);
	perception_oru::transformPointCloudInPlace(initPos, cloud);
	Tnow = initPos;
	//Tnow.setIdentity();

	// ���ɵ�ͼ����Ȼ������ʼ��
	map = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	map->initialize(Tnow.translation()(0), Tnow.translation()(1), Tnow.translation()(2), map_size_x, map_size_y, map_size_z);

	// �����һ֡����
	map->addPointCloud(Tnow.translation(), cloud, 0.1, 100.0, 0.1);

	// ����NDT��Ԫ
	map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);

	isInit = true;
	Tlast_fuse = Tnow;
	Todom = Tnow;
}

//
//   ���ݻ�����λ�˱仯�ͽ��յ��ĵ��ƣ�������Ӧ�ĸ��´���
//
Eigen::Affine3d CMapFuser::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, BOOL& bLocationState, int& nCountGood,
	int& nSourceNDT)
{
	if (!isInit) 
		return Tnow;

	// ������̬�仯���������̬ (we track this only for display purposes!)
	Todom = Todom * Tmotion;
	Eigen::Affine3d Tinit = Tnow * Tmotion;

	double d1 = getDoubleTime();

	// ���Խ��ֲ�ͼ��ģ�͵�ͼ����ƥ�䣬����ɹ���ƥ������̬������Tinit��
	if (Localize(cloud, Tinit, nCountGood,
		nSourceNDT))
	{
		bLocationState = TRUE;
		double d2 = getDoubleTime() - d1;

		// ���������̬����׼��̬֮��Ĳ�ֵ
		Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;

		// �������Ĳ�ֵ̫������Ϊ��׼�������
		if ((diff.translation().norm() > max_translation_norm ||
			diff.rotation().eulerAngles(0, 1, 2).norm() > max_rotation_norm) && checkConsistency) 
		{
			// ��׼��̬�����ţ�������̹�����̬��Ϊ�����̬
			Tnow = Tnow * Tmotion;
		}

		// ������׼�������
		else
		{
			// ���Ÿոյõ�����׼��̬
			Tnow = Tinit;
			double x = Tnow.translation().transpose()(0);
			double y = Tnow.translation().transpose()(1);
			double theta = Tnow.rotation().eulerAngles(0, 1, 2)(2);

			perception_oru::transformPointCloudInPlace(Tnow, cloud);

			// �õ�����������̬
			Eigen::Affine3d spose = Tnow * sensor_pose;

			// ������̬�ı仯��
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse() * Tnow;

			if (diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0, 1, 2).norm() > rotation_fuse_delta)
			{
				// �Ѿֲ����ư����ֵ��ӵ�ͼ��
				map->addPointCloudMeanUpdate(spose.translation(), cloud, localMapSize, 1e5, 5, 2 * map_size_z, 0.06);

				/////////////FIXME: check if this works for error beams //////////////////
				// �ռ���������ļ����
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
//   ���ļ���װ���ͼ��
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
	// �����ͼװ��ɹ����ڴ˳�ʼ��ndt-mcl
	ndtmcl = new NDTMCL(resolution, *map, -0.5);
	if (ndtmcl == NULL)
		return false;

	int NOP = 100;     // ��������

	ndtmcl->initializeFilter(0, 0, 0, 0.5, 0.2, 2.0*M_PI / 180.0, NOP);
	Tnow = last_odom = getAsAffine(0, 0, 0);
#endif

	return true;
}

//
//   ����ͼ���浽�ļ���
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
//   ����ģ��ͼ��
//
void CMapFuser::PlotModelMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder, bool bShowMatched)
{
	map->Plot(pDC, ScrnRef, clrCellFill, clrCellBorder);
}


void CMapFuser::PlotSourceMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder, bool bShowMatched)
{

}

//
//   ����λ��ͼ��
//
void CMapFuser::PlotPoses(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected)
{
	historyPoses.Plot(pDC, ScrnRef, clrTraj, clrPoses, clrSelected);
}

#endif
