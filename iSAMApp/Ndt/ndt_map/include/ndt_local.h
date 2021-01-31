#ifndef NDT_LOCAL_H_
#define NDT_LOCAL_H_

#include <ndt_map/ndt_map.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_map/pointcloud_utils.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "time_patch.h"


class NDTLocal
{
public:
	Eigen::Affine3d Tnow, Tlast_local, Todom; ///< current pose
	perception_oru::NDTMap *map;  ///< da map
	bool checkConsistency; ///perform a check for consistency against initial estimate
	double max_translation_norm, max_rotation_norm;
	double sensor_range;

	NDTLocal(double resolution_, std::string mapName, double sensor_range_ = 10, double local_map_size_ = 6) {
		isInit = false;
		sensor_pose.setIdentity();
		checkConsistency = false;
		translation_fuse_delta = 0.05;
		rotation_fuse_delta = 0.01;
		max_translation_norm = 1.0;
		max_rotation_norm = M_PI / 4;
		sensor_range = sensor_range_;
		resolution = resolution_;
		local_map_size = local_map_size_;

		std::cout << "MAP: resolution: " << resolution << std::endl;
		map = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
		map->LoadFromFile(mapName.c_str());
		//last = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	}

	~NDTLocal()
	{
		delete map;
	}

#if 0
	double getDoubleTime()
	{
		struct timeval time;
		gettimeofday(&time, NULL);
		return time.tv_sec + time.tv_usec * 1e-6;
	}
#endif

	void setSensorPose(Eigen::Affine3d spose) 
	{
		sensor_pose = spose;
	}

	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ>::Ptr initcloud);

	Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
	bool isInit;

	double resolution;
	double local_map_size; ///< resolution of the map
	double map_size_z;

	double translation_fuse_delta, rotation_fuse_delta;

	Eigen::Affine3d sensor_pose;
	perception_oru::NDTMatcherD2D_2D* matcher2D;
	perception_oru::NDTMap *local;
	pcl::PointCloud<pcl::PointXYZ>::Ptr lastcloud;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
#endif