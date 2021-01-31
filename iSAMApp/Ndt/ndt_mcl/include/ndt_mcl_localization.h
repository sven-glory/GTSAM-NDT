#pragma once

#include "ndt_mcl.h"
#include <ndt_map/ndt_map.h>

class ndt_mcl_localization
{
public:
	bool userInitialPose = false;
	bool hasNewInitialPose = false;

	double ipos_x = 0, ipos_y = 0, ipos_yaw = 0;
	double ivar_x = 0, ivar_y = 0, ivar_yaw = 0;

	double time_end;
	std::string tf_odo_topic;
	std::string tf_laser_link;
	int NOP;

	NDTMCL *ndtmcl;
	perception_oru::NDTMap* ndmap;

	// Laser sensor offset
	float offx = 0;
	float offy = 0;
	float offa = 0;

	Eigen::Affine3d Told, Todo;

public:
	ndt_mcl_localization();
	
	bool initialize(perception_oru::NDTMap* ndmap_, double resolution);

	bool match(pcl::PointCloud<pcl::PointXYZ>& source,
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T);
};
