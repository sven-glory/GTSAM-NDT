#include <ndt_local.h>

extern int matchCycles;

void NDTLocal::initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::cout <<"Initializing NDT_LOCAL!" << std::endl;
	map_size_z = map->getmapsizez();
	Tlast_local = Tnow = initPos;
	isInit = true;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	// pass.setInputCloud(cloud);
	// pass.setFilterFieldName("x");
	// pass.setFilterLimits(-local_map_size, local_map_size);
	// pass.filter (*cloud1);
	// pass.setInputCloud(cloud1);
	// pass.setFilterFieldName("y");
	// pass.setFilterLimits(-local_map_size, local_map_size);
	// pass.filter (*cloud2);
	// lastcloud = cloud2;

	//local = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	// last->guessSize(0,0,0,local_map_size,local_map_size,map_size_z);
	// last->loadPointCloud(*lastcloud, local_map_size);
	// last->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	matcher2D = new perception_oru::NDTMatcherD2D_2D();

	std::cout << "Tinit :" << Tnow.translation().transpose() << " " << Tnow.rotation().eulerAngles(0,1,2)(2) << std::endl;

}

Eigen::Affine3d NDTLocal::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (!isInit)
	{
		std::cout << "NDT_LOCAL: Call Initialize first!!\n" << std::endl;
	    return Tnow;
	}
	double t1, t2, t3;

	perception_oru::transformPointCloudInPlace(sensor_pose, *cloud);

	t1 = getDoubleTime();

	perception_oru::NDTMap scan(new perception_oru::LazyGrid(resolution));

	scan.guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
	scan.loadPointCloud(*cloud, sensor_range);
	scan.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	t2 = getDoubleTime();
	t3 = t2;

	Eigen::Affine3d Tinit = Tnow * Tmotion;
	int  nCountGood ;
	int  nSourceNDT ;
	if (matcher2D->match(*map, scan, Tinit, nCountGood, nSourceNDT, true))
	{
		t3 = getDoubleTime();
		Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		if ((diff.translation().norm() > max_translation_norm || diff.rotation().eulerAngles(0, 1, 2).norm() > max_rotation_norm) && checkConsistency)
		{
			std::cerr << "****  NDT_LOCAL -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n" << std::endl;
			Tnow = Tnow * Tmotion;
			//Tnow = Tlast;
		}
		else
		{
			Tnow = Tinit;
			Eigen::Affine3d diff_fuse = Tlast_local.inverse()*Tnow;

			if (diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0, 1, 2).norm() > rotation_fuse_delta)
			{
				Tlast_local = Tnow;
				// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
				// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
				// pass.setInputCloud(cloud);
				// pass.setFilterFieldName("x");
				// pass.setFilterLimits(-local_map_size, local_map_size);
				// pass.filter (*cloud1);
				// std::cerr << "temp loaded" << std::endl;
				// pass.setInputCloud(cloud1);
				// pass.setFilterFieldName("y");
				// pass.setFilterLimits(-local_map_size, local_map_size);
				// pass.filter (*cloud2);
				// lastcloud = cloud2;
				// last->guessSize(0,0,0,local_map_size,local_map_size,map_size_z);
				// last->loadPointCloud(*cloud, local_map_size);
				// last->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
			}
		}
		std::cout << "load: " << (t2 - t1)*1e3 << " match: " << (t3 - t2)*1e3 << " total: " << (t3 - t1)*1e3 << std::endl;
		std::cout << "Match cycles: " << matchCycles << std::endl;
		//    FILE *ftmp = fopen("tmp.txt","a");
		//    fprintf(ftmp,"%lf, ",t4-t3);
		//    fclose(ftmp);
	}
	else
	{
		Tnow = Tnow * Tmotion;
		//Tnow =  Tnow;
	}

	return Tnow;
}
