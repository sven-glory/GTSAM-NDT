#include <stdafx.h>

#include "Geometry.h"
#include "ScrnRef.h"

#undef max
#undef min

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <string>
#include <climits>
#include <ndt_map/ndt_map.h>
#include <ndt_map/lazy_grid.h>
#include <time.h>
#include "time_patch.h"

///////////////////////////////////////////////////////////////////////////////

namespace perception_oru
{
	//
	//   结合给定的范围限制值，统计一个点云的重心点。
	//
	Eigen::Vector3d getCloudCentroid(const pcl::PointCloud<pcl::PointXYZ>& cloud, double range_limit)
	{
		Eigen::Vector3d centroid(0, 0, 0);           // 初始化重心点
		int count = 0;                               // 点的数量

		pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
		for (it = cloud.points.begin(); it != cloud.points.end(); it++)
		{
			// 跳过无效的点
			if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
				continue;

			Eigen::Vector3d d(it->x, it->y, it->z);

			// 跳过超距离的点
			if (range_limit > 0 && d.norm() > range_limit)
				continue;

			// 累计坐标值
			centroid += d;
			count++;
		}

		centroid /= (double)count;
		return centroid;
	}

	//
	//   结合给定的范围限制值，计算给定点云的范围参数及重心点。
	//
	Eigen::Vector3d getCloudDimension(const pcl::PointCloud<pcl::PointXYZ>& cloud, double range_limit,
		double& maxDist, double& minZ, double& maxZ)
	{
		// 先计算重点心
		Eigen::Vector3d centroid = getCloudCentroid(cloud, range_limit);

		minZ = 10000;
		maxZ = -10000;
		maxDist = 0;

		// 计算重心点到最远点的距离
		pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
		for (it = cloud.points.begin(); it != cloud.points.end(); it++)
		{
			// 跳过无效的点
			if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
				continue;

			// 跳过超距离的点
			Eigen::Vector3d d;
			if (range_limit > 0)
			{
				d << it->x, it->y, it->z;
				if (d.norm() > range_limit)
					continue;
			}

			// d为当前点到重心点的差值
			d << centroid(0) - it->x, centroid(1) - it->y, centroid(2) - it->z;
			double dz = centroid(2) - it->z;

			// dist为当前点到重心点的距离
			double dist = d.norm();

			// 更新范围极限值maxDist, maxZ, minZ
			if (dist > maxDist)
				maxDist = dist;

			if (maxZ < dz)
				maxZ = dz;

			if (minZ > dz)
				minZ = dz;
#if 0
			maxDist = (dist > maxDist) ? dist : maxDist;
			maxZ = ((centroid(2) - it->z) > maxZ) ? (centroid(2) - it->z) : maxZ;
			minZ = ((centroid(2) - it->z) < minZ) ? (centroid(2) - it->z) : minZ;
#endif
		}
	
		return centroid;
	}

	//////////////////////////////////////////////////////////////////////////////

	/**
	* loadPointCloud - You can call this if you are only interested in dealing with one scan
	* without need for fusing several ones or representing empty space and occupancy
	*
	* Otherwise you should always call addPointCloud (or if you don't want occupancy then addPointCloudSimple)
	*
	* \param pc the PointCloud that is to be loaded
	* \note every subsequent call will destroy the previous map!
	*/
	bool NDTMap::loadPointCloud(const pcl::PointCloud<pcl::PointXYZ> &pc, double range_limit)
	{
		// spatial index必须是已经初始化的
		if (index_ == NULL)
			return false;

		// 如果原来存在Spatial index，现在需要释放已有的单元
		SpatialIndex *si = index_->clone();
		if (si == NULL)
			return false;

		if (!isFirstLoad_)
			delete index_;
			
		isFirstLoad_ = false;
		index_ = si;

		// 仅支持LazyGrid格式
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		//set sizes
		NDTCell *ptCell = new NDTCell();
		if (ptCell == NULL)
			return false;

		index_->setCellType(ptCell);
		delete ptCell;

		// 如果全图尺寸尚不清楚
		if (guess_size_)
		{
			double maxDist, minz, maxz;
			Eigen::Vector3d centroid = getCloudDimension(pc, range_limit, maxDist, minz, maxz);

			index_->setCenter(centroid(0), centroid(1), centroid(2));

			if (map_sizex > 0 && map_sizey > 0 && map_sizez > 0)
				index_->setSize(map_sizex, map_sizey, map_sizez);
			else
				index_->setSize(4 * maxDist, 4 * maxDist, 3 * (maxz - minz));
		}
		else
		{
			index_->setCenter(centerx, centery, centerz);

			if (map_sizex > 0 && map_sizey > 0 && map_sizez > 0)
				index_->setSize(map_sizex, map_sizey, map_sizez);
		}

		pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
		for (it = pc.points.begin(); it != pc.points.end(); it++)
		{
			Eigen::Vector3d d;
			if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
				continue;

			if (range_limit > 0)
			{
				d << it->x, it->y, it->z;
				if (d.norm() > range_limit)
					continue;
			}

			index_->addPoint(*it);
			NDTCell *ptCell;
			lz->getNDTCellAt(*it, ptCell);

#ifdef REFACTORED
			if (ptCell != NULL) 
				update_set.insert(ptCell);
#endif
		}

		isFirstLoad_ = false;
		
		return true;
	}


	/**
	* loadPointCloudCentroid - A special load function to enable the matching of centroids (create alligned maps)
	* \param pc the PointCloud that is to be loaded
	* \note every subsequent call will destroy the previous map!
	*/
	bool NDTMap::loadPointCloudCentroid(const pcl::PointCloud<pcl::PointXYZ> &pc, const Eigen::Vector3d &origin,
		const Eigen::Vector3d &old_centroid, const Eigen::Vector3d &map_size, double range_limit) 
	{
		if (index_ == NULL)
			return false;

		SpatialIndex *si = index_->clone();
		if (si == NULL)
			return false;

		if (!isFirstLoad_) 
			delete index_;

		isFirstLoad_ = false;
		index_ = si;

		NDTCell *ptCell = new NDTCell();
		if (ptCell == NULL)
			return false;

		index_->setCellType(ptCell);
		delete ptCell;

		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		Eigen::Vector3d diff = origin - old_centroid;  //-origin;
		double cx = 0, cy = 0, cz = 0;
		lz->getCellSize(cx, cy, cz);

		// How many cell to each direction is the new origin from old one
		Eigen::Vector3d centroid;
		centroid(0) = old_centroid(0) + floor(diff(0) / cx) * cx;
		centroid(1) = old_centroid(1) + floor(diff(1) / cy) * cy;
		centroid(2) = old_centroid(2) + floor(diff(2) / cz) * cz;

		index_->setCenter(centroid(0), centroid(1), centroid(2));
		index_->setSize(map_size(0), map_size(1), map_size(2));

		pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
		for (it = pc.points.begin(); it != pc.points.end(); it++)
		{
			if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
				continue;

			if (range_limit > 0)
			{
				Eigen::Vector3d d(it->x, it->y, it->z);
				d = d - origin;

				if (d.norm() > range_limit)
					continue;
			}

			index_->addPoint(*it);
			NDTCell *ptCell = NULL;
			lz->getNDTCellAt(*it, ptCell);

#ifdef REFACTORED
			if (ptCell != NULL) 
				update_set.insert(ptCell);
#endif
		}

		isFirstLoad_ = false;
		return true;
	}

	/**
	* Just adds the points, without raytracing and such
	*/
	bool NDTMap::addPointCloudSimple(const pcl::PointCloud<pcl::PointXYZ> &pc, double maxz)
	{
		if (isFirstLoad_)
			return loadPointCloud(pc);

		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
		for (it = pc.points.begin(); it != pc.points.end(); it++)
		{
			// 滤除无效点
			if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
				continue;

			// 滤除超高的点
			if (it->z > maxz)
				continue;

			index_->addPoint(*it);
			NDTCell *ptCell;
			lz->getNDTCellAt(*it, ptCell);

#ifdef REFACTORED
			if (ptCell != NULL) 
				update_set.insert(ptCell);
#endif
		}
		return true;
	}

	/**
	* Add a distribution to the map
	*/
	bool NDTMap::addDistributionToCell(const Eigen::Matrix3d &ucov, const Eigen::Vector3d &umean, unsigned int numpointsindistribution,
		float r, float g, float b, unsigned int maxnumpoints, float max_occupancy)
	{
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		pcl::PointXYZ pt;
		pt.x = umean[0];
		pt.y = umean[1];
		pt.z = umean[2];
		
		NDTCell *ptCell = lz->getCellAtAllocate(pt);
		if (ptCell != NULL)
		{
			ptCell->updateSampleVariance(ucov, umean, numpointsindistribution, true, max_occupancy, maxnumpoints);
			ptCell->setRGB(r, g, b);
		}

		return true;
	}

	// Get the cell for which the point fall into (not the closest cell)
	bool NDTMap::getCellAtPoint(const pcl::PointXYZ &refPoint, NDTCell *&cell)
	{
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		lz->getNDTCellAt(refPoint, cell);
		return (cell != NULL);
	}

	///Get the cell for which the point fall into (not the closest cell)
	NDTCell* NDTMap::getCellAtAllocate(const pcl::PointXYZ &refPoint)
	{
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return NULL;

		return lz->getCellAtAllocate(refPoint);
	}

	/**
	* Adds a new cloud: NDT-OM update step
	*/
	/**
	* Add new pointcloud to map - This is the main interface for NDT-OM!
	* Performs raytracing, updates conflicts and adds points to cells
	* computeNDTCells must be called after calling this
	*
	* @param &origin is the position of the sensor, from where the scan has been taken from.
	* @param &pc is the pointcloud to be added
	* @param classifierTh A treshold to judge if the ray passes through a gaussian (obsolete)
	* @param maxz threshold for the maximum z-coordinate value for the measurement point_cloud
	* @param sensor_noise The expected standard deviation of the sensor noise
	*/
	bool NDTMap::addPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<pcl::PointXYZ> &pc, double classifierTh, double maxz,
		double sensor_noise, double occupancy_limit)
	{
		// 如果是第一个点云，直接装入
		if (isFirstLoad_)
			return loadPointCloud(pc);

		if (index_ == NULL)
			return false;

		// NDT-OM更新方法仅适用于LazyGrid格式
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		// 获取传感器所在的位置
		pcl::PointXYZ po;
		po.x = origin(0); 
		po.y = origin(1);
		po.z = origin(2);

		NDTCell* ptCell = NULL;

		std::vector< NDTCell*> cells;
		bool updatePositive = true;
		double max_range = 200;

		// 对点云中所有的点依次进行考察
		pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
		for (it = pc.points.begin(); it != pc.points.end(); it++)
		{
			// 跳过那些无效的点
			if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
				continue;

			// 计算点到传感器的距离
			Eigen::Vector3d diff;
			diff << it->x - origin(0), it->y - origin(1), it->z - origin(2);
			double l = diff.norm();

			// 如果距离超限，跳过
			if (l > max_range)
				continue;

			// 跟踪扫描线，收集线上的所有单元，结果保存于cells中
			cells.clear();
			if (!lz->traceLine(origin, *it, diff, maxz, cells)) 
				continue;

			// 依次考察该扫描线上的所有单元
			for (unsigned int i = 0; i < cells.size(); i++)
			{
				ptCell = cells[i];

				// 如果该单元不空
				if (ptCell != NULL)
				{
					double l2target = 0;

					// 如果该单元中有高斯记录
					if (ptCell->hasGaussian_)
					{
						Eigen::Vector3d out, pend, vpt;
						pend << it->x, it->y, it->z;

						double lik = ptCell->computeMaximumLikelihoodAlongLine(po, *it, out);
						l2target = (out - pend).norm();

						double dist = (origin - out).norm();
						if (dist > l) 
							continue; // < don't accept points further than the measurement

						l2target = (out - pend).norm(); // <distance to endpoint

						double sigma_dist = 0.5 * (dist / 30.0); // test for distance based sensor noise
						double snoise = sigma_dist + sensor_noise;
						double thr = exp(-0.5*(l2target*l2target) / (snoise*snoise)); // This is the probability of max lik point being endpoint
						lik *= (1.0 - thr);

						if (lik < 0.3) 
							continue;

						lik = 0.1*lik + 0.5; // Evidence value for empty - alpha * p(x);
						double logoddlik = log((1.0 - lik) / (lik));

						//ptCell->updateEmpty(logoddlik,l2target);
						//fprintf(stderr,"[E=%.2lf] ", logoddlik);
						ptCell->updateOccupancy(logoddlik, occupancy_limit);

						if (ptCell->getOccupancy() <= 0)
							ptCell->hasGaussian_ = false;
					}

					// 如果该单元没有高斯记录
					else
					{
						ptCell->updateOccupancy(-0.2, occupancy_limit);

						if (ptCell->getOccupancy() <= 0)
							ptCell->hasGaussian_ = false;
					}
				}
			}

			if (updatePositive) 
			{
				ptCell = dynamic_cast<NDTCell*>(index_->addPoint(*it));
				if (ptCell != NULL)
					update_set.insert(ptCell);
			}
		}

		isFirstLoad_ = false;
		return true;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	* Add new pointcloud to map - Updates the occupancy using the mean values of
	* a local map generated from an observation
	*
	* Performs raytracing, updates conflicts and adds points to cells
	* computeNDTCells must be called after calling this
	*
	* @param &origin is the position of the sensor, from where the scan has been taken from.
	* @param &pc is the pointcloud to be added
	* @param &localmapsize The dimensions of the local map used for computing the gaussians
	* @param maxnumpoints Defines the forgetting factor (default 100000) the smaller the value the faster the adaptation
	* @param occupancy_limit Clamping threshold for log-odds value
	* @param maxz threshold for the maximum z-coordinate value for the measurement point_cloud
	* @param sensor_noise The expected standard deviation of the sensor noise
	*/
	bool NDTMap::addPointCloudMeanUpdate(const Eigen::Vector3d &origin,
		const pcl::PointCloud<pcl::PointXYZ> &pc,
		const Eigen::Vector3d &localmapsize,
		unsigned int maxnumpoints, float occupancy_limit, double maxz, double sensor_noise)
	{
		// 如果是第一次
		if (isFirstLoad_)
		{
			loadPointCloud(pc);
			computeNDTCells();
			return true;
		}

		if (index_ == NULL)
			return false;

		// 仅支持LazyGrid型的图
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		if (lz == NULL)
			return false;

		// 获取中心点
		double centerX, centerY, centerZ;
		lz->getCenter(centerX, centerY, centerZ);

		// 取得单元尺寸
		double cellSizeX, cellSizeY, cellSizeZ;
		lz->getCellSize(cellSizeX, cellSizeY, cellSizeZ);

		// 取得在X、Y、Z方向上的单元数量
		int sizeX, sizeY, sizeZ;
		lz->getGridSize(sizeX, sizeY, sizeZ);

		Eigen::Vector3d old_centroid;
		old_centroid(0) = centerX;
		old_centroid(1) = centerY;
		old_centroid(2) = centerZ;

		double min1 = std::min(cellSizeX, cellSizeY);
		double min2 = std::min(cellSizeZ, cellSizeY);
		double resolution = std::min(min1, min2); // Select the smallest resolution

		// 先建立一个局部的NDT图
		perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution));
		ndlocal.loadPointCloudCentroid(pc, origin, old_centroid, localmapsize, /*70.0*/-1.); // No range limit used here, assume that the point cloud is already filtered.

		// 采用Sample variance方法生成局部NDT
		ndlocal.computeNDTCells();

		// 以向量的形式取出局部NDT图中的全部单元
		std::vector<perception_oru::NDTCell*> ndts;
		ndts = ndlocal.getAllCells();
		
		NDTCell *ptCell = NULL;

		pcl::PointXYZ pt;
		pcl::PointXYZ po;

		po.x = origin(0); 
		po.y = origin(1); 
		po.z = origin(2);

		int num_high = 0;
		for (unsigned int it = 0; it < ndts.size(); it++)
		{
			// 跳过NDT单元不存在、或者是其中没有高斯分布的单元(其实不应存在这样的单元)
			if (ndts[it] == NULL || !ndts[it]->hasGaussian_)
				continue;

			// 跳过单元内点数为0的单元
			int numpoints = ndts[it]->getN();
			if (numpoints <= 0)
				continue;

			Eigen::Vector3d diff;
			Eigen::Vector3d m = ndts[it]->getMean();
			diff = m - origin;

			double l = diff.norm();
			int NN = l / (resolution);

			// 点的距离不能过远(200???)
			if (l > 200)
				continue;

			// 分辨率不能过低
			if (resolution < 0.01) 
				continue;

			// 扫描线必须含有一定的单元数
			if (NN < 0)
				continue;

			bool updatePositive = true;

			// 如果此单元超高，则不作更新
			if (m(2) > maxz)
			{
				num_high++;
				updatePositive = false; ///Lets update negative even though the measurement was too high
			}

			diff = diff / (float)NN;

			int idxo = 0, idyo = 0, idzo = 0;
			for (int i = 0; i < NN - 2; i++)
			{
				pt.x = origin(0) + ((float)(i + 1)) *diff(0);
				pt.y = origin(1) + ((float)(i + 1)) *diff(1);
				pt.z = origin(2) + ((float)(i + 1)) *diff(2);
				int idx, idy, idz;

				idx = (int)(((pt.x - centerX) / cellSizeX + 0.5) + sizeX / 2.0);
				idy = (int)(((pt.y - centerY) / cellSizeY + 0.5) + sizeY / 2.0);
				idz = (int)(((pt.z - centerZ) / cellSizeZ + 0.5) + sizeZ / 2.0);


				///We only want to check every cell once, so
				///increase the index if we are still in the same cell
				if (idx == idxo && idy == idyo && idz == idzo)
					continue;
				else
				{
					idxo = idx; 
					idyo = idy; 
					idzo = idz;
				}

				ptCell = NULL;

				/// Check the validity of the index
				lz->getNDTCellAt(idx, idy, idz, ptCell);
				if (ptCell != NULL)
				{
//					ptCell->matchStatus = 1;

					double l2target = 0;
					if (ptCell->hasGaussian_)
					{
						Eigen::Vector3d out, pend, vpt;
						pend = m;

						double lik = ptCell->computeMaximumLikelihoodAlongLine(po, pt, out);
						l2target = (out - pend).norm();

						double dist = (origin - out).norm();
						if (dist > l)
							continue; ///< don't accept points further than the measurement

						l2target = (out - pend).norm(); ///<distance to endpoint

						double sigma_dist = 0.5 * (dist / 30.0); ///test for distance based sensor noise
																			  //double sigma_dist = 0;
						double snoise = sigma_dist + sensor_noise;
						double thr = exp(-0.5*(l2target*l2target) / (snoise*snoise)); ///This is the probability of max lik point being endpoint
						lik *= (1.0 - thr);
						lik = 0.2*lik + 0.5; ///Evidence value for empty - alpha * p(x);
						double logoddlik = log((1.0 - lik) / (lik));

						ptCell->updateOccupancy(numpoints*logoddlik, occupancy_limit);

						if (ptCell->getOccupancy()<0) 
							ptCell->hasGaussian_ = false;
					}
					else
					{
						ptCell->updateOccupancy(-0.85*numpoints, occupancy_limit); ///The cell does not have gaussian, so we mark that we saw it empty...
																									  //ptCell->updateEmpty(-0.2*numpoints,l2target); ///The cell does not have gaussian, so we mark that we saw it empty...
					}
				}
				else
				{
					ptCell = dynamic_cast<NDTCell*>(index_->addPoint(pt)); ///Add fake point to initialize!
//					ptCell->matchStatus = 1;
				}
			}

			if (updatePositive)
			{
				Eigen::Matrix3d ucov = ndts[it]->getCov();
				float r, g, b;
				ndts[it]->getRGB(r, g, b);
				addDistributionToCell(ucov, m, numpoints, r, g, b, maxnumpoints, occupancy_limit); ///<FIXME:: local implementation can be faster?
			}
		}

		for (unsigned int i = 0; i<ndts.size(); i++)
			delete ndts[i];

		isFirstLoad_ = false;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/**
	* Adds one measurement to the map using NDT-OM update step
	* @return true if an inconsistency was detected
	*/
	bool NDTMap::addMeasurement(const Eigen::Vector3d &origin, pcl::PointXYZ endpoint, double classifierTh, double maxz, double sensor_noise)
	{
		if (index_ == NULL)
			return false;

		bool retval = false;

		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		double centerX, centerY, centerZ;
		lz->getCenter(centerX, centerY, centerZ);
		double cellSizeX, cellSizeY, cellSizeZ;
		lz->getCellSize(cellSizeX, cellSizeY, cellSizeZ);
		int sizeX, sizeY, sizeZ;
		lz->getGridSize(sizeX, sizeY, sizeZ);

		double min1 = std::min(cellSizeX, cellSizeY);
		double min2 = std::min(cellSizeZ, cellSizeY);

		double resolution = std::min(min1, min2); // Select the smallest resolution

		if (lz == NULL)
		{
			fprintf(stderr, "NOT LAZY GRID!!!\n");
			exit(1);
		}

		NDTCell *ptCell = NULL;

		pcl::PointXYZ pt;
		pcl::PointXYZ po;
		po.x = origin(0), origin(1), origin(2);

		Eigen::Vector3d diff;
		diff << endpoint.x - origin(0), endpoint.y - origin(1), endpoint.z - origin(2);

		double l = diff.norm();
		if (l > 200)
		{
			fprintf(stderr, "addMeasurement::Very long distance (%lf) :( \n", l);
			return false;
		}

		if (resolution<0.01)
		{
			fprintf(stderr, "Resolution very very small (%lf) :( \n", resolution);
			return false;
		}

		int NN = l / (resolution);
		if (NN <= 0) 
			return false;

		diff = diff / (float)NN;

		bool updatePositive = true;
		if (endpoint.z>maxz)
			return false;

		int idxo = 0, idyo = 0, idzo = 0;
		for (int i = 0; i<NN - 2; i++)
		{
			pt.x = origin(0) + ((float)(i + 1)) *diff(0);
			pt.y = origin(1) + ((float)(i + 1)) *diff(1);
			pt.z = origin(2) + ((float)(i + 1)) *diff(2);
			int idx, idy, idz;

			idx = (int)(((pt.x - centerX) / cellSizeX + 0.5) + sizeX / 2);
			idy = (int)(((pt.y - centerY) / cellSizeY + 0.5) + sizeY / 2);
			idz = (int)(((pt.z - centerZ) / cellSizeZ + 0.5) + sizeZ / 2);

			// We only want to check every cell once, so
			// increase the index if we are still in the same cell
			if (idx == idxo && idy == idyo && idz == idzo)
				continue;
			else
			{
				idxo = idx;
				idyo = idy, idzo = idz;
			}

			lz->getNDTCellAt(idx, idy, idz, ptCell);

			if (ptCell != NULL)
			{
				double l2target = 0;

				if (ptCell->hasGaussian_)
				{
					Eigen::Vector3d out, pend, vpt;

					pend << endpoint.x, endpoint.y, endpoint.z; ///< endpoint

					double lik = ptCell->computeMaximumLikelihoodAlongLine(po, pt, out);
					double dist = (origin - out).norm();
					if (dist > l) continue; ///< don't accept points further than the measurement

					l2target = (out - pend).norm(); ///<distance to endpoint
															  //double thr =exp(-0.5*(l2target*l2target)/(sensor_noise*sensor_noise)); ///This is the probability of max lik point being endpoint
															  //ptCell->updateEmpty(lik*(1-thr),l2target);

					double sigma_dist = 0.5 * (dist / 30.0); ///test for distance based sensor noise
					double snoise = sigma_dist + sensor_noise;
					double thr = exp(-0.5*(l2target*l2target) / (snoise*snoise)); ///This is the probability of max lik point being endpoint
					lik *= (1.0 - thr);
					lik = 0.2*lik + 0.5; ///Evidence value for empty - alpha * p(x);
					double logoddlik = log((1.0 - lik) / (lik));
					ptCell->updateEmpty(logoddlik, l2target);
				}
				else
					ptCell->updateEmpty(-0.1, l2target); ///The cell does not have gaussian, so we mark that we saw it empty...
			}
			else
				index_->addPoint(pt); ///Add fake point to initialize!
		}

		if (updatePositive) 
			index_->addPoint(endpoint);

		isFirstLoad_ = false;

		return retval;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/** Helper function, computes the  NDTCells
	*/
	// 生成NDT单元
	void NDTMap::computeNDTCells(int cellupdatemode, unsigned int maxnumpoints, float occupancy_limit, Eigen::Vector3d origin, double sensor_noise)
	{
		conflictPoints.clear();

		 std::set<NDTCell*>::iterator it;
		for (it = update_set.begin(); it != update_set.end(); it++)
		{
			NDTCell *cell = *it;
			if (cell != NULL)
			{
				cell->computeGaussian(cellupdatemode, maxnumpoints, occupancy_limit, origin, sensor_noise);
				
				// Process the conflict points
				if (cell->points_.size() > 0)
				{
					for (unsigned int i = 0; i < cell->points_.size(); i++)
						conflictPoints.push_back(cell->points_[i]);

					cell->points_.clear();
				}
			}
		}

		update_set.clear();
	}

	//
	//   从文件中装入NDT图。
	//
	bool NDTMap::LoadFromFile(const char* filename)
	{
		FILE* fp = fopen(filename, "rb");
		if (fp == NULL)
			return false;

		// 装入NDT图
		LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
		if (gr == NULL || !gr->LoadFromFile(fp))
			return false;

		// 设置单元类型
		NDTCell *ptCell = new NDTCell();
		index_->setCellType(ptCell);
		delete ptCell;

		isFirstLoad_ = false;
		return true;
	}

	//
	//   将NDT图保存到文件。
	//
	bool NDTMap::SaveToFile(const char* filename)
	{
		FILE * fp = fopen(filename, "wb");
		if (fp == NULL)
			return false;

		// 以LazyGrid方式保存文件
		LazyGrid *ind = dynamic_cast<LazyGrid*>(index_);
		if (ind == NULL)
			return false;

		bool ok = ind->SaveToFile(fp);
		fclose(fp);

		return ok;
	}

	//computes the *negative log likelihood* of a single observation
	double NDTMap::getLikelihoodForPoint(pcl::PointXYZ pt)
	{
		//assert(false);
		double uniform = 0.00100;
		NDTCell* ndCell = NULL;

		LazyGrid *gr = dynamic_cast<LazyGrid*>(index_);
		if (gr == NULL)
		{
			//cout<<"bad index - getLikelihoodForPoint\n";
			return uniform;
		}
		ndCell = gr->getClosestNDTCell(pt);

		if (ndCell == NULL) return uniform;

		double prob = ndCell->getLikelihood(pt);
		prob = (prob<0) ? 0 : prob; //uniform!! TSV
		return prob;
	}


	std::vector<NDTCell*> NDTMap::getCellsForPoint(const pcl::PointXYZ pt, int n_neigh, bool checkForGaussian) const
	{
		std::vector<NDTCell*> cells;

		LazyGrid *gr = dynamic_cast<LazyGrid*>(index_);
		if (gr == NULL)
			return cells;

		cells = gr->getClosestNDTCells(pt, n_neigh, checkForGaussian);
		return cells;
	}

	bool NDTMap::getCellForPoint(const pcl::PointXYZ &pt, NDTCell* &out_cell, bool checkForGaussian) const
	{
		out_cell = NULL;
		LazyGrid *gr = dynamic_cast<LazyGrid*>(index_);
		if (gr != NULL)
		{
			out_cell = gr->getClosestNDTCell(pt, checkForGaussian);
			return true;
		}

		return false;
	}

	/**
	* Returns a transformed NDT as a vector of NDT cells
	*/
	// 将经过变换的NDT图转为一组NDT单元的向量后返回。
	VectNDTCells NDTMap::pseudoTransformNDT(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> T) const
	{
		VectNDTCells ret;

		 SpatialIndex::CellVectorConstItr it;
		for (it = index_->begin(); it != index_->end(); it++)
		{
			NDTCell *cell = (*it);
			if (cell != NULL && cell->hasGaussian_)
			{
				Eigen::Vector3d mean = cell->getMean();
				Eigen::Matrix3d cov = cell->getCov();
				mean = T * mean;

				// NOTE: The rotation of the covariance fixed by Jari 6.11.2012
				cov = T.rotation() * cov * T.rotation().transpose();
				NDTCell* nd = (NDTCell*)cell->clone();
				nd->setMean(mean);
				nd->setCov(cov);
				ret.push_back(nd);
			}
		}
		return ret;
	}

	std::vector<perception_oru::NDTCell*> NDTMap::getAllCells() const
	{
		std::vector<NDTCell*> ret;

		 SpatialIndex::CellVectorItr it;
		for (it = index_->begin(); it != index_->end(); it++)
		{
			NDTCell *cell = (*it);
			if (cell != NULL && cell->hasGaussian_)
			{
				NDTCell* nd = cell->copy();
				ret.push_back(nd);
			}
		}

		return ret;
	}

	std::vector<perception_oru::NDTCell*> NDTMap::getAllInitializedCells() const
	{
		std::vector<NDTCell*> ret;
		 SpatialIndex::CellVectorItr it = index_->begin();
		while (it != index_->end())
		{
			NDTCell* nd = (*it)->copy();
			ret.push_back(nd);
			it++;
		}
		return ret;
	}

	int NDTMap::numberOfActiveCells() const
	{
		if (index_ == NULL)
			return 0;
		
		int count = 0;
		 SpatialIndex::CellVectorItr it;
		for (it = index_->begin(); it != index_->end(); it++)
			if ((*it)->hasGaussian_)
				count++;

		return count;
	}

	NDTCell* NDTMap::getCellAtID(int x, int y, int z) const 
	{
		NDTCell* cell;
		LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
		lz->getCellAt(x, y, z, cell);
		return cell;
	}

	double NDTMap::getmapsizez()
	{
		LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
		double z;
		gr->getGridSizeInZMeters(z);
		return z;
	}

	// 取得单元的数量
	int NDTMap::GetCellCount()
	{
		LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
		return lz->size();
	}

	// 取得指定序号的单元的指针
	NDTCell* NDTMap::GetCell(int nIdx)
	{
		LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
		return lz->getCell(nIdx);
	}

	// 取得指定序号的单元中心点的位置
	bool NDTMap::GetCellPnt(int nIdx, CPnt& pt)
	{
		Eigen::Vector3d mean;  /// Mean of the normal distribution
		mean = GetCell(nIdx)->getMean();
		CPnt pt1(mean(0), mean(1));
		pt = pt1;
		return true;
	}

	// 删除指定序号的单元
	bool NDTMap::DeleteCell(int nCellIdx)
	{
		LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
		lz->deleteCell(nCellIdx);
		return true;
//		index_;
	}

	// 清除所有单元的匹配标志字
	void NDTMap::ClearMatchStatus()
	{
		if (index_ == NULL) 
			return;

		for (SpatialIndex::CellVectorItr it = index_->begin(); it != index_->end(); it++)
			(*it)->matchStatus = 0;
	}

	// 对全图进行坐标变换
	void NDTMap::Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr)
	{
		SpatialIndex::CellVectorConstItr it;
		for (it = index_->begin(); it != index_->end(); it++)
		{
			NDTCell *cell = (*it);
			if (cell != NULL)
				cell->Transform(tr);
		}
	}

	void NDTMap::Transform(double x, double y, double ang)
	{
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> t;
		t = Eigen::Translation<double, 3>(x, y, 0) *
			Eigen::AngleAxis<double>(ang, Eigen::Vector3d::UnitZ());
		Transform(t);
	}

#ifdef _MSC_VER
	//   判断一个给定的点是否“触碰”到图中的某个单元
	//   返回值：
	//      -1   - 未触碰到
	//      其它 - 触碰单元的序号

	int NDTMap::PointHitCell(const CPnt& pt, float fDistGate)
	{
		int i = 0;
		 SpatialIndex::CellVectorConstItr it;
		for (it = index_->begin(); it != index_->end(); it++)
		{
			NDTCell *cell = (*it);
			if (cell != NULL && cell->PointHit(pt, fDistGate) == 1)
				return i;       //  触碰到，返回其序号

			i++;
		}

		// 未触碰到任何单元，返回-1
		return -1;
	}

	void NDTMap::Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder)
	{
		SpatialIndex::CellVectorConstItr it;
		for (it = index_->begin(); it != index_->end(); it++)
		{
			NDTCell *cell = (*it);
			if (cell != NULL)
				cell->Plot(ScrnRef, pDc, clrCellBorder, clrCellFill);
		}
	}
#endif
}
