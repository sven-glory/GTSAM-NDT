#pragma once

#include <ndt_map/spatial_index.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/lazy_grid.h>

#include <set>
#include <cstdlib>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sstream"
#include "string"
#include "iostream"

#ifdef _MSC_VER
class CDC;
class CScreenReference;
#endif

class CPnt;


namespace perception_oru
{
	// 结合给定的范围限制值，统计一个点云的重心点
	Eigen::Vector3d getCloudCentroid(const pcl::PointCloud<pcl::PointXYZ>& cloud, double range_limit);

	// 结合给定的范围限制值，计算给定点云的范围参数及重心点
	Eigen::Vector3d getCloudDimension(const pcl::PointCloud<pcl::PointXYZ>& cloud, double range_limit,
		double& maxDist, double& minZ, double& maxZ);

	/**
	*  \brief Implements an NDT based spatial index
	*  \author Jari Saarinen (jari.saarinen@aalto.fi) and Todor Stoyanov (todor.stoyanov@oru.se)
	*  \version 2.0
	*  \details This class contains an interface to a SpatialIndex (custom defined)
	* that contains NDT cells. Provides methods to create from a PointCloud.
	*
	* This class implements two approaches to NDT mapping -  "traditional" NDT approach, as well as
	* a novel NDT Occupancy Map approach.
	*
	* The "traditional" approach uses only the measurement points and a single
	* scan in order to construct the NDT map.
	*
	* The NDT-OM fuses incrementally measurement using Recursive Sample Covariance (RSC)
	* approach. It also models the occupancy and free space, it adapts to changes in the cell
	* etc.
	*
	* Having these two versions combined also means that not all features are working with both.
	* The known NDT-OM issues are
	* - Only Lazy-Grid spatial index is supported
	*
	* The old interface (e.g. used in registration) loadPointCloud(const pcl::PointCloud<PointT> &pc, double range_limit = -1);
	* works as before: it computes an NDT map using only the samples and without tracking occupancy.
	*
	* Since version 2.0 the ndt_map integrates also incremental update features. These are accessible through two methods:
	* 1) void addPointCloudSimple(const pcl::PointCloud<PointT> &pc,double maxz=100.0);
	* 2) void addPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<PointT> &pc, double classifierTh=0.06, double maxz = 100.0, double sensor_noise = 0.25);
	*
	* The first one only updates the measurement points and thus is faster, but does not model free space and does not tolerate dynamics
	* The second one uses ray tracing and a number of approaches to model the occupancy as well as adapts to the dynamics.
	*
	* In all cases the procedure to use the ndt_map is the following:
	* 1) Add the measurement (by above mentioned load or add methods)
	* 2) call computeNDTCells

	*/
	class NDTMap
	{
	protected:
		SpatialIndex *index_;
		//LazyGrid *index_;
		bool isFirstLoad_;
		float map_sizex;
		float map_sizey;
		float map_sizez;
		float centerx, centery, centerz;
		bool guess_size_;
		std::set<NDTCell*> update_set;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			pcl::PointCloud<pcl::PointXYZ> conflictPoints; // < points that were conflicting during update
	
	public:
		NDTMap()
		{
			index_ = NULL;
			guess_size_ = true;
			map_sizex = map_sizey = map_sizez = -1.0;
			centerx = centery = centerz = 0.0;
			guess_size_ = true;
			isFirstLoad_ = true;
		}

		/** default constructor. The SpatialIndex sent as a paramter
		*	is used as a factory every time that loadPointCloud is called.
		*	it can/should be deallocated outside the class after the destruction of the NDTMap
		*/
		NDTMap(SpatialIndex *idx, bool dealloc = false)
		{
			index_ = idx;

			//this is used to prevent memory de-allocation of the *si
			//si was allocated outside the NDT class and should be deallocated outside
			isFirstLoad_ = !dealloc;
			map_sizex = map_sizey = map_sizez = -1.0;
			centerx = centery = centerz = 0.0;
			guess_size_ = true;
		}

		// 拷贝构造函数
		NDTMap(const NDTMap& other)
		{
			if (other.index_ != NULL)
			{
				this->index_ = other.index_->copy();
				isFirstLoad_ = false;
			}
		}

		/**
		* Construct with given centroid and sizes
		* @param cenx, ceny, cenz; (x,y,z) of the center of the map
		* @param sizex, sizey, sizez: The size of the map in each respective direction
		* NOTE: Implementation only for the laze grid
		**/
		NDTMap(SpatialIndex *idx, float cenx, float ceny, float cenz, float sizex, float sizey, float sizez, bool dealloc = false)
		{
			if (idx == NULL)
			{
				fprintf(stderr, "Idx == NULL - abort()\n");
				exit(1);
			}
			index_ = idx;

			//this is used to prevent memory de-allocation of the *si
			//si was allocated outside the NDT class and should be deallocated outside
			isFirstLoad_ = !dealloc;//////////////////////////////////////////////////////////////////////////////false; Henrik - was false, but why?

			NDTCell *ptCell = new NDTCell();
			index_->setCellType(ptCell);
			delete ptCell;
			index_->setCenter(cenx, ceny, cenz);
			index_->setSize(sizex, sizey, sizez);
			map_sizex = sizex;
			map_sizey = sizey;
			map_sizez = sizez;
			centerx = cenx;
			centery = ceny;
			centerz = cenz;
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL)
			{
				fprintf(stderr, "Unfortunately This constructor works only with Lazygrid!\n");
				exit(1);
			}
			lz->initialize();
			guess_size_ = false;
		}

		virtual ~NDTMap()
		{
			if (index_ != NULL && !isFirstLoad_)
			{
				delete index_;
				index_ = NULL;
			}
		}

		/**
		* Initilize with known values - normally this is done automatically, but in some cases you want to
		* influence these - call only once and before calling any other function
		*/
		void initialize(double cenx, double ceny, double cenz, double sizex, double sizey, double sizez)
		{
			isFirstLoad_ = false;

			NDTCell *ptCell = new NDTCell();
			index_->setCellType(ptCell);
			delete ptCell;
			index_->setCenter(cenx, ceny, cenz);
			index_->setSize(sizex, sizey, sizez);
			map_sizex = sizex;
			map_sizey = sizey;
			map_sizez = sizez;
			centerx = cenx;
			centery = ceny;
			centerz = cenz;

			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL)
			{
				fprintf(stderr, "Unfortunately This constructor works only with Lazygrid!\n");
				exit(1);
			}
			//        lz->initializeAll();
			lz->initialize();
			guess_size_ = false;
		}

		/**
		* Set the map size in meters - Must be called before first addPointCloud call if
		* you want to set the size - otherwise it is automatically determined
		*/
		void setMapSize(float sx, float sy, float sz)
		{
			map_sizex = sx;
			map_sizey = sy;
			map_sizez = sz;
		}


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
		virtual bool addPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<pcl::PointXYZ> &pc, double classifierTh = 0.06,
			double maxz = 100.0, double sensor_noise = 0.25, double occupancy_limit = 255);

		/**
		* This interface updates only the end points into the map without raytracing
		*/
		bool addPointCloudSimple(const pcl::PointCloud<pcl::PointXYZ> &pc, double maxz = 100.0);


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
		virtual bool addPointCloudMeanUpdate(const Eigen::Vector3d &origin,
			const pcl::PointCloud<pcl::PointXYZ> &pc,
			const Eigen::Vector3d &localmapsize,
			unsigned int maxnumpoints = 1e9, float occupancy_limit = 255, double maxz = 100.0, double sensor_noise = 0.25);

		/**
		* Adds one measurement to the map using NDT-OM update step
		* @return true if an inconsistency was detected
		*/

		virtual bool addMeasurement(const Eigen::Vector3d &origin, pcl::PointXYZ endpoint, double classifierTh, double maxz, double sensor_noise);


		/**
		* Adds a sample mean and covariance to the map
		* @param &ucov The covariance matrix to be added
		* @param &umean The mean of the normal distribution
		* @param numpointsindistribution The number of points used in computation of the sample mean and covariance
		* @param r,g,b -- optional color parameters
		* @param maxnumpoints -- optional adaptation of the gaussians
		*/
		bool addDistributionToCell(const Eigen::Matrix3d &ucov, const Eigen::Vector3d &umean, unsigned int numpointsindistribution,
			float r = 0, float g = 0, float b = 0, unsigned int maxnumpoints = 1e9, float max_occupancy = 1024);


		/**
		* loadPointCloud - You can call this if you are only interested in dealing with one scan
		* without need for fusing several ones or representing empty space and occupancy
		*
		* Otherwise you should always call addPointCloud (or if you don't want occupancy then addPointCloudSimple)
		*
		* \param pc the PointCloud that is to be loaded
		* \note every subsequent call will destroy the previous map!
		*/
		virtual bool loadPointCloud(const pcl::PointCloud<pcl::PointXYZ> &pc, double range_limit = -1);
		/// each entry in the indices vector contains a set of indices to a NDC cell.

		/**
		* loadPointCloudCentroid - A special load function to enable the matching of centroids (create alligned maps)
		* This is more efficient than the standard, but needs also the origin and size as parameters
		* \param &pc the PointCloud that is to be loaded
		* \param &origin The desired origin of the map (will be fitted acording to old_centroid)
		* \param &old_centroid The centroid to which we want to align
		* \param &map_size The size of the new map
		* \param range_limit The maximum range value for measurements
		* \note every subsequent call will destroy the previous map!
		*/
		bool loadPointCloudCentroid(const pcl::PointCloud<pcl::PointXYZ> &pc, const Eigen::Vector3d &origin, const Eigen::Vector3d &old_centroid, const Eigen::Vector3d &map_size, double range_limit);

		// void loadDepthImage(const cv::Mat& depthImage, DepthCamera<pcl::PointXYZ> &cameraParams);
		// pcl::PointCloud<pcl::PointXYZ> loadDepthImageFeatures(const cv::Mat& depthImage, std::vector<cv::KeyPoint> &keypoints,
		//                                                       size_t &supportSize, double maxVar, DepthCamera<pcl::PointXYZ> &cameraParams, bool estimateParamsDI=false, bool nonMean = false);

		/**
		* Computes the NDT-cells after a measurement has been added
		* @param cellupdatemode Defines the update mode (default CELL_UPDATE_MODE_SAMPLE_VARIANCE)
		* @param maxnumpoints Defines the forgetting factor (default 100000) the smaller the value the faster the adaptation
		*/
		virtual void computeNDTCells(int cellupdatemode = CELL_UPDATE_MODE_SAMPLE_VARIANCE, unsigned int maxnumpoints = 1e9, float occupancy_limit = 255, Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0), double sensor_noise = 0.1);

		// 从文件中装入NDT图
		bool LoadFromFile(const char* filename);

		// 将NDT图保存到文件
		bool SaveToFile(const char* filename);

		inline SpatialIndex* getMyIndex() const
		{
			return index_;
		}

		//computes the likelihood of a single observation
		virtual double getLikelihoodForPoint(pcl::PointXYZ pt);

		///Get the cell for which the point fall into (not the closest cell)
		virtual bool getCellAtPoint(const pcl::PointXYZ &refPoint, NDTCell *&cell);
		virtual NDTCell* getCellAtAllocate(const pcl::PointXYZ &refPoint);

		/**
		* returns the closest cell to refPoint
		* Does not work with NDT-OM
		*/
		virtual bool getCellForPoint(const pcl::PointXYZ &refPoint, NDTCell *&cell, bool checkForGaussian = true) const;
		/**
		* Returns all the cells within radius
		* Does not work with NDT-OM
		*/
		virtual std::vector<NDTCell*> getCellsForPoint(const pcl::PointXYZ pt, int n_neighbours, bool checkForGaussian = true) const;

		/**
		* Returns a transformed NDT as a vector of NDT cells
		*/
		// 将经过变换的NDT图转为一组NDT单元的向量后返回
		virtual /*std::vector<NDTCell*>*/VectNDTCells pseudoTransformNDT(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> T) const;

		virtual std::vector<perception_oru::NDTCell*> getAllCells() const;

		/**
		* Returns all cells that have been initialized (including ones that do not contain gaussian at the moment).
		* This is useful if you want to use the empty cells or dynamic cells
		*/
		virtual std::vector<perception_oru::NDTCell*> getAllInitializedCells() const;

//		int numberOfActiveCells();
		int numberOfActiveCells() const;

		virtual bool getCentroid(double &cx, double &cy, double &cz)
		{
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL) return false;
			lz->getCenter(cx, cy, cz);
			return true;
		}
		bool getGridSize(int &cx, int &cy, int &cz)
		{
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL) return false;
			lz->getGridSize(cx, cy, cz);
			return true;
		}

		bool getGridSizeInMeters(double &cx, double &cy, double &cz)
		{
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL) return false;
			lz->getGridSizeInMeters(cx, cy, cz);
			return true;
		}

		bool getGridSizeInMeters(double &cx, double &cy, double &cz) const
		{
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL) return false;
			lz->getGridSizeInMeters(cx, cy, cz);
			return true;
		}

		bool getCellSizeInMeters(double &cx, double &cy, double &cz)
		{
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL) return false;
			lz->getCellSize(cx, cy, cz);
			return true;
		}

		bool getCellSizeInMeters(double &cx, double &cy, double &cz) const
		{
			LazyGrid *lz = dynamic_cast<LazyGrid*>(index_);
			if (lz == NULL) return false;
			lz->getCellSize(cx, cy, cz);
			return true;
		}

		/**
		* \param guess_size try to guess the size based on point cloud. Otherwise use pre-set map size
		*/
		void guessSize(float cenx, float ceny, float cenz, float sizex, float sizey, float sizez) 
		{
			guess_size_ = true;
			centerx = cenx;
			centery = ceny;
			centerz = cenz;
			map_sizex = sizex;
			map_sizey = sizey;
			map_sizez = sizez;
		}

		/**
		* Computes a maximum likelihood depth from the map, given a position and a view vector
		*/

		NDTCell* getCellAtID(int x, int y, int z) const;
		double getmapsizez();

		// 取得单元的数量
		int GetCellCount();

		// 取得指定序号的单元的指针
		NDTCell* GetCell(int nIdx);

		// 取得指定序号的单元中心点的位置
		bool GetCellPnt(int nIdx, CPnt& pt);

		// 删除指定序号的单元
		bool DeleteCell(int nCellIdx);

		// 清除所有单元的匹配标志字
		void ClearMatchStatus();

		// 对全图进行坐标变换
		void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);

		// 对全图进行坐标变换
		void Transform(double x, double y, double ang);

	public:
#ifdef _MSC_VER
		//   判断一个给定的点是否“触碰”到图中的某个单元
		int PointHitCell(const CPnt& pt, float fDistGate);

		void Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder);
#endif
	};

} // end namespace
