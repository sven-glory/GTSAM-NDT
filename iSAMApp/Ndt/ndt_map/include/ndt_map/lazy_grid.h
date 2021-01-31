#pragma once

#include <stdio.h>
#include <ndt_map/spatial_index.h>
#include <ndt_map/ndt_cell.h>

#define DEFAULT_MAP_SIZE_X               500               // 地图缺省宽度为200m
#define DEFAULT_MAP_SIZE_Y               500               // 地图缺省高度为200m
#define DEFAULT_MAP_RESO                 0.2               // 地图缺省分辨率0.2m

namespace perception_oru
{
	/** \brief A spatial index represented as a grid map
	\details A grid map with delayed allocation of cells.
	*/
	class LazyGrid : public SpatialIndex
	{
	protected:
		bool initialized;
		NDTCell ****dataArray;       // NDT单元缓冲区首指针
		NDTCell *protoType;          // 样板NDT单元，用来照此样板生成新单元
		std::vector<NDTCell*> activeCells;

		bool centerIsSet;            // 区域的中心点是否已设置
		bool sizeIsSet;              // 区域的范围是否已设置
		double sizeXmeters;          // 区域的X方向尺寸(m)
		double sizeYmeters;          // 区域的Y方向尺寸(m)
		double sizeZmeters;          // 区域的Z方向尺寸(m)
		double cellSizeX;            // NDT单元X方向尺寸
		double cellSizeY;            // NDT单元Y方向尺寸
		double cellSizeZ;            // NDT单元Z方向尺寸
		double centerX;              // 区域中心点的X坐标
		double centerY;              // 区域中心点的Y坐标
		double centerZ;              // 区域中心点的Z坐标
		int cellsCountX;             // 区域X方向上含有NDT单元的数量
		int cellsCountY;             // 区域Y方向上含有NDT单元的数量
		int cellsCountZ;             // 区域Z方向上含有NDT单元的数量

	public:
		// 构建立方体栅格
		LazyGrid(double cellSize = DEFAULT_MAP_RESO);

		// 构建长方体栅格
		LazyGrid(double cellSizeX_, double cellSizeY_, double cellSizeZ_);

		// 根据另外一个对象构造
		LazyGrid(LazyGrid *another);

		// 根据完整参数进行构造
		LazyGrid(double sizeXmeters, double sizeYmeters, double sizeZmeters,
			double cellSizeX, double cellSizeY, double cellSizeZ,
			double _centerX, double _centerY, double _centerZ,
			NDTCell *cellPrototype);

		virtual ~LazyGrid();

		virtual NDTCell* getCellForPoint(const pcl::PointXYZ &point);
		virtual NDTCell* addPoint(const pcl::PointXYZ &point);

		//these two don't make much sense...
		///iterator through all cells in index, points at the begining
		virtual  SpatialIndex::CellVectorItr begin();
		virtual  SpatialIndex::CellVectorConstItr begin() const;
		///iterator through all cells in index, points at the end
		virtual  SpatialIndex::CellVectorItr end();
		virtual  SpatialIndex::CellVectorConstItr end() const;
		virtual int size();

		///clone - create an empty object with same type
		virtual SpatialIndex* clone() const;
		virtual SpatialIndex* copy() const;

		///method to return all cells within a certain radius from a point
		virtual void getNeighbors(const pcl::PointXYZ &point, double radius, std::vector<NDTCell*> &cells);

		///sets the cell factory type
		virtual void setCellType(NDTCell *type);

		virtual void setCenter(double cx, double cy, double cz);
		virtual void setSize(double sx, double sy, double sz);
		
		// 删除指定序号的单元
		bool deleteCell(int nIdx);
		
		// 取得指定序号的单元的指针
		NDTCell* getCell(int nIdx);

		virtual NDTCell* getClosestNDTCell(const pcl::PointXYZ &pt, bool checkForGaussian = true);
		virtual std::vector<NDTCell*> getClosestNDTCells(const pcl::PointXYZ &pt, int &n_neigh, bool checkForGaussian = true);

		virtual inline void getCellAt(int indX, int indY, int indZ, NDTCell* &cell)
		{
			if (indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0)
			{
				cell = dataArray[indX][indY][indZ];
			}
			else 
			{
				cell = NULL;
			}
		}

		virtual inline void getCellAt(const pcl::PointXYZ& pt, NDTCell* &cell) 
		{
			int indX, indY, indZ;
			this->getIndexForPoint(pt, indX, indY, indZ);
			this->getCellAt(indX, indY, indZ, cell);
		}


		///automatically allocate the cell if needed (checks that the indexes etc. are correct).
		NDTCell* getCellAtAllocate(const pcl::PointXYZ& pt);

		//FIXME: these two are now not needed anymore
		virtual inline void getNDTCellAt(int indX, int indY, int indZ, NDTCell* &cell) 
		{
			if (indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0) 
			{
				cell = (dataArray[indX][indY][indZ]);
			}
			else 
			{
				cell = NULL;
			}
		}

		virtual inline void getNDTCellAt(const pcl::PointXYZ& pt, NDTCell* &cell) 
		{
			int indX, indY, indZ;
			this->getIndexForPoint(pt, indX, indY, indZ);
			this->getNDTCellAt(indX, indY, indZ, cell);
		}

		void getCellSize(double &cx, double &cy, double &cz);
		void getGridSize(int &cx, int &cy, int &cz);
		void getGridSizeInMeters(double &cx, double &cy, double &cz);
		void getCenter(double &cx, double &cy, double &cz);
		void getGridSizeInZMeters(double &cz);
		virtual void getIndexForPoint(const pcl::PointXYZ& pt, int &idx, int &idy, int &idz);
		
		NDTCell* getProtoType()
		{
			return protoType;
		}

		virtual void initialize();
		virtual void initializeAll();

		NDTCell ****getDataArrayPtr()
		{
			return dataArray;
		}

		// 从文件中装入LazyGrid
		virtual bool LoadFromFile(FILE * fp);

		// 将LazyGrid保存到文件
		virtual bool SaveToFile(FILE* fp);

		bool traceLine(const Eigen::Vector3d &origin, const pcl::PointXYZ &endpoint, const Eigen::Vector3d &diff, const double& maxz, std::vector<NDTCell*> &cells);

		bool isInside(const pcl::PointXYZ& pt) 
		{
			int indX, indY, indZ;
			this->getIndexForPoint(pt, indX, indY, indZ);
			return(indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0);
		}

		virtual bool checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian = true);


	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
};

