#include <cstring>
#include <cstdio>
#include <ndt_map/lazy_grid.h>

namespace perception_oru
{
	// 构建立方体栅格
	LazyGrid::LazyGrid(double cellSize) : protoType(NULL)
	{
		initialized = false;
		centerIsSet = false;
		sizeIsSet = false;
		cellSizeX = cellSizeY = cellSizeZ = cellSize;
	}

	// 构建长方体栅格
	LazyGrid::LazyGrid(double cellSizeX_, double cellSizeY_, double cellSizeZ_)
	{
		initialized = false;
		centerIsSet = false;
		sizeIsSet = false;
		cellSizeX = cellSizeX_;
		cellSizeY = cellSizeY_;
		cellSizeZ = cellSizeZ_;
	}

	// 根据另外一个对象构造
	LazyGrid::LazyGrid(LazyGrid *another)
	{
		sizeXmeters = another->sizeXmeters;
		sizeYmeters = another->sizeYmeters;
		sizeZmeters = another->sizeZmeters;

		cellSizeX = another->cellSizeX;
		cellSizeY = another->cellSizeY;
		cellSizeZ = another->cellSizeZ;

		cellsCountX = abs(ceil(sizeXmeters / cellSizeX));
		cellsCountY = abs(ceil(sizeYmeters / cellSizeY));
		cellsCountZ = abs(ceil(sizeZmeters / cellSizeZ));

		centerX = another->centerX;
		centerY = another->centerY;
		centerZ = another->centerZ;

		protoType = another->protoType->clone();
		initialize();
	}

	// 根据完整参数进行构造
	LazyGrid::LazyGrid(double _sizeXmeters, double _sizeYmeters, double _sizeZmeters,
		double _cellSizeX, double _cellSizeY, double _cellSizeZ,
		double _centerX, double _centerY, double _centerZ,
		NDTCell *cellPrototype)
	{
		sizeXmeters = _sizeXmeters;
		sizeYmeters = _sizeYmeters;
		sizeZmeters = _sizeZmeters;

		cellSizeX = _cellSizeX;
		cellSizeY = _cellSizeY;
		cellSizeZ = _cellSizeZ;

		cellsCountX = abs(ceil(sizeXmeters / cellSizeX));
		cellsCountY = abs(ceil(sizeYmeters / cellSizeY));
		cellsCountZ = abs(ceil(sizeZmeters / cellSizeZ));

		centerX = _centerX;
		centerY = _centerY;
		centerZ = _centerZ;

		protoType = cellPrototype->clone();
		initialize();
	}

	LazyGrid::~LazyGrid()
	{
		if (initialized)
		{
			int cnt = 0;

			//go through all cells and delete the non-NULL ones
			for (unsigned int i = 0; i < activeCells.size(); ++i)
			{
				if (activeCells[i])
				{
					delete activeCells[i];
					cnt++;
				}
			}

			// 释放所有NDT单元
			for (int i = 0; i < cellsCountX; i++)
			{
				for (int j = 0; j < cellsCountY; j++)
					delete[] dataArray[i][j];

				delete[] dataArray[i];
			}

			delete[] dataArray;

			if (protoType != NULL)
				delete protoType;
		}
	}

	void LazyGrid::setCenter(double cx, double cy, double cz)
	{
		centerX = cx;
		centerY = cy;
		centerZ = cz;

		centerIsSet = true;
		if (sizeIsSet)
		{
			initialize();
		}
	}

	void LazyGrid::setSize(double sx, double sy, double sz)
	{
		sizeXmeters = sx;
		sizeYmeters = sy;
		sizeZmeters = sz;

		cellsCountX = abs(ceil(sizeXmeters / cellSizeX));
		cellsCountY = abs(ceil(sizeYmeters / cellSizeY));
		cellsCountZ = abs(ceil(sizeZmeters / cellSizeZ));

		sizeIsSet = true;
		if (centerIsSet)
		{
			initialize();
		}
	}

	void LazyGrid::initializeAll()
	{
		if (!initialized)
			this->initialize();

		int idcX, idcY, idcZ;
		pcl::PointXYZ center;
		center.x = centerX;
		center.y = centerY;
		center.z = centerZ;
		this->getIndexForPoint(center, idcX, idcY, idcZ);

		pcl::PointXYZ centerCell;
		for (int i = 0; i < cellsCountX; i++)
		{
			for (int j = 0; j < cellsCountY; j++)
			{
				for (int k = 0; k < cellsCountZ; k++)
				{
					dataArray[i][j][k] = new NDTCell();
					dataArray[i][j][k]->setDimensions(cellSizeX, cellSizeY, cellSizeZ);

					centerCell.x = centerX + (i - idcX)*cellSizeX;
					centerCell.y = centerY + (j - idcY)*cellSizeY;
					centerCell.z = centerZ + (k - idcZ)*cellSizeZ;

					dataArray[i][j][k]->setCenter(centerCell);
					activeCells.push_back(dataArray[i][j][k]);
				}
			}
		}
	}

	// 为LazyGrid分配NDT单元空间
	void LazyGrid::initialize()
	{
		if (initialized)
			return;

		dataArray = new NDTCell***[cellsCountX];
		for (int i = 0; i < cellsCountX; i++)
		{
			dataArray[i] = new NDTCell**[cellsCountY];
			for (int j = 0; j < cellsCountY; j++)
			{
				dataArray[i][j] = new NDTCell*[cellsCountZ];
				//set all cells to NULL
				memset(dataArray[i][j], 0, cellsCountZ * sizeof(NDTCell*));
			}
		}

		initialized = true;
	}

	//
	//   根据给定的空间点位置，取得指向对应的NDT单元的指针。
	//
	NDTCell* LazyGrid::getCellForPoint(const pcl::PointXYZ &point)
	{
		// 先取得该点所在单元在X、Y、Z方向上的索引值
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		// 验证索引值范围
		if (indX >= cellsCountX || indY >= cellsCountY || indZ >= cellsCountZ || indX < 0 || indY < 0 || indZ < 0)
			return NULL;

		if (!initialized || dataArray == NULL || dataArray[indX] == NULL || dataArray[indX][indY] == NULL)
			return NULL;

		return dataArray[indX][indY][indZ];
	}

	//
	//
	//
	NDTCell* LazyGrid::getCellAtAllocate(const pcl::PointXYZ &pt)
	{
		// 验证空间点的有效性
		pcl::PointXYZ point = pt;
		if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
			return NULL;

		// 取得对应于该点的NDT单元在X、Y、Z方向上的索引地址
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		// 验证索引值的合法性
		if (indX >= cellsCountX || indY >= cellsCountY || indZ >= cellsCountZ || indX < 0 || indY < 0 || indZ < 0)
			return NULL;

		// 单元地址必须是已经分配过了
		if (!initialized || dataArray == NULL || dataArray[indX] == NULL || dataArray[indX][indY] == NULL)
			return NULL;

		// 如果该NDT单元是空的，现在设置该单元并激活它
		if (dataArray[indX][indY][indZ] == NULL)
		{
			// 依样板生成新单元并设置其尺寸
			dataArray[indX][indY][indZ] = protoType->clone();
			dataArray[indX][indY][indZ]->setDimensions(cellSizeX, cellSizeY, cellSizeZ);

			pcl::PointXYZ mapCenter;
			mapCenter.x = centerX;
			mapCenter.y = centerY;
			mapCenter.z = centerZ;

			// 计算整个LazyGrid图的中心点的索引值
			int idcX, idcY, idcZ;
			this->getIndexForPoint(mapCenter, idcX, idcY, idcZ);
			
			// 计算并设置该新单元的中心点的坐标
			pcl::PointXYZ cellCenter;
			cellCenter.x = centerX + (indX - idcX) * cellSizeX;
			cellCenter.y = centerY + (indY - idcY) * cellSizeY;
			cellCenter.z = centerZ + (indZ - idcZ) * cellSizeZ;
			dataArray[indX][indY][indZ]->setCenter(cellCenter);

			// 记录此为一个活动有效的单元
			activeCells.push_back(dataArray[indX][indY][indZ]);
		}

		return dataArray[indX][indY][indZ];
	}

	//
	//   向LazyGrid中加入一个空间点。
	//
	NDTCell* LazyGrid::addPoint(const pcl::PointXYZ &point_c)
	{
		// 根据给定的点，取得对应的NDT单元
		NDTCell* cell = this->getCellAtAllocate(point_c);
		if (cell == NULL)
			return NULL;

		// 将该点记录到此NDT单元中去
		cell->addPoint(point_c);

		return cell;
	}

	 SpatialIndex::CellVectorItr LazyGrid::begin()
	{
		return activeCells.begin();
	}

	 SpatialIndex::CellVectorConstItr LazyGrid::begin() const
	{
		return activeCells.begin();
	}

	 SpatialIndex::CellVectorItr LazyGrid::end()
	{
		return activeCells.end();
	}

	 SpatialIndex::CellVectorConstItr LazyGrid::end() const
	{
		return activeCells.end();
	}

	int LazyGrid::size()
	{
		return activeCells.size();
	}

	SpatialIndex* LazyGrid::clone() const
	{
		return new LazyGrid(cellSizeX);
	}

	//
	//   副制本LazyGrid，得到一个副本。
	//
	SpatialIndex* LazyGrid::copy() const
	{
		// 为新LazyGrid分配空间
		LazyGrid *ret = new LazyGrid(cellSizeX);   // 必须是立方体的?

		// 仅副制所有活动有效的单元
		 std::vector<NDTCell*>::const_iterator it;
		for (it = activeCells.begin(); it != activeCells.end(); it++)
		{
			NDTCell* r = (*it);
			if (r != NULL)
			{
				// 依次将各活动单元中的原始点加入到新LazyGrid中
				for (unsigned int i = 0; i < r->points_.size(); i++)
					ret->addPoint(r->points_[i]);
			}
		}

		return ret;
	}

	//
	//   以给定的空间点为中心，radius为半径，寻找所有邻近的NDT单元。
	//
	void LazyGrid::getNeighbors(const pcl::PointXYZ &point, double radius, std::vector<NDTCell*> &cells)
	{
		// 取得该点所对应的索引值
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		// 如果索引值超界，则返回空表
		if (indX >= cellsCountX || indY >= cellsCountY || indZ >= cellsCountZ)
		{
			cells.clear();
			return;
		}

		// 搜索并收集邻近的单元
		for (int x = indX - radius / cellSizeX; x <= indX + radius / cellSizeX; x++)
		{
			if (x < 0 || x >= cellsCountX)
				continue;

			for (int y = indY - radius / cellSizeY; y <= indY + radius / cellSizeY; y++)
			{
				if (y < 0 || y >= cellsCountY) 
					continue;

				for (int z = indZ - radius / cellSizeZ; z <= indZ + radius / cellSizeZ; z++)
				{
					if (z < 0 || z >= cellsCountZ) 
						continue;

					if (dataArray[x][y][z] == NULL) 
						continue;

					cells.push_back(dataArray[x][y][z]);
				}
			}
		}
	}

	//
	//   根据给定的空间点的位置，计算对应的NDT单元的地址。
	//
	void LazyGrid::getIndexForPoint(const pcl::PointXYZ& point, int &indX, int &indY, int &indZ)
	{
		indX = floor((point.x - centerX) / cellSizeX + 0.5) + cellsCountX / 2.0;
		indY = floor((point.y - centerY) / cellSizeY + 0.5) + cellsCountY / 2.0;
		indZ = floor((point.z - centerZ) / cellSizeZ + 0.5) + cellsCountZ / 2.0;
	}

	//
	//   计算与给定点的邻近距离不超过n_neigh的所有单元，并返回这些单元的列表。
	//
	std::vector< NDTCell* > LazyGrid::getClosestNDTCells(const pcl::PointXYZ &point, int &n_neigh, bool checkForGaussian)
	{
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		std::vector<NDTCell*> cells;
		int indXn, indYn, indZn;

		//the strange thing for the indeces is for convenience of writing
		//basicly, we go through 2* the number of cells and use parity to
		//decide if we subtract or add. should work nicely
		int i = n_neigh; //how many cells on each side
		for (int x = 1; x < 2 * i + 2; x++)
		{
			indXn = (x % 2 == 0) ? indX + x / 2 : indX - x / 2;
			for (int y = 1; y < 2 * i + 2; y++)
			{
				indYn = (y % 2 == 0) ? indY + y / 2 : indY - y / 2;
				for (int z = 1; z < 2 * i + 2; z++)
				{
					indZn = (z % 2 == 0) ? indZ + z / 2 : indZ - z / 2;

					// 如果该单元中有NDT单元，则将该单元加入返回列表
					if (checkCellforNDT(indXn, indYn, indZn, checkForGaussian))
					{
						cells.push_back(dataArray[indXn][indYn][indZn]);
					}
				}
			}
		}

		return cells;
	}

	//
	//   计算与给定点最近的单元并返回它的指针。
	//
	NDTCell* LazyGrid::getClosestNDTCell(const pcl::PointXYZ &point, bool checkForGaussian)
	{
		int indXn, indYn, indZn;
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);
		NDTCell *ret = NULL;
		std::vector<NDTCell*> cells;

		if (!checkForGaussian)
		{
			//just give me whatever is in this cell
			if (checkCellforNDT(indX, indY, indZ, checkForGaussian))
			{
				ret = (dataArray[indX][indY][indZ]);
			}
			return ret;
		}

		int i = 1; //how many cells on each side

		//for(int i=1; i<= maxNumberOfCells; i++) {
		//the strange thing for the indeces is for convenience of writing
		//basicly, we go through 2* the number of cells and use parity to
		//decide if we subtract or add. should work nicely
		for (int x = 1; x < 2 * i + 2; x++)
		{
			indXn = (x % 2 == 0) ? indX + x / 2 : indX - x / 2;
			for (int y = 1; y < 2 * i + 2; y++)
			{
				indYn = (y % 2 == 0) ? indY + y / 2 : indY - y / 2;
				for (int z = 1; z < 2 * i + 2; z++)
				{
					indZn = (z % 2 == 0) ? indZ + z / 2 : indZ - z / 2;
					if (checkCellforNDT(indXn, indYn, indZn))
					{
						ret = (dataArray[indXn][indYn][indZn]);
						cells.push_back(ret);
					}
				}
			}
		}

		double minDist = INT_MAX;
		Eigen::Vector3d tmean;
		pcl::PointXYZ pt = point;
		for (unsigned int i = 0; i < cells.size(); i++)
		{
			tmean = cells[i]->getMean();
			tmean(0) -= pt.x;
			tmean(1) -= pt.y;
			tmean(2) -= pt.z;
			double d = tmean.norm();
			if (d < minDist)
			{
				minDist = d;
				ret = cells[i];
			}
		}
		cells.clear();
		return ret;
	}

	//
	//   核对指定的索引位置是否有相应的NDT单元。
	//   注意：该函数并不核对该单元是否是活动的。
	//
	bool LazyGrid::checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian)
	{
		if (indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0)
		{
			if (dataArray[indX][indY][indZ] != NULL)
			{
				if (dataArray[indX][indY][indZ]->hasGaussian_ || (!checkForGaussian))
					return true;
			}
		}
		return false;
	}

	//
	//   为LazyGrid设置NDT单元样板。
	//
	void LazyGrid::setCellType(NDTCell *type)
	{
		if (type != NULL)
		{
			// 如原已有样板，现在需要删除它
			if (protoType != NULL)
				delete protoType;

			// 分配空间并设置新的样板
			protoType = type->clone();
		}
	}

	void LazyGrid::getCellSize(double &cx, double &cy, double &cz)
	{
		cx = cellSizeX;
		cy = cellSizeY;
		cz = cellSizeZ;
	}

	void LazyGrid::getCenter(double &cx, double &cy, double &cz)
	{
		cx = centerX;
		cy = centerY;
		cz = centerZ;
	}

	void LazyGrid::getGridSize(int &cx, int &cy, int &cz)
	{
		cx = cellsCountX;
		cy = cellsCountY;
		cz = cellsCountZ;
	}

	void LazyGrid::getGridSizeInMeters(double &cx, double &cy, double &cz)
	{
		cx = sizeXmeters;
		cy = sizeYmeters;
		cz = sizeZmeters;
	}

	void LazyGrid::getGridSizeInZMeters(double &cz)
	{
		cz = sizeZmeters;
	}

	//
	// 从文件中装入LazyGrid。(注意：缺乏通用性！)
	//
	bool LazyGrid::LoadFromFile(FILE* fp)
	{
		centerIsSet = false;
		sizeIsSet = false;

		float f[3];
		// 读入单元尺寸(X/Y/Z，单位：米)
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;
		cellSizeX = f[0]; cellSizeY = f[1];	cellSizeZ = f[2];

		// 读入场地最大尺寸(X/Y/Z，单位：米)
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;
		setSize(f[0], f[1], f[2]);

		// 读入图的中心位置(X/Y/Z，单位：米)
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;
		setCenter(f[0], f[1], f[2]);

		int n;
		if (fread(&n, sizeof(int), 1, fp) != 1)
			return false;

		int indX, indY, indZ;
		pcl::PointXYZ centerCell;

		NDTCell prototype_(cellSizeX, cellSizeY, cellSizeZ);
		// load all cells
		while (1)
		{
			if (prototype_.loadFromJFF(fp) < 0)
				if (feof(fp))
					break;

			if (feof(fp))
				break;

			centerCell = prototype_.getCenter();

			this->getIndexForPoint(centerCell, indX, indY, indZ);

			if (indX < 0 || indX >= cellsCountX) 
				continue;
			if (indY < 0 || indY >= cellsCountY) 
				continue;
			if (indZ < 0 || indZ >= cellsCountZ) 
				continue;

			if (!initialized || dataArray == NULL || dataArray[indX] == NULL || dataArray[indX][indY] == NULL)
				return false;
			
			if (dataArray[indX][indY][indZ] != NULL)
			{
				NDTCell* ret = dataArray[indX][indY][indZ];

				ret->setDimensions(cellSizeX, cellSizeY, cellSizeZ);
				ret->setCenter(centerCell);
				ret->setMean(prototype_.getMean());
				ret->setCov(prototype_.getCov());
				ret->setN(prototype_.getN());
				ret->setOccupancy(prototype_.getOccupancy());
				ret->hasGaussian_ = prototype_.hasGaussian_;
			}
			else
			{
				//initialize cell
				dataArray[indX][indY][indZ] = prototype_.copy();
				activeCells.push_back(dataArray[indX][indY][indZ]);
			}
		}

		return true;
	}

	//
	//   将LazyGrid保存到文件。
	//	
	bool LazyGrid::SaveToFile(FILE* fp)
	{
		// 写入单元尺寸(X/Y/Z，单位：米)
		float f[3] = { cellSizeX, cellSizeY, cellSizeZ };
		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		// 写入场地最大尺寸(X/Y/Z，单位：米)
		f[0] = sizeXmeters; f[1] = sizeYmeters; f[2] = sizeZmeters;
		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		// 写入图的中心位置(X/Y/Z，单位：米)
		f[0] = centerX; f[1] = centerY; f[2] = centerZ;
		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		// 写入活动单元数量
		int n = size();
		if (fwrite(&n, sizeof(int), 1, fp) != 1)
			return false;

		// 依次写入各活动单元的数据
		for (SpatialIndex::CellVectorItr it = begin(); it != end(); it++)
		{
			if ((*it)->writeToJFF(fp) < 0)
				return false;
		}

		return true;
	}

	//
	//   跟踪扫描线，收集线上的所有单元，结果保存于cells中。
	//
	bool LazyGrid::traceLine(const Eigen::Vector3d &origin, const pcl::PointXYZ &endpoint, const Eigen::Vector3d &diff_,
		const double& maxz, std::vector<NDTCell*> &cells)
	{
		if (endpoint.z > maxz)
			return false;

		double min1 = std::min(cellSizeX, cellSizeY);
		double min2 = std::min(cellSizeZ, cellSizeY);
		double resolution = std::min(min1, min2); // Select the smallest resolution

		if (resolution < 0.01)
		{
			fprintf(stderr, "Resolution very very small (%lf) :( \n", resolution);
			return false;
		}

		double l = diff_.norm();
		int N = l / (resolution);

		NDTCell* ptCell = NULL;

		pcl::PointXYZ po;
		po.x = origin(0); 
		po.y = origin(1);
		po.z = origin(2);

		Eigen::Vector3d diff = diff_ / (float)N;

		pcl::PointXYZ pt;
		int idxo = 0, idyo = 0, idzo = 0;
		for (int i = 0; i < N - 2; i++)
		{
			pt.x = origin(0) + ((float)(i + 1)) * diff(0);
			pt.y = origin(1) + ((float)(i + 1)) * diff(1);
			pt.z = origin(2) + ((float)(i + 1)) * diff(2);

			int idx, idy, idz;
			idx = floor((pt.x - centerX) / cellSizeX + 0.5) + cellsCountX / 2.0;
			idy = floor((pt.y - centerY) / cellSizeY + 0.5) + cellsCountY / 2.0;
			idz = floor((pt.z - centerZ) / cellSizeZ + 0.5) + cellsCountZ / 2.0;

			// We only want to check every cell once, so
			// increase the index if we are still in the same cell
			if (idx == idxo && idy == idyo && idz == idzo)
			{
				continue;
			}
			else
			{
				idxo = idx;
				idyo = idy;
				idzo = idz;
			}

			if (idx < cellsCountX && idy < cellsCountY && idz < cellsCountZ && idx >= 0 && idy >= 0 && idz >= 0) 
			{
				ptCell = dataArray[idx][idy][idz];
				if (ptCell != NULL) 
					cells.push_back(ptCell);
				else 
					this->addPoint(pt); // Add fake point to initialize!
			}
		}
		return true;
	}

	// 删除指定序号的单元
	bool LazyGrid::deleteCell(int nIdx)
	{
		delete activeCells[nIdx];
		activeCells.erase(activeCells.begin() + nIdx);
		return true;
	}

	// 取得指定序号的单元的指针
	NDTCell* LazyGrid::getCell(int nIdx)
	{
		return activeCells[nIdx];
	}
} //end namespace
