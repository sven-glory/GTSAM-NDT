#ifndef __CScan
#define __CScan

#include <vector>
#include "ScanPointCloud.h"
#include "matrix.h"
#include "posegauss.h"
#include "misc.h"
#include "ScrnRef.h"
#include "LineFeatureSet.h"
#include <afxwin.h>

using namespace std;

#define DEFAULT_SCAN_MAX_RANGE         30000
#define PROJECT_MAX_REDUCTION          1.0

class CEdgePoint : public CPnt
{
public:
	int m_nEdgeType;

public:
	CEdgePoint() 
	{
		m_nEdgeType = -1;
	}
};

class CCornerPointsPair
{
public:
	int nIndex1;
	int nIndex2;

public:
	CCornerPointsPair(int _nIndex1, int _nIndex2)
	{
		nIndex1 = _nIndex1;
		nIndex2 = _nIndex2;
	}

	bool operator == (const CCornerPointsPair& another)
	{
		if ((nIndex1 == another.nIndex1 && nIndex2 == another.nIndex2) ||
			 (nIndex1 == another.nIndex2 && nIndex2 == another.nIndex1))
			return true;
		else
			return false;
	}
};

///////////////////////////////////////////////////////////////////////////////
//   “CScan”类实现了“二维激光扫描”的概念。

class CScan : public CScanPointCloud
{
public:
	PoseGauss m_poseScanner;                    /* estimated robot position */
	CPosture  sc_ScannerPos;             /* scanner pos relative to robots pos system */
	float     m_fStartAng;                 /* scanner start scan angle position */
	float     m_fEndAng;                   /* scanner end scan angle position */
	CLineFeatureSet* m_pLineFeatures;
	CLineFeatureSet* m_pLineFeaturesMerged;
	vector<CPnt> m_ptCorners;
	vector<CEdgePoint> m_ptEdges;
	vector<CCornerPointsPair> m_CornerPairs;

	//CCriticalSection         m_crit;

public:
	void SetProperties();

public:
	CScan() {}

	// 构造具有nNum个点的CScan对象
	CScan(int nNum);

	~CScan();

	// 重载“=”操作符
	void operator = (const CScan& Scan);

	// 清除所有数据
	void Clear();

	// 生成一个新的扫描
	BOOL Create(int nNum);

	// 生成所有直线特征
	bool CreateLineFeatures();
	
	// 生成所有点特征
	bool CreatePointFeatures(/*CPosture& pstScanner*/);

	void SortByAngle();

	// 取得指定扫描头的绝对姿态
	void GetScannerAbsPos(CPosture& pos);

	// 计算两个扫描数据的扫描头之间的距离
	float ScannerDistance(CScan& scan2);

	// 分配空间并复制当前扫描
	CScan *Duplicate();

	// 将当前扫描与另一个扫描集进行合并
	void Merge(CScan& scan2, BOOL bNewPointsOnly = TRUE);

	// 将全部点移动指定的距离
	void Move(float dx, float dy);

	// 将全部点旋转指定的姿态
	void Rotate(float angle);

	void RotatePos(float angle, float cx, float cy);

	// converts raw range readings to scan, taking into account interlaced scan and
	// robot velocities.  If deinterlace is TRUE then only every interlace-th
   // scan point is generated. 
	CScan* CreateFromPolarRanges(short num, short interlace, bool deinterlace, 
		const USHORT *r, const PoseGauss *robotpos, float tvel, float rvel, 
		const CPosture *relpos, float start_a, float end_a, USHORT maxrange, 
		float scanTime);

	CScan* PolarRangesToScan(short num, const USHORT *r, const PoseGauss *robotPos, 
		const CPosture *relPose, float startA, float endA, USHORT maxRange, const int* m_nIntensityData);

	// 判断该扫描集是否包含指定的扫描点
	BOOL ContainScanPoint(const CScanPoint& sp, float fThreshHoldDist = 50);

	// 查看给定的点是否是直线集中某条直线的端点，并返回该端点
	bool VerifyEdgePoint(CPnt& pt);

	// 判断给定的点是否与某个角点距离过近
	bool PointTooCloseToSomeCorner(CPnt& pt);

	// 从文件中读取扫描数据
	BOOL LoadFromFile(FILE* fp);

	// 将扫描数据保存到文件
	BOOL SaveToFile(FILE* fp);

	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crColorLine, COLORREF crSelectedLine,
		BOOL bShowPoints, BOOL bShowLineScan, BOOL bShowScanner, int nPointSize = 1, bool bShowInWorldFrame = false);

	// 根据所提供的中心点和半径对该扫描进行缩减
//	void Reduce(CPoint2d& ptCenter, float dRadius);
};

/*
** Sorts scan points in increasing angle.
*/
void SortScanPoints(CScanPoint *sp, long num);

//CScan* NewSimulateScan(CPosture& poseScanner, float dViewAngle = PI*2, CPosture* pAssumeScannerPos = NULL);
CScan* NewSimulateScan(CPosture& poseScanner, float fStartAngle, float fEndAngle, CPosture* pAssumeScannerPos = NULL);

#endif

