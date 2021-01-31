#ifndef __CScanPointCloud
#define __CScanPointCloud

#include "ScrnRef.h"
#include <afxwin.h>

///////////////////////////////////////////////////////////////////////////////
// “CScanPoint”类定义了二维扫描点的概念。

class CScanPoint : public CPnt
{
public:
	short m_nWeight;     // 对应的原始点的数量，用来表示该点的“重量”
	short m_nLineID;     // 对应的直线段的编号(如果该点落在一条直线段上),否则-1
	float m_fScanAng;    // 此点对应于扫描器处的扫描角

public:
	CScanPoint()
	{
		m_nWeight = 0;
		m_nLineID = -1;
		m_fScanAng = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
// “CScanPointCloud”类定义二维点云。

class CScanPointCloud
{
public:
	int         m_nCount;                    // 扫描点数量
	CScanPoint* m_pPoints;                   // 指向扫描点数据缓冲区的指针

public:
	// 根据指定的点数量生成对象(只分配空间)
	CScanPointCloud(int nNum);

	// 根据另外一个点云生成对象
	CScanPointCloud(const CScanPointCloud& Cloud);

	// 生成空的对象
	CScanPointCloud();

	~CScanPointCloud();

	CScanPointCloud* GetScanPointCloudPointer() {return this;}

	// 为点云数据分配空间
	BOOL Create(int nNum);

	// converts raw range readings to scan, taking into account interlaced scan and
	// robot velocities.  If deinterlace is TRUE then only every interlace-th
	// scan point is generated. 
	BOOL CreateFromPolarRanges(int nNum, short interlace, bool deinterlace, 
											const USHORT *r, 
											/*const PoseGauss *robotPos, */
											float tvel, float rvel, const CPosture *relPose, 
											float startA, float endA, USHORT maxRange, 
											float scanTime);

	// 清除所有原来的数据
	void Clear();

	// 重载“=”操作符
	void operator = (const CScanPointCloud& Cloud2);

	// 重载“+=”操作符
	void operator += (const CScanPointCloud& Cloud2);

	// 将全部点移动指定的距离
	void Move(float fDx, float fDy);

	// 将全部点绕指定的中心点进行旋转
	void Rotate(float centerX, float centerY, float angle);

	// 计算点云可见面积(目前未使用)
	float AreaSize();

	// 计算扫描点云所占的扫描角
	float FieldOfView();

	// 计算从给定点到点云中各点的平均距离
	float AverageDistanceFrom(const CPnt& pt);

	// 计算从重心点到点云中各点的平均距离
	float AverageDistanceFromGravity();

	// 计算点云的重心差(即点云中距离重心最远和最近两个点之间的距离)
	float DynamicsGravity();

	// 返回点云的重心点
	CPnt CenterOfGravity();

	// 计算点云的环绕长度
	float TotalLength();

	float LeftMost();
	float TopMost();
	float RightMost();
	float BottomMost();
	float Width();
	float Height();

	// 以指定的点的为中心，按指定的半径对所有点进行过滤，只留下处于半径以内的点
	void Reduce(CPnt& ptCenter, float dRadius);

	int ClosestPoint(const CPnt& ptNew, float* fDist = NULL);

	void Add(int nIndex, int nCurCount, const CScanPoint& sp);

	// 从点云中删除一个点
	void Delete(int nIndex);

	// 核对点云是否包含指定的点
	int ContainPoint(const CPnt& pt, float fThreshHoldDist);

	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1);

	void Dump();
	void DebugFilter(float fMinX, float fMaxX, float fMinY, float fMaxY);

	BOOL LoadFromFile(FILE* file);

	BOOL SaveToFile(FILE* file);
};

/*
** Calculates a line in the form (n1, n2) * (x, y) + c = 0,
** so that the leastsquare sum of the corresponding scan points
** is minimized.
** sp is an array of scan points, num specifies the number of scan points,
** n1, n2 and c are the result values.
*/
BOOL RegressionLine(const CScanPoint *sp, long num,
    float *n1, float *n2, float *c);

/*
** Calculates variance of distance from a set of scan points 
** given in (sp, num) to a line given in (n1, n2, c).
*/
float LineDeviation(CScanPoint *sp, long num,
    float n1, float n2, float c);

#endif
