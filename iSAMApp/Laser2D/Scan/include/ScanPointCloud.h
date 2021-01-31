#ifndef __CScanPointCloud
#define __CScanPointCloud

#include "ScrnRef.h"
#include "ScanPoint.h"
#include "TimeStamp.h"

///////////////////////////////////////////////////////////////////////////////
// “CScanPointCloud”类定义二维点云。

class CScanPointCloud : public CTimeStamp
{
public:
	int         m_nCount;                    // 扫描点数量
	CScanPoint* m_pPoints;                   // 指向扫描点数据缓冲区的指针

protected:
	// 将那些标记为“删除”的点真正删除
	void RemoveDeletedPoints();

public:
	// 根据指定的点数量生成对象(只分配空间)
	CScanPointCloud(int nNum);

	// 根据另外一个点云生成对象
	CScanPointCloud(const CScanPointCloud& Cloud);

	// 生成空的对象
	CScanPointCloud();

	~CScanPointCloud();

	// 取得点云对象的指针(主要用在派生类中)
	CScanPointCloud* GetScanPointCloudPointer() {return this;}

	// 清除点云数据
	void Clear();

	// 为点云数据分配空间
	bool Create(int nNum);

	// 向点集末尾添加一个点
	bool Add(const CScanPoint& sp);

	// 从点云中删除一个点
	bool Delete(int nIndex);

	// 重载“=”操作符
	void operator = (const CScanPointCloud& Cloud2);

	// 重载“+=”操作符
	void operator += (const CScanPointCloud& Cloud2);

	// 从文本文件中装入点云数据
	bool Load(FILE* file);

	// 将点云数据存入文本文件
	bool Save(FILE* file);

	// 从二进制文件中读取扫描数据
	bool LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount, int nFileVersion);

	// 将扫描数据保存到二进制文件
	bool SaveBinary(FILE* fp, int nFileVersion);

	// 根据迪卡尔坐标计算出所有点的极坐标
	void UpdatePolar();

	// 根据极坐标计算出所有点的迪卡尔坐标
	void UpdateCartisian();

	// 将全部点移动指定的距离
	void Move(float fDx, float fDy);

	// 将全部点绕指定的中心点进行旋转
	void Rotate(float centerX, float centerY, float angle);

	// 将整个点云进行坐标变换
	void Transform(float x, float y, float thita);

	// 将整个点云进行坐标变换(第二种形式)
	void Transform(const CPosture& pstOrigin);

	// 对整个点云进行坐标系变换
	virtual void Transform(const CFrame& frame);

	// 对整个点云进行坐标系逆变换
	virtual void InvTransform(const CFrame& frame);

	// 取得最左点X坐标
	float LeftMost();

	// 取得最上点Y坐标
	float TopMost();

	// 取得最右点X坐标
	float RightMost();

	// 取得最下点Y坐标
	float BottomMost();

	// 取得点云的X向宽度
	float Width();

	// 取得点云的Y向高度
	float Height();

	// 取得此点云所覆盖的最大区域
	CRectangle GetCoveringRect();

	// 核对点云是否包含指定的点
	int ContainPoint(const CPnt& pt, float fThreshHoldDist);

	// 以指定的点为中心，按指定的半径对所有点进行过滤，只留下处于半径以内的点
	void ReduceByRadius(const CPnt& ptCenter, float dRadius);

	// 以指定的方向角为中心，指定的角度范围对所有点进行过滤，只留下有效角度内的点
	void ReduceByAngle(CPosture& pst, float fViewAngle);

	// 依据指定的姿态，对点云根据扫描角进行排序
	void SortByAngle();

	// 针对已角度排序的点云，删除那些被遮挡的点
	void RemoveHiddenPoints(const CPosture& pst, float fDistGate);

	// 将所有超出距离限的点移除
	void RemoveOutOfRangePoints();

	// 按指定的角分辨率和扫描距离对点云进行重新采样
	CScanPointCloud* ReSample(CPosture& pstScanner, float fStartAng, float fViewAngle, float fAngReso, float fMaxRange);

	// 根据给定的直线仿真生成点云
	bool CreateFromLine(CLine& ln, float fPointDist, float fNoise);

	// 根据给定的直线数组仿真生成点云
	bool CreateFromLineArray(int nCountLines, CLine* lines, float fPointDist, float fNoise);

	// 根据给定的圆仿真生成点云
	bool CreateFromCircle(CCircle& circle, float fPointDist, float fNoise);

	// 根据给定的圆的数组仿真生成点云
	bool CreateFromCircleArray(int nCountCircles, CCircle* circles, float fPointDist, float fNoise);

#ifdef _MSC_VER
	// 在调试器内显示点云数据
	void Dump();

	// 绘制点云
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1);
#endif
};

#endif
