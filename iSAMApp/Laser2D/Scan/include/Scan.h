#ifndef __CScan
#define __CScan

#include <vector>
#include "ScanPointCloud.h"
#include "RawScanData.h"
#include "PostureGauss.h"
#include "ScrnRef.h"
#include "Frame.h"
#include "ScannerParam.h"

#ifdef QT_VERSION
#include <QColor>
#endif

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   “CScan”类实现了“二维激光扫描”的概念。

class CScan : public CScanPointCloud
{
public:
	CPostureGauss m_poseRelative;        // 激光头的相对姿态
	CPostureGauss m_poseScanner;         // 激光头的估算姿态
	float         m_fStartAng;           // 激光头启始扫描角(弧度)
	float         m_fEndAng;             // 激光头终止扫描角(弧度)

public:
	CScan();

	// 构造具有nNum个点的CScan对象
	CScan(int nNum);

	CScan(CScanPointCloud* pCloud);

	~CScan();

	// 重载“=”操作符
	void operator = (const CScan& Scan);

	// 清除所有数据
	void Clear();

	// 生成一个新的扫描
	bool Create(int nNum);

	// 根据给定点云生成一个新的扫描
	bool Create(CScanPointCloud* pCloud);

	void SortByAngle();
	void SortByAngle(float fViewAngle);

	// 移除那些被遮挡的点
	void RemoveHiddenPoints(float fDistGate);

	// 分配空间并复制当前扫描
	CScan *Duplicate();

	// 将当前扫描与另一个扫描集进行合并
	void Merge(CScan& scan2, bool bNewPointsOnly = true);

	// 将全部点旋转指定的姿态
	void Rotate(float angle);

	void RotatePos(float angle, float cx, float cy);

	// 按指定的角分辨率和扫描距离对点云进行重新采样
	CScan* ReSample(float fStartAng, float fViewAngle, float fAngReso, float fMaxRange);

	// 根据指定的扫描角范围，获取点云的一个子集
	CScan* GetSubset(float fStartAngle, float fEndAngle);

	// 根据给定的“强反光门限”值，标记哪些点是属于“强反光”
	void MarkReflectivePoints(int nReflectiveGateValue);

	// 启用新的激光扫描器有效范围
	void ApplyNewScannerAngles(const CRangeSet& AngleRange);

	// 启用关于扫描角度范围的约束
	void ApplyScanAngleRule(float fMinAngle, float fMaxAngle);

	// 启用关于扫描距离的约束
	void ApplyScanDistRule(float fMinDist, float fMaxDist);

	// 从文本文件中读取扫描数据
	bool Load(FILE* fp);

	// 将扫描数据保存到文本文件
	bool Save(FILE* fp);

	// 从二进制文件中读取扫描数据
	bool LoadBinary(FILE* fp, const CPosture& pstRobot, const CLaserScannerParam& Param, int nFileVersion);

	// 将扫描数据保存到二进制文件
	bool SaveBinary(FILE* fp, int nFileVersion);

	// 对整个点云进行坐标系变换
	virtual void Transform(const CFrame& frame);

	// 对整个点云进行坐标系逆变换
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crHighLightPoint,
		bool bShowScanner, int nPointSize = 1);
#elif defined QT_VERSION
	void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crHighLightPoint,
		bool bShowScanner, int nPointSize = 1);

#endif
};
#endif

