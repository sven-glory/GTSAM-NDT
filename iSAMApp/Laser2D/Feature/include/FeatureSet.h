#ifndef __CFeatureSet
#define __CFeatureSet

#include "PointFeatureSet.h"
#include "FeatureCreationParam.h"
#include "Scan.h"

///////////////////////////////////////////////////////////////////////////////
//    定义移动机器人的“特征集合”。
class CFeatureSet : public CPointFeatureSet
{
public:
	CPosture         m_pstObserver;         // 观测器所在的姿态

public:
	CFeatureSet() : m_pstObserver(0, 0, 0) {}

	CPointFeatureSet& GetPointFeatureSet();

	// 设置观测器的姿态
	virtual void SetObserverPosture(const CPosture& pst) { m_pstObserver = pst; }

	// 清除数据
	void Clear();

	// 从文本文件中装入特征集合
	int LoadText(FILE* fp);

	// 将特征集合存入文本文件中
	int SaveText(FILE* fp);

	// 从二进制文件中装入特征集合
	int LoadBinary(FILE* fp);

	// 将特征集合存入二进制文件中
	int SaveBinary(FILE* fp);

	// 从原始扫描点云生成特征集合
	bool CreateFromRawScan(const CScan& Scan, CFeatureCreationParam& Param);

	// 通过对给定特征模型的采样来生成特征采样集合
	bool CreateFromFeatureMap(const CFeatureSet& FeatureMap, const CPosture& pstScanner);

	// 以给定的点为中心，以指定的范围为半径，截取出一个特征子集
	bool GetSubset(CPnt& ptCenter, float fRange, CFeatureSet& Subset);

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	// 绘制全局图
	void Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF clrPoint, bool bShowId = false);
#endif
};
#endif
