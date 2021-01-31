#ifndef __CLineFeature
#define __CLineFeature

#include "MultiSegLine.h"
#include "Feature.h"

// 特征类型
#define GENERIC_LINE_FEATURE             0         // 一般直线特征
#define SINGLE_SIDED_LINE_FEATURE        1         // 单侧直线特征

///////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。

class CLineFeature : public CFeature, public CMultiSegLine
{
public:
	long  m_lStart;
	long  m_lEnd;              // point numbers in scan
	float m_fSigma2;           // 匹配误差

	int   m_nWhichSideToUse;   // 使用直线的哪一侧(1-前面;2-后面;3-两面)
	bool  m_bSelected;         // 标明此线段是否被选中(用于屏幕编辑处理)

	bool     m_bCreateRef;     // 是否已生成参考点
	CPnt m_ptRef;          // 参考点的位置
	CPnt m_ptProjectFoot;  // 投影点的位置

public:
	CLineFeature(CPnt& ptStart, CPnt& ptEnd);
	CLineFeature();

	// 生成一个复本
	virtual CLineFeature* Duplicate() const;

	CLineFeature& GetLineFeatureObject() {return *this; }
	
	// 从文本文件读入数据
	virtual int LoadText(FILE* file);

	// 将数据写入到文本文件
	virtual int SaveText(FILE* file);

	// 从二进制文件读入数据
	virtual int LoadBinary(FILE* file);

	// 将数据写入到二进制文件
	virtual int SaveBinary(FILE* file);

	// 生成线段
	void Create(CPnt& ptStart, CPnt& ptEnd);

	// 生成线段
	void Create(CLine& ln);

	// 根据多段线生成特征
	void Create(CMultiSegLine& MultiSegLine);
	
	// 设置扫检测到该直线特征时的激光头姿态，以便计算有效的观测朝向
	void SetDetectPosture(const CPosture& pstDetect);

	// 判断本线段是否与另一线段有重叠区
	bool IsOverlapWith(CLineFeature& Feature2);

	// 将此多段线与另外一条多段线(如果共线的话)进行合并
	bool ColinearMerge(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines);
	bool ColinearMerge1(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines);

	// 将特征进行平移
	virtual void Move(float fX, float fY);

	// 将特征进行旋转
	virtual void Rotate(CAngle ang, CPnt ptCenter);

	// 选中/取消选中此线段
	void Select(bool bOn) {m_bSelected = bOn;}

	// 以pstScanner为观测姿态，判断本直线特征是否可与给定的另一直线特征进行配准
	bool RegisterWith(CLineFeature& another, CPosture& pstScanner, float fDistGate, float fAngGate);

	// 计算本直线特征到另一直线特征的变换代价
	float TransformCostTo(const CLineFeature& another) const;

	// 判断该直线特征能否与另外一个直线特征构成一个“角点”
	bool MakeCorner(const CLineFeature& another, float fMinAngle, float fMaxAngle, float fMaxGap, CPnt& ptCorner);

	// 进行坐标正变换
	virtual void Transform(const CFrame& frame);

	// 进行坐标逆变换
	virtual void InvTransform(const CFrame& frame);

#ifdef _MSC_VER
	// (在屏幕窗口上)测试指定的点是否在线段上
	bool HitTest(CScreenReference& ScrnRef, CPoint point);

	// 在屏幕上显示此直线特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
		int nSize, int nShowDisabled = DISABLED_FEATURE_UNDERTONE, bool bShowSuggested = true);
#endif
};
#endif
