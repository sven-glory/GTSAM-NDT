#ifndef __CPointFeature
#define __CPointFeature

#include <stdio.h>
#include "Geometry.h"
#include "Feature.h"

#ifdef QT_VERSION
#include <QColor>
#endif

// 特征类型
#define GENERIC_POINT_FEATURE      0        // 一般特征
#define FLAT_REFLECTOR_FEATURE     1        // 平面反光板特征
#define ROUND_FEATURE              2        // 圆柱特征
#define CORNER_FEATURE             3        // 角点
#define SHORT_LINE_FEATURE         4        // 短直线特征
#define EDGE_FEATURE               5        // 边缘特征
#define IRREGULAR_PILLAR_FEATURE   6

// 同类点特征距离误差门限
#define FEATURE_REG_DISTANCE_WINDOW           1.0f

class CScan;

///////////////////////////////////////////////////////////////////////////////
//   定义“点特征”基类。

class CPointFeature : public CFeature, public CPnt
{
public:
	int m_nMatchId;            // 与之匹配的点特征的ID号
	int m_nStartPointId;       // 原始点云中与此特征对应的起始点ID
	int m_nEndPointId;         // 原始点云中与此特征对应的终止点ID
	int m_nPointCount;         // 扫描点数量

public:
	CPointFeature(float _x = 0, float _y = 0);

	// 设置中心位置
	void SetCenterPoint(const CPnt& ptCenter) { GetPntObject() = ptCenter;}

	// 生成一个复本
	virtual CPointFeature* Duplicate() const;

	// 判断该点特征是否被指定的扫描线照到，并返回扫描点坐标
	virtual bool HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist);

	// 针对给定的点云，在规定的角度范围内，检测点云中是否含有该点特征，并返回它的中心位置
	virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

	// 判断指定的入射光线是否可用
	virtual bool CheckInRay(CLine& lnRay) {return true;}

	// 从文本文件中装入点特征的参数
	virtual int LoadText(FILE* fp);

	// 将点特征的参数保存到文本文件中
	virtual int SaveText(FILE* fp);

	// 从二进制文件中装入点特征的参数
	virtual int LoadBinary(FILE* fp);

	// 将点特征的参数保存到二进制文件中
	virtual int SaveBinary(FILE* fp);

	// 返回点特征的物理半径(主要用于屏幕鼠标操作)
	virtual float GetPointRadius() { return 0.02f; }

#ifdef _MFC_VER
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize,
		int nLineWidth = 0);

	// 在屏幕上绘制此点特征的ID号
	virtual void PlotId(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor);

#elif defined QT_VERSION
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crSelected, int nPointSize,
		int nLineWidth = 0);

	// 在屏幕上绘制此点特征的ID号
	virtual void PlotId(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor);
#endif
};
#endif
