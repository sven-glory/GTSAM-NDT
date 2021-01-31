#ifndef __CPointFeature
#define __CPointFeature

#include "Geometry.h"
#include "Feature.h"

// 特征类型
#define GENERIC_POINT_FEATURE      0        // 一般特征
#define FLAT_REFLECTOR_FEATURE     1        // 平面反光板特征
#define CYLINDER_FEATURE           2        // 圆柱特征
#define CORNER_FEATURE             3        // 角点
#define SHORT_LINE_FEATURE         4        // 短直线特征
#define EDGE_FEATURE               5        // 边缘特征
///////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。

class CPointFeature : public CFeature
{
protected:
	int      m_nType;               // 点特征的类型
	CPoint2d m_ptCenter;            // 特征点位置

public:
	CPointFeature();

	// 设置点特征的类型
	void SetType(int nType) { m_nType = nType; }

	// 取得点特征的类型
	int GetType() { return m_nType; }

	// 取得点特征的中心位置
	CPoint2d& GetCenterPoint() { return m_ptCenter; }

	// 设置中心位置
	void SetCenterPoint(const CPoint2d& ptCenter) { m_ptCenter = ptCenter; }

	// 生成一个复本
	virtual CPointFeature* Duplicate();

	// 从文件中装入点特征的参数
	virtual bool Load(FILE* fp);

	// 将点特征的参数保存到文件中
	virtual bool Save(FILE* fp);

#ifdef _MSC_VER
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif
};
#endif
