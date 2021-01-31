#pragma once

#include "Geometry.h"

#ifdef _MFC_VER
class CScreenReference;
#endif

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CEllipse".
class DllExport CEllipse
{
public:
	CPnt m_ptCenter;        // 椭圆中心
	float m_fHalfMajorAxis; // 长半轴长度
	float m_fHalfMinorAxis; // 短半轴长度
	CAngle m_angSlant;      // 倾斜角

public:
	// The constructor
	CEllipse(const CPnt& ptCenter, float fHalfMajorAxis, float fHalfMiorAxis, const CAngle& angSlant);

	// Default constructor
	CEllipse() {}

	// 生成此椭圆圆
	void Create(const CPnt& ptCenter, float fHalfMajorAxis, float fHalfMiorAxis, const CAngle& angSlant);

	// 判断一个点是否处于椭圆以内
	bool Contain(const CPnt& pt) const;

#ifdef _MFC_VER
	// 在屏幕上绘制此圆
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crLineColor, int nLineWidth, 
		COLORREF crFillColor = 0, bool bFill = false);
#endif
};
