#ifndef __CMultiSegLine
#define __CMultiSegLine

#include <vector>
#include "Geometry.h"
#include "Range.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////////
// 多段直线
class CMultiSegLine : public CLine
{
public:
	CRangeSet m_Ranges;

private:
	void CreateSingleLine();
	void ProjectToBaseLine(CLineBase& Line);

public:
	// Constructor form #1
	CMultiSegLine(const CPnt& ptStart, const CPnt& ptEnd);

	// Constructor form #2
	CMultiSegLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// Constructor form #3
	CMultiSegLine(const CLine& Line2);

	// The default constructor
	CMultiSegLine() {}

	// 取得指定的子段
	bool GetSegment(int i, CLine& Line) const;

	// 根据两点生成线段
	void Create(const CPnt& ptStart, const CPnt& ptEnd);

	// 根据一点和直线的倾斜角生成线段
	void Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// 从多条线段生成此多段线
	void Create(CLine* pLines, int nNum);

	// 从多段线生成此多段线
	void Create(const CMultiSegLine& MultiSegLine);

	// 判断一个给定的点是否“触碰”到该直线
	bool PointHit(const CPnt& pt, float fDistGate);

#ifdef _MFC_VER
	// 在屏幕上绘制此直线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#endif
};
#endif
