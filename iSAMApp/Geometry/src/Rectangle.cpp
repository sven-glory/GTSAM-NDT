//                          - RECTANGLE.CPP -
//
//   Implementation of class "CRectangle" - which defines the geometric concept
//   "Rectangle".
//
//   Author: Zhang Lei
//   Date:   2015. 4. 11
//

#include "stdafx.h"
#include <assert.h>
#include <math.h>
#include "Tools.h"
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   CRectangle: The constructor form #1.
//
CRectangle::CRectangle(const CPnt& ptLeftTop, const CPnt& ptRightBottom)
{
	// 检查输入参数的合法性
	if (ptLeftTop.x > ptRightBottom.x || ptLeftTop.y < ptRightBottom.y)
		assert(false);

	Create(ptLeftTop, ptRightBottom);
}

CRectangle::CRectangle(float fLeft, float fTop, float fRight, float fBottom)
{
	if (!Create(fLeft, fTop, fRight, fBottom))
		assert(false);
}

CRectangle::CRectangle()
{
	Clear();
}

//
//   生成矩形。
//
bool CRectangle::Create(const CPnt& ptLeftTop, const CPnt& ptRightBottom)
{
	// 检查输入参数的合法性
	if (ptLeftTop.x > ptRightBottom.x || ptLeftTop.y < ptRightBottom.y)
		return false;

	m_ptLeftTop = ptLeftTop;
	m_ptRightBottom = ptRightBottom;
	m_bInit = true;

	return true;
}

//
//   生成矩形。
//
bool CRectangle::Create(float fLeft, float fTop, float fRight, float fBottom)
{
	// 检查输入参数的合法性
	if (fLeft > fRight || fTop < fBottom)
		return false;

	m_ptLeftTop.Set(fLeft, fTop);
	m_ptRightBottom.Set(fRight, fBottom);
	m_bInit = true;

	return true;
}

//
//   清除区域。
//
void CRectangle::Clear()
{
	Create(0, 0, 0, 0);
	m_bInit = false;
}

//
//   取得中心点的位置。
//
CPnt CRectangle::GetCenterPoint() const
{
	CPnt ptCenter;
	ptCenter.x = (m_ptLeftTop.x + m_ptRightBottom.x) / 2;
	ptCenter.y = (m_ptLeftTop.y + m_ptRightBottom.y) / 2;

	return ptCenter;
}

//
//   判断一个点是否处于矩形以内。
//
bool CRectangle::Contain(float x, float y) const
{
	if (x >= m_ptLeftTop.x && x <= m_ptRightBottom.x &&
		y >= m_ptRightBottom.y && y <= m_ptLeftTop.y)
		return true;
	else
		return false;
}

//
//   判断一个点是否在此矩形内。
//
bool CRectangle::Contain(const CPnt& pt) const
{
	if (pt.x >= m_ptLeftTop.x && pt.x <= m_ptRightBottom.x &&
		 pt.y >= m_ptRightBottom.y && pt.y <= m_ptLeftTop.y)
		return true;
	else
		return false;
}

//
//   判断一线段是否处于矩形以内。
//
bool CRectangle::Contain(const CLine& ln) const
{
	if (Contain(ln.m_ptStart) && Contain(ln.m_ptEnd))
		return true;
	else
		return false;
}

//
//   判断另一个矩形是否处于该矩形以内。
//
bool CRectangle::Contain(const CRectangle& r) const
{
	if (Contain(r.m_ptLeftTop) && Contain(r.m_ptRightBottom))
		return true;
	else
		return false;
}

//
//   取得矩形区域的宽度。
//
float CRectangle::Width() const
{
	return m_ptRightBottom.x - m_ptLeftTop.x;
}

//
//   取得矩形区域的高度。
//
float CRectangle::Height() const
{
	return m_ptLeftTop.y - m_ptRightBottom.y;
}

//
//   调整矩形区域大小以便容纳给定的点。
//
void CRectangle::operator += (const CPnt& pt)
{
	if (!m_bInit)
	{
		m_ptLeftTop = m_ptRightBottom = pt;
		m_bInit = true;
		return;
	}

	if (pt.x < m_ptLeftTop.x)
		m_ptLeftTop.x = pt.x;

	if (pt.x > m_ptRightBottom.x)
		m_ptRightBottom.x = pt.x;

	if (pt.y < m_ptRightBottom.y)
		m_ptRightBottom.y = pt.y;

	if (pt.y > m_ptLeftTop.y)
		m_ptLeftTop.y = pt.y;
}

//
//   调整矩形区域大小以便容纳给定的直线。
//
void CRectangle::operator += (const CLine& line)
{
	*this += line.m_ptStart;
	*this += line.m_ptEnd;
}

//
//   调整矩形区域大小以便容纳给定的矩形区域。
//
void CRectangle::operator += (const CRectangle& rect)
{
	*this += rect.GetLeftTopPoint();
	*this += rect.GetRightBottomPoint();
}

//
//   从文件装入矩形数据。
//
bool CRectangle::Load(FILE* fp)
{
	if (fscanf(fp, "%f\t%f\t%f\t%f\n", &m_ptLeftTop.x, &m_ptLeftTop.y, &m_ptRightBottom.x, &m_ptRightBottom.y) != 4)
		return false;

	return true;
}

//
//   将矩形数据写入文件。
//
bool CRectangle::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\t%f\n", m_ptLeftTop.x, m_ptLeftTop.y, m_ptRightBottom.x, m_ptRightBottom.y);
	return true;
}

#ifdef _MSC_VER

//
//   在屏幕上绘制此矩形。
//
//void CRectangle::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
//{
//	CPnt ptRightTop(m_ptRightBottom.x, m_ptLeftTop.y);
//	CPnt ptLeftBottom(m_ptLeftTop.x, m_ptRightBottom.y);
//
//	CLine ln1(m_ptLeftTop, ptRightTop);
//	CLine ln2(m_ptLeftTop, ptLeftBottom);
//	CLine ln3(ptLeftBottom, m_ptRightBottom);
//	CLine ln4(ptRightTop, m_ptRightBottom);
//
//	ln1.Draw(ScrnRef, pDC, crColor, nWidth);
//	ln2.Draw(ScrnRef, pDC, crColor, nWidth);
//	ln3.Draw(ScrnRef, pDC, crColor, nWidth);
//	ln4.Draw(ScrnRef, pDC, crColor, nWidth);
//}
#endif
