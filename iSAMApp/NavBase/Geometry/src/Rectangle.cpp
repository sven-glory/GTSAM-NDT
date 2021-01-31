//                          - RECTANGLE.CPP -
//
//   Implementation of class "CRectangle" - which defines the geometric concept
//   "Rectangle".
//
//   Author: Zhang Lei
//   Date:   2015. 4. 11
//

#include "stdafx.h"
#include <math.h>
#include "Tools.h"
#include "Geometry.h"
#include "ScrnRef.h"

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   CRectangle: The constructor form #1.
//
CRectangle:CRectangle(CPoint2d& ptLeftTop, CPoint2d& ptRightBottom)
{
	m_ptLeftTop = ptLeftTop;
	m_ptRightBottom = ptRightBottom;
}


//
//   判断一个点是否在此矩形内。
//
BOOL CRectangle::ContainPoint(CPoint2d& pt)
{
	if (p
}

//
//   在屏幕上绘制此矩形。
//
void CRectangle::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize)
{
}
