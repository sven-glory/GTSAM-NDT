//                         - CIRCLE.CPP -
//
//   Implementation of class "CCircle", which depicts circle.
//
//   Author: Zhang Lei
//   Date:   2014. 9. 11
//

#include "stdafx.h"
#include <math.h>
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArc".

//
//   The constructor.
//
CCircle::CCircle(CPoint2d& ptCenter, float fRadius)
{
	Create(ptCenter, fRadius);
}

//
//   生成此圆。
//
void CCircle::Create(CPoint2d& ptCenter, float fRadius)
{
	m_ptCenter = ptCenter;
	m_fRadius = fRadius;
}

//
//   取得此圆与直线段的第一个交点。
//
BOOL CCircle::IntersectLineAt(CLine& Line, CPoint2d& ptNear, float& fDist)
{
	CPoint2d ptFoot;
	float fLambda;
	fDist = Line.DistanceToPoint(TRUE, m_ptCenter, &fLambda, &ptFoot);

	// 构建圆心到垂足点的直线ln1
	CLine ln1(ptFoot, m_ptCenter);

	// 如果垂足点就是圆心点
	if (ln1.Length() < 1E-8)
	{
		CLine ln(m_ptCenter, Line.m_ptStart);

		// 如果圆心与线段Line的起点重合
		if (ln.Length() < 1E-8)
		{
			CLine ln0(m_ptCenter, Line.m_ptEnd);
			if (ln0.Length() >= m_fRadius)
			{
				ptNear = ln0.TrajFun(m_fRadius);
				fDist = Line.m_ptStart.DistanceTo(ptNear);
				return TRUE;
			}
			else
				return FALSE;
		}

		// 如果圆心与线段Line的起点不重合
		else
		{
			CLine ln0(m_ptCenter, Line.m_ptStart);
			ptNear = ln0.TrajFun(m_fRadius);
			if (Line.ContainPoint(ptNear))
			{
				fDist = Line.m_ptStart.DistanceTo(ptNear);
				return TRUE;
			}
			else
				return FALSE;
		}
	}

	// 如果ln1的长度小于圆的半径，说明垂足点在圆内，进一步说明直线Line与圆有交点
	else if (ln1.Length() <= m_fRadius)
	{
		// 现在计算半玄长
		float fL = (float)sqrt(m_fRadius*m_fRadius - fDist*fDist);

		// 定位Line与圆的交点
		CLine ln2(ptFoot, Line.m_ptStart);
		if (ln2.Length() < 1E-8)
		{
			CLine ln4(ptFoot, Line.m_ptEnd);
			ptNear = ln4.TrajFun(fL);
			fDist = Line.m_ptStart.DistanceTo(ptNear);
			return TRUE;
		}
		else
		{
			ptNear = ln2.TrajFun(fL);

			// 下面判断ptNear点是否落入线段Line之内(因为它也可能落在线段Line的延长线上)
			CLine ln3(ptNear, Line.m_ptStart);
			if (ln3.Length() < Line.Length())
			{
				fDist = Line.m_ptStart.DistanceTo(ptNear);
				return TRUE;
			}
			else
				return FALSE;
		}
	}
	else
		return FALSE;
}

//
//   以此圆截取一条直线，并将截取到的新直线保存到Line中。
//
bool CCircle::CutLine(CLine& Line, CLine& NewLine)
{
	CPoint2d ptFoot, pt;
	float fLambda;
	float fDist = Line.DistanceToPoint(TRUE, m_ptCenter, &fLambda, &ptFoot);

	// 如果垂线长度超过半径，说明没有截取到直线
	if (fDist > m_fRadius)
		return false;
	
	// 计算半弦长
	float fL = sqrt(m_fRadius*m_fRadius - fDist*fDist);

	// 如果起点在圆内
	if (Contain(Line.m_ptStart))
	{
		// 终点也在圆内
		if (Contain(Line.m_ptEnd))
		{
			// Line无变化
			NewLine = Line;
			return true;
		}
		// 如果终点在圆外
		else
		{
			CLine ln(ptFoot, Line.m_ptEnd);
			pt = ln.TrajFun(fL);

			// 重新构造直线Line(仅终点有变化)
			NewLine.Create(Line.m_ptStart, pt);

			return true;
		}
	}

	// 如果起点在圆外，终点在圆内
	else if (Contain(Line.m_ptEnd))
	{
		CLine ln(ptFoot, Line.m_ptStart);

		// 计算半弦长
		float fL = sqrt(m_fRadius*m_fRadius - fDist*fDist);
		pt = ln.TrajFun(fL);

		// 重新构造直线Line(仅起点有变化)
		NewLine.Create(pt, Line.m_ptEnd);

		return true;
	}

	// 起点、终点均在圆外
	else
	{
		// 分别构造垂足点到Line起点、终点的连线
		CLine ln1(ptFoot, Line.m_ptStart);
		CLine ln2(ptFoot, Line.m_ptEnd);
		
		// 如果两线同向，说明Line的两个端点均在圆外，所以圆没有截取到直线Line
		if (ln1.SlantAngle() == ln2.SlantAngle())
			return false;

		// 两线反向，说明圆在直线Line上有两个截取点
		else
		{
			CPoint2d pt1 = ln1.TrajFun(fL);
			CPoint2d pt2 = ln2.TrajFun(fL);

			// Line的两个端点都变化了
			NewLine.Create(pt1, pt2);

			return true;
		}
	}
}

//
//   判断一个点是否在圆内(或圆上)。
//
bool CCircle::Contain(const CPoint2d& pt)
{
	return (m_ptCenter.DistanceTo(pt) <= m_fRadius);
}

#ifdef _MSC_VER

//
//   在屏幕上绘制此圆。
//
void CCircle::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
{
	CPen pen(PS_SOLID, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPoint2d ptLeftTop(m_ptCenter.x - m_fRadius, m_ptCenter.y + m_fRadius);
	CPoint2d ptRightBottom(m_ptCenter.x + m_fRadius, m_ptCenter.y - m_fRadius);

	CPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	CPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);

	CRect r(pnt1, pnt2);
	pDC->Arc(r, pnt1, pnt1);

	pDC->SelectObject(pOldPen);
}
#endif
