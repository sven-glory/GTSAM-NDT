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
//   ���ɴ�Բ��
//
void CCircle::Create(CPoint2d& ptCenter, float fRadius)
{
	m_ptCenter = ptCenter;
	m_fRadius = fRadius;
}

//
//   ȡ�ô�Բ��ֱ�߶εĵ�һ�����㡣
//
BOOL CCircle::IntersectLineAt(CLine& Line, CPoint2d& ptNear, float& fDist)
{
	CPoint2d ptFoot;
	float fLambda;
	fDist = Line.DistanceToPoint(TRUE, m_ptCenter, &fLambda, &ptFoot);

	// ����Բ�ĵ�������ֱ��ln1
	CLine ln1(ptFoot, m_ptCenter);

	// �����������Բ�ĵ�
	if (ln1.Length() < 1E-8)
	{
		CLine ln(m_ptCenter, Line.m_ptStart);

		// ���Բ�����߶�Line������غ�
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

		// ���Բ�����߶�Line����㲻�غ�
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

	// ���ln1�ĳ���С��Բ�İ뾶��˵���������Բ�ڣ���һ��˵��ֱ��Line��Բ�н���
	else if (ln1.Length() <= m_fRadius)
	{
		// ���ڼ��������
		float fL = (float)sqrt(m_fRadius*m_fRadius - fDist*fDist);

		// ��λLine��Բ�Ľ���
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

			// �����ж�ptNear���Ƿ������߶�Line֮��(��Ϊ��Ҳ���������߶�Line���ӳ�����)
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
//   �Դ�Բ��ȡһ��ֱ�ߣ�������ȡ������ֱ�߱��浽Line�С�
//
bool CCircle::CutLine(CLine& Line, CLine& NewLine)
{
	CPoint2d ptFoot, pt;
	float fLambda;
	float fDist = Line.DistanceToPoint(TRUE, m_ptCenter, &fLambda, &ptFoot);

	// ������߳��ȳ����뾶��˵��û�н�ȡ��ֱ��
	if (fDist > m_fRadius)
		return false;
	
	// ������ҳ�
	float fL = sqrt(m_fRadius*m_fRadius - fDist*fDist);

	// ��������Բ��
	if (Contain(Line.m_ptStart))
	{
		// �յ�Ҳ��Բ��
		if (Contain(Line.m_ptEnd))
		{
			// Line�ޱ仯
			NewLine = Line;
			return true;
		}
		// ����յ���Բ��
		else
		{
			CLine ln(ptFoot, Line.m_ptEnd);
			pt = ln.TrajFun(fL);

			// ���¹���ֱ��Line(���յ��б仯)
			NewLine.Create(Line.m_ptStart, pt);

			return true;
		}
	}

	// ��������Բ�⣬�յ���Բ��
	else if (Contain(Line.m_ptEnd))
	{
		CLine ln(ptFoot, Line.m_ptStart);

		// ������ҳ�
		float fL = sqrt(m_fRadius*m_fRadius - fDist*fDist);
		pt = ln.TrajFun(fL);

		// ���¹���ֱ��Line(������б仯)
		NewLine.Create(pt, Line.m_ptEnd);

		return true;
	}

	// ��㡢�յ����Բ��
	else
	{
		// �ֱ��촹��㵽Line��㡢�յ������
		CLine ln1(ptFoot, Line.m_ptStart);
		CLine ln2(ptFoot, Line.m_ptEnd);
		
		// �������ͬ��˵��Line�������˵����Բ�⣬����Բû�н�ȡ��ֱ��Line
		if (ln1.SlantAngle() == ln2.SlantAngle())
			return false;

		// ���߷���˵��Բ��ֱ��Line����������ȡ��
		else
		{
			CPoint2d pt1 = ln1.TrajFun(fL);
			CPoint2d pt2 = ln2.TrajFun(fL);

			// Line�������˵㶼�仯��
			NewLine.Create(pt1, pt2);

			return true;
		}
	}
}

//
//   �ж�һ�����Ƿ���Բ��(��Բ��)��
//
bool CCircle::Contain(const CPoint2d& pt)
{
	return (m_ptCenter.DistanceTo(pt) <= m_fRadius);
}

#ifdef _MSC_VER

//
//   ����Ļ�ϻ��ƴ�Բ��
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
