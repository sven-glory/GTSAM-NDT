//                          - LINE.CPP -
//
//   Implementation of class "CLine" - which defines the geometric concept
//   "Directional Straight Line".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#include "stdafx.h"
#include <math.h>
#include <float.h>
#include "Tools.h"
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define FLT_EPSILON        1.192092896e-07F

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   CLine: The constructor form #1.
//
CLine::CLine(CPoint2d& ptStart, CPoint2d& ptEnd)
{
	Create(ptStart, ptEnd);
}

//
//   CLine: The constructor form #2.
//
CLine::CLine(CPoint2d& ptStart, CAngle& angSlant, float fTotalLen)
{
	Create(ptStart, angSlant, fTotalLen);
}

void CLine::Create(CPoint2d& ptStart, CPoint2d& ptEnd)
{
//	ASSERT(ptStart != ptEnd);
	if (ptStart == ptEnd)
		return;
	// Init the start/end points
	m_ptStart = ptStart;
	m_ptEnd = ptEnd;

	// Caculate the total length of the line
	float fDx = m_ptEnd.x - m_ptStart.x;
	float fDy = m_ptEnd.y - m_ptStart.y;
	m_fTotalLen = (float)sqrt(Square(fDx) + Square(fDy));

	ASSERT(m_fTotalLen != 0);
	if (m_fTotalLen == 0)
		return;
	// Caculate the line's slant angle
	float fTemp = (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);
	m_angSlant = CAngle(fTemp);
}

void CLine::Create(CPoint2d& ptStart, CAngle& angSlant, float fTotalLen)
{
	ASSERT(m_fTotalLen != 0);
	if (m_fTotalLen == 0)
		return;
	CPoint2d ptEnd; 
	// Caculate the coordinates of the end point
	ptEnd.x = ptStart.x + fTotalLen * cos(angSlant);
	ptEnd.y = ptStart.y + fTotalLen * sin(angSlant);

	// construct the object using constructor #1
//	CLine(ptStart, ptEnd);
	Create(ptStart, ptEnd);
}

//
//   Constructor form #3
//
CLine::CLine(const CLine& Line2)
{
	*this = Line2;
}

//
//   ��������ֱ�߶ε����в���(��ǡ�����)��
//
void CLine::ComputeParam()
{
	// Caculate the total length of the line
	float fDx = m_ptEnd.x - m_ptStart.x;
	float fDy = m_ptEnd.y - m_ptStart.y;
	m_fTotalLen = (float)sqrt(Square(fDx) + Square(fDy));

	ASSERT(m_fTotalLen != 0);
	if (m_fTotalLen == 0)
		return;
	// Caculate the line's slant angle
	float fTemp = (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);
	m_angSlant = CAngle(fTemp);
}

//
//   ��ת�߶εķ���(����㡢�յ�Ե�)��
//
void CLine::Reverse()
{
	Create(m_ptEnd, m_ptStart);
}

//
//   �ı��߶εĳ���(�������ӳ�/����)��
//   fDist1: ����ʼ���ӳ��ľ��롣
//   fDist2: ����ֹ���ӳ��ľ��롣
//
void CLine::Resize(float fDist1, float fDist2)
{
	m_ptStart.x -= fDist1 * cos(m_angSlant);
	m_ptStart.y -= fDist1 * sin(m_angSlant);

	m_ptEnd.x += fDist2 * cos(m_angSlant);
	m_ptEnd.y += fDist2 * sin(m_angSlant);

	m_fTotalLen += fDist1 + fDist2;
}

//
//   Length: Get the length of the line.
//
float CLine::Length()
{
	return m_fTotalLen;
}

//
//   ȡ���߶γ��ȵ�ƽ����
//
float CLine::Length2()
{
	return m_ptStart.Distance2To(m_ptEnd);
}

//   TrajFun: The trajectory generation function.
//
CPoint2d& CLine::TrajFun(float fCurLen)
{
	m_pt.x = m_ptStart.x + fCurLen * cos(m_angSlant);
	m_pt.y = m_ptStart.y + fCurLen * sin(m_angSlant);

	return m_pt;
}

//
//   GetSlantAngle: Get the line's slant angle.
//
CAngle& CLine::SlantAngle()
{
	return m_angSlant;
}

//
//   ȡ���߶ε���б�ǡ�
//
float CLine::GetAngle() const
{
	return (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);
}

//
//   �����߶ε��е㡣
//
CPoint2d CLine::GetCenterPoint()
{
	CPoint2d ptCenter;
	ptCenter.x = (m_ptStart.x + m_ptEnd.x) / 2;
	ptCenter.y = (m_ptStart.y + m_ptEnd.y) / 2;
	
	return ptCenter;
}

//
//    The curvature generation function��
//
float CLine::CurvatureFun()
{
	return 0.0f;
}

//
//   �ж�һ�����Ƿ��ڴ�ֱ��(��)�ϡ�
//
BOOL CLine::ContainPoint(CPoint2d& pt, BOOL bExtend)
{
	if (pt == m_ptStart || pt == m_ptEnd)
		return TRUE;

	CLine ln1(pt, m_ptStart);
	CLine ln2(pt, m_ptEnd);

	// bExtend: ����������߶ε������ӳ�����
	if (bExtend)
	{
		if (ln1.SlantAngle() == SlantAngle() || !ln1.SlantAngle() == SlantAngle())
			return TRUE;
	}

	// ֻ����������߶�����
	else
	{
		if ((ln1.SlantAngle() == SlantAngle()) && (ln2.SlantAngle() == !SlantAngle()))
			return TRUE;
		else if ((ln2.SlantAngle() == SlantAngle()) && (ln1.SlantAngle() == !SlantAngle()))
			return TRUE;
	}

	return FALSE;
}

//
//   �ж�����ֱ���Ƿ�ƽ�С�
//   fMaxAngDiff: ��������������(����)��
//
BOOL CLine::IsParallelTo(CLine& Line, float fMaxAngDiff)
{
	if (m_angSlant.ApproxEqualTo(Line.SlantAngle(), fMaxAngDiff) ||	
		 (!m_angSlant).ApproxEqualTo(Line.SlantAngle(), fMaxAngDiff))
		return TRUE;
	else
		return FALSE;
}

//
//   �ж�����ֱ���Ƿ�ֱ��
//
bool CLine::IsVerticalTo(CLine& Line, float fMaxAngDiff)
{
	if (fMaxAngDiff == 0)
		fMaxAngDiff = CAngle::m_fReso;

	if ((fabs(m_angSlant.GetDifference(Line.SlantAngle()) - PI/2) < fMaxAngDiff) ||
		 (fabs(!m_angSlant.GetDifference(Line.SlantAngle()) - PI/2) < fMaxAngDiff))
		return true;
	else
		return false;
}

//
//   �ж�����ֱ���Ƿ���(fMaxAngDiff��fMaxDistDiff��������ĽǶȼ����������)��
//
BOOL CLine::IsColinearWith(CLine& Line, float fMaxAngDiff, float fMaxDistDiff)
{
	// ���ж��Ƿ�ƽ��
	if (!IsParallelTo(Line, fMaxAngDiff))
		return FALSE;

	// ����Line�������˵㵽��ֱ��(�����߶�!)�ľ���
	float fDist1 = DistanceToPoint(FALSE, Line.m_ptStart);
	float fDist2 = DistanceToPoint(FALSE, Line.m_ptEnd);

	// ������볬�ޣ��򲻹���
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return FALSE;

	return TRUE;
}

//
//   ȡ������ֱ�ߵĽ��㡣
//
BOOL CLine::IntersectLineAt(CLine& Line, CPoint2d& pt, float& fDist)
{
	CPoint2d pt1;
	float k1, k2;

	if (IsParallelTo(Line))
		return FALSE;

	// ����1����һ��ֱ��ƽ����Y��
	if (fabs(m_ptEnd.x - m_ptStart.x) < FLT_EPSILON)
	{
		if (fabs(Line.m_ptEnd.x - Line.m_ptStart.x) < FLT_EPSILON)
		{
			ASSERT(FALSE);               // ���߾�ƽ����Y�ᣬ�������ڴ˳���
		}
		else
		{
			k2 = (Line.m_ptEnd.y - Line.m_ptStart.y) / (Line.m_ptEnd.x - Line.m_ptStart.x);
			pt1.x = m_ptEnd.x;
			pt1.y = Line.m_ptStart.y + (pt1.x - Line.m_ptStart.x) * k2;
		}
	}

	// ����2���ڶ���ֱ��ƽ����Y��
	else if (fabs(Line.m_ptEnd.x - Line.m_ptStart.x) < FLT_EPSILON)
	{
		if (fabs(m_ptEnd.x - m_ptStart.x) < FLT_EPSILON)
		{
			ASSERT(FALSE);               // ���߾�ƽ����Y�ᣬ�������ڴ˳���
		}
		else
		{
			k1 = (m_ptEnd.y - m_ptStart.y) / (m_ptEnd.x - m_ptStart.x);
			pt1.x = Line.m_ptEnd.x;
			pt1.y = m_ptStart.y + (pt1.x - m_ptStart.x) * k1;
		}
	}

	// ����3������ֱ�߾���ƽ����Y��
	else
	{
		k1 = (m_ptEnd.y - m_ptStart.y) / (m_ptEnd.x - m_ptStart.x);
		k2 = (Line.m_ptEnd.y - Line.m_ptStart.y) / (Line.m_ptEnd.x - Line.m_ptStart.x);

		pt1.x = (Line.m_ptStart.y - m_ptStart.y + k1 * m_ptStart.x - k2 * Line.m_ptStart.x) / (k1 - k2);
		pt1.y = m_ptStart.y + (pt1.x - m_ptStart.x) * k1;
	}

	// ������Ҫ���������߶�����
	if (!ContainPoint(pt1) || !Line.ContainPoint(pt1))
		return FALSE;

	pt = pt1;
	fDist = pt1.DistanceTo(m_ptStart);
	return TRUE;
}

//
//   Intersects two lines.  Returns TRUE if intersection point exists,
//   FALSE otherwise.  (*px, *py) will hold intersection point,
//   *onSegment[12] is TRUE if point is on line segment[12].
//   px, py, onSegment[12] might be NULL.
//
BOOL CLine::Intersect(CLine& line2, float *px, float *py, BOOL *onSegment1, BOOL *onSegment2)
{
	float l1dx, l1dy, l2dx, l2dy, det, ldx1, ldy1, lambda1, lambda2;

	l1dx = m_ptEnd.x - m_ptStart.x;
	l1dy = m_ptEnd.y - m_ptStart.y;
	l2dx = line2.m_ptEnd.x - line2.m_ptStart.x;
	l2dy = line2.m_ptEnd.y - line2.m_ptStart.y;
	det = l1dy * l2dx - l1dx * l2dy;

	if (fabs(det) < 1e-10)
		return FALSE;

	ldx1 = m_ptStart.x - line2.m_ptStart.x;
	ldy1 = m_ptStart.y - line2.m_ptStart.y;
	lambda1 = (ldx1 * l2dy - ldy1 * l2dx) / (l1dy * l2dx - l1dx * l2dy);
	lambda2 = (ldx1 * l1dy - ldy1 * l1dx) / (l1dy * l2dx - l1dx * l2dy);

	if (px != NULL)
		*px = m_ptStart.x + l1dx * lambda1;

	if (py != NULL)
		*py = m_ptStart.y + l1dy * lambda1;

	if (onSegment1 != NULL)
		*onSegment1 = (lambda1 >= 0.0 && lambda1 <= 1.0);

	if (onSegment2 != NULL)
		*onSegment2 = (lambda2 >= 0.0 && lambda2 <= 1.0);

	return TRUE;
}

//
//   �����߶�������һ���߶κϲ���
//
BOOL CLine::Merge(CLine& Line2, float fMaxAngDiff, float fMaxDistDiff, bool bExtMode)
{
	// ���ж��Ƿ�ƽ�У������ƽ��ֱ�ӷ���FALSE
	if (!IsParallelTo(Line2, fMaxAngDiff))
		return FALSE;

	// �����ж��Ƿ������ص��Ĳ���
	float lambda1, lambda2;

	// �����߶�Line2����㡢�յ㵽��ֱ��(�����߶�!)�ľ���
	float fDist1 = DistanceToPoint(FALSE, Line2.m_ptStart, &lambda1);
	float fDist2 = DistanceToPoint(FALSE, Line2.m_ptEnd, &lambda2);

	// ������볬�ޣ��򲻹���
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return FALSE;

	// ���Line2�����λ�ڱ��߶ε�����֮��
	if (lambda1 < 0)
	{
		// ���Line2���յ�Ҳλ�ڱ��߶ε�����֮��
		if (lambda2 < 0)
		{
			// Line2��ȫλ�ڱ��߶�����
			if (!bExtMode)
				return FALSE;
			else
			{
				// ������������߶ε��ں��߶�
				float fDist3 = m_ptEnd.DistanceTo(Line2.m_ptStart);
				float fDist4 = m_ptEnd.DistanceTo(Line2.m_ptEnd);
				
				if (fDist3 > fDist4)
					Create(Line2.m_ptStart, m_ptEnd);
				else
					Create(Line2.m_ptEnd, m_ptEnd);
			
				return TRUE;
			}
		}
		// Line2���յ�λ�ڱ��߶ε��յ��֮��
		else if (lambda2 > 1)
		{
			// ˵��Line2�����˱��߶�
			*this = Line2;
			return TRUE;
		}
		else
		{
			// ˵��Line2����ʼ�����ڱ��߶�(��ʼ�㷽��)���棬������������ڱ��߶���
			Create(Line2.m_ptStart, m_ptEnd);
			return TRUE;
		}
	}

	// ���Line2�����λ�ڱ��߶ε��յ��֮��
	else if (lambda1 > 1)
	{
		// ���Line2���յ�Ҳλ�ڱ��߶ε��յ��֮��
		if (lambda2 > 1)
		{
			// Line2��ȫλ�ڱ��߶�����
			if (!bExtMode)
				return FALSE;
			else
			{
				// ������������߶ε��ں��߶�
				float fDist3 = m_ptStart.DistanceTo(Line2.m_ptStart);
				float fDist4 = m_ptStart.DistanceTo(Line2.m_ptEnd);
				
				if (fDist3 > fDist4)
					Create(m_ptStart, Line2.m_ptStart);
				else
					Create(m_ptStart, Line2.m_ptEnd);
			
				return TRUE;
			}
		}
		// Line2���յ�λ�ڱ��߶ε�����֮��
		else if (lambda2 < 0)
		{
			// ˵��Line2(����)�����˱��߶�(����Line2������Ӧ�ı䱾�߶�ԭ���ķ���)
			Create(Line2.m_ptEnd, Line2.m_ptStart);
			return TRUE;
		}
		else
		{
			// ˵��Line2��������ڱ��߶�(��ֹ�㷽��)���棬������������ڱ��߶���
			Create(m_ptStart, Line2.m_ptStart);
			return TRUE;
		}
	}

	// Line2�����λ�ڱ��߶���
	else
	{
		// Line2���յ�λ�ڱ��߶ε�����֮��
		if (lambda2 < 0)
		{
			Create(Line2.m_ptEnd, m_ptEnd);
		}
		// ���Line2���յ�λ�ڱ��߶ε��յ��֮��
		else if (lambda2 > 1)
		{
			Create(m_ptStart, Line2.m_ptEnd);
		}
		// ���Line2���յ�λ�ڱ��߶�֮��(���ظı䱾�߶�)
		else
		{
		}
		return TRUE;
	}
	return TRUE;
}

//
//   Calculates angle from line1 to line2, counterclockwise, result will be in interval [-PI;PI].
//
CAngle CLine::AngleToLine(CLine& line2)
{
	return line2.m_angSlant - m_angSlant;
}

//
//   �����ֱ������һ������ֱ�ߵĽǶȲ�(���������н�С��)��
//
//   ˵����line2Ϊһ���޷���ֱ�ߡ�������������ӱ�����ֱ�߿�ʼ������ʱ�뷽����ת����һ����
//   �޷���ֱ�����һ��ʱ����ת���ĽǶȡ�
//
CAngle CLine::AngleToUndirectionalLine(CLine& line2)
{
	CAngle ang1 = line2.m_angSlant - m_angSlant;
	CAngle ang2 = !line2.m_angSlant - m_angSlant;
	if (ang1.m_fRad < ang2.m_fRad)
		return ang1;
	else
		return ang2;
}

//
//   �����ֱ�ߵ�ָ����ľ��롣
//
float CLine::DistanceToPoint(BOOL bIsSegment, const CPoint2d& pt, float* pLambda, CPoint2d* pFootPoint)
{
	float dx = m_ptEnd.x - m_ptStart.x;
	float dy = m_ptEnd.y - m_ptStart.y;
	float d2 = dx*dx + dy*dy;

	float lambda = ((pt.y - m_ptStart.y) * dy + (pt.x - m_ptStart.x) * dx) / d2;

	if (pLambda != NULL)
		*pLambda = lambda;

	if (bIsSegment)
	{
		/* make sure point is on line (lambda <- [0..1]) */
		if (lambda < 0)
			lambda = 0;
		else if (lambda > 1)
			lambda = 1.0f;
	}

	float x = m_ptStart.x + lambda * dx;
	float y = m_ptStart.y + lambda * dy;

	if (pFootPoint != NULL)
	{
		pFootPoint->x = x;
		pFootPoint->y = y;
	}

	return (float)_hypot(pt.x - x, pt.y - y);
}

//
//   ��ֱ����ָ�������ĵ������ת��
//
void CLine::Rotate(float fAng, float fCx, float fCy)
{
	m_ptStart.Rotate(fAng, fCx, fCy);
	m_ptEnd.Rotate(fAng, fCx, fCy);

	Create(m_ptStart, m_ptEnd);
}

//
//   ��ֱ����ָ�������ĵ������ת��
//
void CLine::Rotate(float fAng, CPoint2d ptCenter)
{
	m_ptStart.Rotate(fAng, ptCenter);
	m_ptEnd.Rotate(fAng, ptCenter);

	Create(m_ptStart, m_ptEnd);
}

//
//   ȡ��ֱ�߶ε�б��k��Y��ؾ�b��(��ֱ�ߴ�ֱ��X��ʱ��)X��ؾ�c��
//   ����ֵ��
//     1 - ֱ�߲���ֱ��X�ᣬk��b����ֵ������, c������
//     0 - ֱ�߲���ֱ��X�ᣬk��b����ֵ��������, c������
//
int CLine::GetParam(float* k, float* b, float* c)
{
	CLine lineXAxis(CPoint2d(0, 0), CAngle(0), FLT_MAX);

	// ֱ�߲���ֱ��X�ᣬ����ֱ�ߵ�б�ʼ���Y���ϵĽؾ�
	if (!IsVerticalTo(lineXAxis))
	{
		float K, B;
		
		// ����б��
		if (m_angSlant > PI)
			K = tan(!m_angSlant);
		else
			K = tan(m_angSlant);

		// ����Y��ؾ�
		B = m_ptStart.y - K * m_ptStart.x;

		if (k != NULL)
			*k = K;

		if (b != NULL)
			*b = B;

		return 1;
	}

	// ֱ�ߴ�ֱ��X�ᣬ����ֱ����X���ϵĽؾ�
	else
	{
		if (c != NULL)
			*c = GetCenterPoint().x;
		return 0;
	}
}

//
//   ����ֱ�ߵ������˵����ĸ�����ָ���ĵ�pt����������fDist�з��ش˽����롣
//   ����ֵ��
//     0 - pt�������m_ptStart����
//     1 - pt�����յ�m_ptEnd����
//
int CLine::FindNearPoint(CPoint2d& pt, float* pDist)
{
	float f[2];
	
	f[0] = pt.DistanceTo(m_ptStart);
	f[1] = pt.DistanceTo(m_ptEnd);
	
	if (f[0] < f[1])
	{
		if (pDist != NULL)
			*pDist = f[0];
		
		return 0;
	}
	else
	{
		if (pDist != NULL)
			*pDist = f[1];
		
		return 1;
	}
}

#ifdef _MSC_VER

//
//   ����Ļ�ϻ��ƴ�ֱ�ߡ�
//
void CLine::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, BOOL bBigVertex)
{
	CPen pen(PS_SOLID, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPoint2d pt1(m_ptStart.x, m_ptStart.y);
	CPoint2d pt2(m_ptEnd.x, m_ptEnd.y);

	CPoint pnt1 = ScrnRef.GetWindowPoint(pt1);
	CPoint pnt2 = ScrnRef.GetWindowPoint(pt2);

	pDC->MoveTo(pnt1);
	pDC->LineTo(pnt2);

	pDC->SelectObject(pOldPen);

	if (bBigVertex)
	{
		m_ptStart.Draw(ScrnRef, pDC, crColor, nPointSize * 2);
		m_ptEnd.Draw(ScrnRef, pDC, crColor, nPointSize * 2);
	}
}

#endif
