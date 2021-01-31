#include "stdafx.h"
#include <math.h>
#include "LineFeature.h"

///////////////////////////////////////////////////////////////////////////////

CLineFeature::CLineFeature() 
{
	m_bSelected = false;
}

bool CLineFeature::Load(FILE* fp)
{
	return true;
}

bool CLineFeature::Save(FILE* fp)
{
	return true;
}

//
//   �����߶Ρ�
//
void CLineFeature::Create(CPnt& ptStart, CPnt& ptEnd)
{
	CLine::Create(ptStart, ptEnd);

	m_bSelected = false;

	// �������m_fNormalX��m_fNormalY��m_fDistOrigin
	float xm = m_ptStart.x + m_ptEnd.x;
	float ym = m_ptStart.y + m_ptEnd.y;
	float xxm = m_ptStart.x * m_ptStart.x + m_ptEnd.x * m_ptEnd.x;
	float yym = m_ptStart.y * m_ptStart.y + m_ptEnd.y * m_ptEnd.y;
	float xym = m_ptStart.x * m_ptStart.y + m_ptEnd.x * m_ptEnd.y;

	float a = 0.5f * (float)atan2((float)(-2.0*(xym - xm*ym/2)), (float)(yym - ym*ym/2 - xxm + xm*xm/2));
	float n1 = (float)cos(a);
	float n2 = (float)sin(a);

	float dx = m_ptEnd.x - m_ptStart.x;
	float dy = m_ptEnd.y - m_ptStart.y;

	if (dx * n2 - dy * n1 > 0.0)
	{
		n1 = -n1;
		n2 = -n2;
	}

	m_fNormalX = n1;
	m_fNormalY = n2;
	m_fDistOrigin = -(n1 * xm + n2 * ym)/2;
}

//
//   �����߶Ρ�
//
void CLineFeature::Create(CLine& ln)
{
	Create(ln.m_ptStart, ln.m_ptEnd);
}

//
//   ��ת�߶εķ���(����㡢�յ�Ե�)��
//
void CLineFeature::Reverse()
{
	CPnt ptStart = m_ptStart;
	CPnt ptEnd = m_ptEnd;
	Create(ptEnd, ptStart);
}

//
//   ����ֱ�߶�ͶӰ����һֱ�߶Σ��õ���ͶӰ�߶δ���lnResult��
//
bool CLineFeature::ProjectToLine(CLineFeature& ln, CLineFeature& lnResult)
{
	CPnt ptFoot1, ptFoot2;

	// ���㱾�߶������˵㵽�߶�ln��ͶӰ��(����λ���߶ε��ӳ�����)
	ln.DistanceToPoint(false, m_ptStart, 0, &ptFoot1);
	ln.DistanceToPoint(false, m_ptEnd, 0, &ptFoot2);

	// �������̫�̣�˵��ͶӰ�����һ����(�ò����߶�)
	if (ptFoot1.DistanceTo(ptFoot2) < 1E-7)
		return false;
	
	// ����ֱ��
	lnResult.Create(ptFoot1, ptFoot2);
	return true;
}

//   ����������ƽ�ơ�
//
void CLineFeature::Move(float fX, float fY)
{
	CPnt ptStart = m_ptStart;
	CPnt ptEnd = m_ptEnd;

	ptStart.Move(fX, fY);
	ptEnd.Move(fX, fY);

	Create(ptStart, ptEnd);
}

//
//   ������������ת��
//
void CLineFeature::Rotate(CAngle ang, CPnt ptCenter)
{
	// ���������յ㾭��ת���λ��
	m_ptStart.Rotate(ang.m_fRad, ptCenter);
	m_ptEnd.Rotate(ang.m_fRad, ptCenter);

	Create(m_ptStart, m_ptEnd);
}

///////////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
//
//   (����Ļ������)����ָ���ĵ��Ƿ����߶��ϡ�
//
//bool CLineFeature::HitTest(CScreenReference& ScrnRef, CPoint point)
//{
//	CPoint pntStart = ScrnRef.GetWindowPoint(m_ptStart);
//	CPoint pntEnd = ScrnRef.GetWindowPoint(m_ptEnd);
//
//	CPnt ptStart((float)pntStart.x, (float)pntStart.y);
//	CPnt ptEnd((float)pntEnd.x, (float)pntEnd.y);
//	CPnt pt((float)point.x, (float)point.y);
//
//	CLine ln(ptStart, ptEnd);
//	float fLambda;
//	float fDist = ln.DistanceToPoint(false, pt, &fLambda);
//	if (fLambda < 0 || fLambda > 1 || fDist > 5)
//		return false;
//
//	return true;
//}

void CLineFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize)
{
}
#endif

