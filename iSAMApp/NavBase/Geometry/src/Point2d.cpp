#include "stdafx.h"
//#include "misc.h"
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   �����������캯����
//
void CPoint2d::operator = (const CPoint& point)
{
	x = (float)point.x;
	y = (float)point.y;
}

//
//   �����ƶ�ָ���ľ��롣
//
void CPoint2d::Move(float dx, float dy)
{
	x += dx;
	y += dy;
}

//
//   ������ԭ�������ת��
//
void CPoint2d::Rotate(float fAng, float fCx, float fCy)
{
	float fX = (x - fCx) * cos(fAng) - (y - fCy) * sin(fAng) + fCx;
	float fY = (x - fCx) * sin(fAng) + (y - fCy) * cos(fAng) + fCy;

	x = fX;
	y = fY;
}

//
//   ������ָ�������ĵ������ת��
//
void CPoint2d::Rotate(float fAng, CPoint2d ptCenter)
{
	float fX = (x - ptCenter.x) * cos(fAng) - (y - ptCenter.y) * sin(fAng) + ptCenter.x;
	float fY = (x - ptCenter.x) * sin(fAng) + (y - ptCenter.y) * cos(fAng) + ptCenter.y;

	x = fX;
	y = fY;
}

//
//   ���ز����� "==".
//
BOOL CPoint2d::operator == (const CPoint2d& pt)
{
	return (x == pt.x && y == pt.y);
}

//
//   ���ز����� "!=".
//
BOOL CPoint2d::operator != (const CPoint2d& pt)
{
	return (x != pt.x || y != pt.y);
}

//
//   ���ز����� "==".
//
BOOL CPoint2d::operator == (CPoint& point)
{
	return ((x == (float)point.x) && (y == (float)point.y));
}

//
//   ���ز����� "!=".
//
BOOL CPoint2d::operator != (CPoint& point)
{
	return (x != (float)point.x || y != (float)point.y);
}

//
//   ����������֮��ľ���.
//
float CPoint2d::DistanceTo(const CPoint2d& pt)
{
	return (float)_hypot(x - pt.x, y - pt.y);
}


//
//   ����������֮��ľ����ƽ����
//
float CPoint2d::Distance2To(const CPoint2d& pt2)
{
	float d_x = x - pt2.x;
	float d_y = y - pt2.y;

	return d_x * d_x + d_y * d_y;
}

//
//   �ж��������Ƿ���Խ��Ƶ���Ϊ��һ����(���ǳ���)��
//
bool CPoint2d::IsEqualTo(const CPoint2d& pt2, float limit)
{
	if (Distance2To(pt2) < (limit*limit))
		return true;
	else
		return false;
}

//
//   ���ݵϿ�������������ļ����ꡣ
//
void CPoint2d::UpdatePolar()
{
	// ���㼫��
	r = sqrt(y*y + x*x);

	// ���㼫��
	// �������̫С��˵���㴦��ԭ�㸽�����޼���
	if (r < 1E-7)
		a = 0;
	else
	{
		float fAngle = (float)atan2(y, x);
		a = CAngle::NormAngle(fAngle);
	}
}

//
//   ���ݼ�����������ĵϿ������ꡣ
//
void CPoint2d::UpdateCartisian()
{
	x = r * cos(a);
	y = r * sin(a);
}

//
//   ����õ㵽��һ��ĽǶȾ��롣
//
float CPoint2d::AngleDistanceTo(const CPoint2d& pt)
{
	return (float)fabs(CAngle::NormAngle(a - pt.a));
}

//
//   ���ļ�װ������ݡ�
//
bool CPoint2d::Load(FILE* fp)
{
	if (fscanf(fp, "%d\t%f\t%f\t%f\t%f\n", &id, &x, &y, &r, &a) != 5)
		return false;

	return true;
}

//
//   ��������д���ļ���
//
bool CPoint2d::Save(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t%f\t%f\n", id, x, y, r, a);
	return true;
}


#ifdef _MSC_VER

void CPoint2d::Dump()
{
	TRACE(_T("%d\t%f\t%f\t%f\t%f\n"), id, x, y, r, a);
}

//
//   ����Ļ�ϻ��Ƹõ�.
//
void CPoint2d::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize)
{
	bool bSolidFill = true;
	
	if (nPointSize < 0)
	{
		bSolidFill = false;
		nPointSize = -nPointSize;
	}

	CPoint2d pt(x, y);

	CPoint pnt1 = ScrnRef.GetWindowPoint(pt);
	CRect r(pnt1.x - nPointSize, pnt1.y - nPointSize, pnt1.x + nPointSize, pnt1.y + nPointSize);

	CPen Pen(PS_SOLID, 1, color);
	CPen* pOldPen = pDC->SelectObject(&Pen);

	CBrush Brush(color);
	CBrush* pOldBrush;

	if (bSolidFill)
		pOldBrush = pDC->SelectObject(&Brush);
	else
		pOldBrush = (CBrush*)pDC->SelectStockObject(NULL_BRUSH);

	pDC->Ellipse(&r);

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);
}
#endif