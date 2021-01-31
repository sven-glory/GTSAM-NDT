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
//   “拷备”构造函数。
//
void CPoint2d::operator = (const CPoint& point)
{
	x = (float)point.x;
	y = (float)point.y;
}

//
//   将点移动指定的距离。
//
void CPoint2d::Move(float dx, float dy)
{
	x += dx;
	y += dy;
}

//
//   将点绕原点进行旋转。
//
void CPoint2d::Rotate(float fAng, float fCx, float fCy)
{
	float fX = (x - fCx) * cos(fAng) - (y - fCy) * sin(fAng) + fCx;
	float fY = (x - fCx) * sin(fAng) + (y - fCy) * cos(fAng) + fCy;

	x = fX;
	y = fY;
}

//
//   将点绕指定的中心点进行旋转。
//
void CPoint2d::Rotate(float fAng, CPoint2d ptCenter)
{
	float fX = (x - ptCenter.x) * cos(fAng) - (y - ptCenter.y) * sin(fAng) + ptCenter.x;
	float fY = (x - ptCenter.x) * sin(fAng) + (y - ptCenter.y) * cos(fAng) + ptCenter.y;

	x = fX;
	y = fY;
}

//
//   重载操作符 "==".
//
BOOL CPoint2d::operator == (const CPoint2d& pt)
{
	return (x == pt.x && y == pt.y);
}

//
//   重载操作符 "!=".
//
BOOL CPoint2d::operator != (const CPoint2d& pt)
{
	return (x != pt.x || y != pt.y);
}

//
//   重载操作符 "==".
//
BOOL CPoint2d::operator == (CPoint& point)
{
	return ((x == (float)point.x) && (y == (float)point.y));
}

//
//   重载操作符 "!=".
//
BOOL CPoint2d::operator != (CPoint& point)
{
	return (x != (float)point.x || y != (float)point.y);
}

//
//   计算两个点之间的距离.
//
float CPoint2d::DistanceTo(const CPoint2d& pt)
{
	return (float)_hypot(x - pt.x, y - pt.y);
}


//
//   计算两个点之间的距离的平方。
//
float CPoint2d::Distance2To(const CPoint2d& pt2)
{
	float d_x = x - pt2.x;
	float d_y = y - pt2.y;

	return d_x * d_x + d_y * d_y;
}

//
//   判断两个点是否可以近似地认为是一个点(相距非常近)。
//
bool CPoint2d::IsEqualTo(const CPoint2d& pt2, float limit)
{
	if (Distance2To(pt2) < (limit*limit))
		return true;
	else
		return false;
}

//
//   根据迪卡尔坐标计算出点的极坐标。
//
void CPoint2d::UpdatePolar()
{
	// 计算极径
	r = sqrt(y*y + x*x);

	// 计算极角
	// 如果极径太小，说明点处于原点附近，无极角
	if (r < 1E-7)
		a = 0;
	else
	{
		float fAngle = (float)atan2(y, x);
		a = CAngle::NormAngle(fAngle);
	}
}

//
//   根据极坐标计算出点的迪卡尔坐标。
//
void CPoint2d::UpdateCartisian()
{
	x = r * cos(a);
	y = r * sin(a);
}

//
//   计算该点到另一点的角度距离。
//
float CPoint2d::AngleDistanceTo(const CPoint2d& pt)
{
	return (float)fabs(CAngle::NormAngle(a - pt.a));
}

//
//   从文件装入点数据。
//
bool CPoint2d::Load(FILE* fp)
{
	if (fscanf(fp, "%d\t%f\t%f\t%f\t%f\n", &id, &x, &y, &r, &a) != 5)
		return false;

	return true;
}

//
//   将点数据写入文件。
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
//   在屏幕上绘制该点.
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