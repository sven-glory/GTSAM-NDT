#include "stdafx.h"
#include <math.h>
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   点斜式构建。
//
CLineBase::CLineBase(const CPnt& pt, float fAng)
{
	Create(pt.x, pt.y, fAng);
}

//
//   点斜式构建直线。
//
void CLineBase::Create(float x, float y, float fAng)
{
	float cos_ = (float)cos(fAng);
	float sin_ = (float)sin(fAng);

	a = sin_;
	b = -cos_;
	c = -sin_ * x + cos_ * y;

	// 将a调整为正数
	if (a < 0)
	{
		a = -a;
		b = -b;
		c = -c;
	}

	// 最后进行标准化
	float fNorm = (float)sqrt(a * a + b * b);
	a /= fNorm;
	b /= fNorm;
	c /= fNorm;
}

//
//   通过对点云的线性回归求得最优拟合线。
//   说明：
//     p      - 点云数组缓冲区指针
//     nCount - 点的数量
//
bool CLineBase::CreateFitLine(const CPnt* sp, int num)
{
	if (num <= 1)
		return false;              // 点数太少，无法拟合直线

	float xm = 0, ym = 0, xxm = 0, yym = 0, xym = 0;
	for (int i = 0; i < num; i++)
	{
		float x = sp[i].x;
		float y = sp[i].y;

		xm += x;
		ym += y;
		xxm += x * x;
		yym += y * y;
		xym += x * y;
	}

	float a = (float)atan2(-2 * (xym - xm * ym / num), yym - ym * ym / num - xxm + xm * xm / num) / 2;
	float n1 = (float)cos(a);
	float n2 = (float)sin(a);
	float dx = sp[num - 1].x - sp[0].x;
	float dy = sp[num - 1].y - sp[0].y;

	if (dx * n2 - dy * n1 > 0)
	{
		n1 = -n1;
		n2 = -n2;
	}

	float c = -(n1 * xm + n2 * ym) / num;

	DirectCreate(n1, n2, c);
	return true;
}

//
//   计算该直线到指定点的距离。
//
//   说明：点(x0, y0)到直线(a*x + b*y + c = 0)距离公式为：
//     d = abs(a*x0 + b*y0 + c)/sqrt(a*a + b*b)
//
float CLineBase::DistanceToPoint(const CPnt& pt) const
{
	return (float)(fabs(a * pt.x + b * pt.y + c) / sqrt(a*a + b*b));
}

//
//   求得直线外一点到此直线的投影。
//
CPnt CLineBase::GetProjectPoint(const CPnt& pt) const
{
	CPnt ptFoot;
	ptFoot.x = (b*b*pt.x - a*b*pt.y - a*c) / (a*a + b*b);
	ptFoot.y = (-a*b*pt.x + a*a*pt.y - b*c) / (a*a + b*b);

	return ptFoot;
}

//
//   求一直线段到此直线的投影。
//
CLine CLineBase::GetProjectLine(const CLine& Line) const
{
	CPnt pt1 = GetProjectPoint(Line.m_ptStart);
	CPnt pt2 = GetProjectPoint(Line.m_ptEnd);

	CLine lnProject(pt1, pt2);
	return lnProject;
}

//
//   取得直线的倾斜角(弧度)。
//   注：
//      由于CLineBase代表无限长直线，它也没有方向，所以本函数返回的角度值在[0, PI)之内。
//
float CLineBase::StdSlantAngleRad() const
{
	float fAng = (float)atan2(-a, b);
	if (fAng < 0)
		fAng += PI;

	return fAng;
}
