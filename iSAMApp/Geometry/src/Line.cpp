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

//#define FLT_EPSILON        1.192092896e-07F
#define MIN_LINE_LENGTH       1e-3F

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   第一种构造函数。
//
CLine::CLine(const CPnt& ptStart, const CPnt& ptEnd)
{
	Create(ptStart, ptEnd);
}

//
//   第二种构造函数。
//
CLine::CLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen)
{
	Create(ptStart, angSlant, fTotalLen);
}

//
//   第三种构造函数。
//
CLine::CLine(const CPosture& pstStart, float fTotalLen)
{
	Create(pstStart, pstStart.GetAngle(), fTotalLen);
}

//
//   第四种构造函数。
//
CLine::CLine(const CLine& Line2)
{
	*this = Line2;
}

//
//   根据两点生成线段。
//
bool CLine::Create(const CPnt& ptStart, const CPnt& ptEnd)
{
	// 计算直线长度
	float fTotalLen = ptStart.DistanceTo(ptEnd);

	// 如果直线太短，返回false
	if (fTotalLen < MIN_LINE_LENGTH)
		return false;

	// 设置起点/终点/长度
	m_ptStart = ptStart;
	m_ptEnd = ptEnd;
	m_fTotalLen = fTotalLen;
	m_nId = 0;

	// 计算直线的倾角
	m_angSlant = (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);

	// 下面计算a、b和c
	float xm = m_ptStart.x + m_ptEnd.x;
	float ym = m_ptStart.y + m_ptEnd.y;
	float xxm = m_ptStart.x * m_ptStart.x + m_ptEnd.x * m_ptEnd.x;
	float yym = m_ptStart.y * m_ptStart.y + m_ptEnd.y * m_ptEnd.y;
	float xym = m_ptStart.x * m_ptStart.y + m_ptEnd.x * m_ptEnd.y;

	float ang = 0.5f * (float)atan2((float)(-2.0*(xym - xm*ym / 2)), (float)(yym - ym*ym / 2 - xxm + xm*xm / 2));
	float n1 = (float)cos(ang);
	float n2 = (float)sin(ang);

	float dx = m_ptEnd.x - m_ptStart.x;
	float dy = m_ptEnd.y - m_ptStart.y;

	if (dx * n2 - dy * n1 > 0.0)
	{
		n1 = -n1;
		n2 = -n2;
	}

	a = n1;
	b = n2;
	c = -(n1 * xm + n2 * ym) / 2;
	
	return true;
}

//
//   根据起始点、倾角和长度生成线段。
//
bool CLine::Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen)
{
	// 直线不能过短
	if (fTotalLen < MIN_LINE_LENGTH)
		return false;

	// 计算终点坐标
	CPnt ptEnd; 
	ptEnd.x = ptStart.x + fTotalLen * cos(angSlant);
	ptEnd.y = ptStart.y + fTotalLen * sin(angSlant);

	// 利用两点生成直线段
	return Create(ptStart, ptEnd);
}

//
//   计算完整直线段的所有参数(倾角、长度)。
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
//   反转线段的方向(将起点、终点对调)。
//
void CLine::Reverse()
{
	CPnt ptStart = m_ptStart;
	CPnt ptEnd = m_ptEnd;
	Create(ptEnd, ptStart);
}

//
//   改变线段的长度(沿两端延长/缩短)。
//   fDist1: 沿起始点延长的距离。
//   fDist2: 沿终止点延长的距离。
//
//   注意：上面两个距离值均可为负数！
//
bool CLine::Resize(float fDist1, float fDist2)
{
	// 路段长度不得过小(更不能为负)
	if (m_fTotalLen + fDist1 + fDist2 < MIN_LINE_LENGTH)
		return false;

	m_ptStart.x -= fDist1 * cos(m_angSlant);
	m_ptStart.y -= fDist1 * sin(m_angSlant);

	m_ptEnd.x += fDist2 * cos(m_angSlant);
	m_ptEnd.y += fDist2 * sin(m_angSlant);

	m_fTotalLen += fDist1 + fDist2;
	
	return true;
}

//
//   取得线段长度的平方。
//
float CLine::Length2() const
{
	return m_ptStart.Distance2To(m_ptEnd);
}

//
//   轨迹点函数。
//
CPnt CLine::TrajFun(float fCurLen) const
{
	CPnt pt = m_ptStart;
	pt.x += fCurLen * cos(m_angSlant);
	pt.y += fCurLen * sin(m_angSlant);

	return pt;
}

//
//   取得线段的倾斜角。
//
float CLine::SlantAngleRad() const
{
	return (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);
}

//
//   返回线段的中点。
//
CPnt CLine::GetMidpoint() const
{
	CPnt ptCenter;
	ptCenter.x = (m_ptStart.x + m_ptEnd.x) / 2;
	ptCenter.y = (m_ptStart.y + m_ptEnd.y) / 2;

	return ptCenter;
}

#if 0
//
//    The curvature generation function。
//
float CLine::CurvatureFun()
{
	return 0.0f;
}
#endif

//
//   判断一个点是否在此直线(段)上。
// !!!!!!!!! 下面的计算可能有问题，当ln1, ln2非常短时，可能会出现错误结果!!!!
//
bool CLine::ContainPoint(const CPnt& pt, bool bExtend) const
{
	if (pt == m_ptStart || pt == m_ptEnd)
		return true;

	// !!!!!!!!! 下面的计算可能有问题，当ln1, ln2非常短时，可能会出现错误结果!!!!
	CLine ln1(pt, m_ptStart);
	CLine ln2(pt, m_ptEnd);

	// bExtend: 允许点落在线段的两端延长线上
	if (bExtend)
	{
		if (ln1.SlantAngle() == m_angSlant || !ln1.SlantAngle() == m_angSlant)
			return true;
	}

	// 只允许点落在线段以内
	else
	{
		if ((ln1.SlantAngle() == m_angSlant) && (ln2.SlantAngle() == !m_angSlant))
			return true;
		else if ((ln2.SlantAngle() == m_angSlant) && (ln1.SlantAngle() == !m_angSlant))
			return true;
	}

	return false;
}

//
//   判断两条直线是否平行。
//   fMaxAngDiff: 所允许的最大角误差(弧度)。
//
bool CLine::IsParallelTo(const CLine& Line, float fMaxAngDiff) const
{
	if (fMaxAngDiff == 0)
		fMaxAngDiff = CAngle::m_fReso;

	CAngle ang1 = Line.SlantAngle();
	if (m_angSlant.ApproxEqualTo(ang1, fMaxAngDiff) || (!m_angSlant).ApproxEqualTo(ang1, fMaxAngDiff))
		return true;
	else
		return false;
}

//
//   判断两条直线是否垂直。
//
bool CLine::IsVerticalTo(const CLine& Line, float fMaxAngDiff) const
{
	if (fMaxAngDiff == 0)
		fMaxAngDiff = CAngle::m_fReso;

	CAngle ang1 = Line.SlantAngle() - PI / 2;
	if ((fabs(m_angSlant.GetDifference(ang1)) < fMaxAngDiff) ||
		(fabs((float)!m_angSlant.GetDifference(ang1)) < fMaxAngDiff))
		return true;
	else
		return false;
}

//
//   判断两条直线是否共线(fMaxAngDiff和fMaxDistDiff是所允许的角度及距离误差限)。
//
bool CLine::IsColinearWith(const CLine& Line, float fMaxAngDiff, float fMaxDistDiff) const
{
	// 先判断是否平行
	if (!IsParallelTo(Line, fMaxAngDiff))
		return false;

	// 计算Line的两个端点到本直线(不是线段!)的距离
	float fDist1 = DistanceToPoint(false, Line.m_ptStart);
	float fDist2 = DistanceToPoint(false, Line.m_ptEnd);

	// 如果距离超限，则不共线
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return false;

	return true;
}

//
//   取得两条直线的交点。
//
bool CLine::IntersectLineAt(const CLine& Line, CPnt& pt, float& fDist) const
{
	CPnt pt1;
	float k1, k2;

	if (IsParallelTo(Line))
		return false;

	// 情形1：第一条直线平行于Y轴
	if (fabs(m_ptEnd.x - m_ptStart.x) < MIN_LINE_LENGTH)
	{
		if (fabs(Line.m_ptEnd.x - Line.m_ptStart.x) < MIN_LINE_LENGTH)
		{
			ASSERT(false);               // 两线均平行于Y轴，不可能在此出现
		}
		else
		{
			k2 = (Line.m_ptEnd.y - Line.m_ptStart.y) / (Line.m_ptEnd.x - Line.m_ptStart.x);
			pt1.x = m_ptEnd.x;
			pt1.y = Line.m_ptStart.y + (pt1.x - Line.m_ptStart.x) * k2;
		}
	}

	// 情形2：第二条直线平行于Y轴
	else if (fabs(Line.m_ptEnd.x - Line.m_ptStart.x) < MIN_LINE_LENGTH)
	{
		if (fabs(m_ptEnd.x - m_ptStart.x) < MIN_LINE_LENGTH)
		{
			ASSERT(false);               // 两线均平行于Y轴，不可能在此出现
		}
		else
		{
			k1 = (m_ptEnd.y - m_ptStart.y) / (m_ptEnd.x - m_ptStart.x);
			pt1.x = Line.m_ptEnd.x;
			pt1.y = m_ptStart.y + (pt1.x - m_ptStart.x) * k1;
		}
	}

	// 情形3：两条直线均不平行于Y轴
	else
	{
		k1 = (m_ptEnd.y - m_ptStart.y) / (m_ptEnd.x - m_ptStart.x);
		k2 = (Line.m_ptEnd.y - Line.m_ptStart.y) / (Line.m_ptEnd.x - Line.m_ptStart.x);

		pt1.x = (Line.m_ptStart.y - m_ptStart.y + k1 * m_ptStart.x - k2 * Line.m_ptStart.x) / (k1 - k2);
		pt1.y = m_ptStart.y + (pt1.x - m_ptStart.x) * k1;
	}

	// 交点需要处于两条线段以内
	if (!ContainPoint(pt1) || !Line.ContainPoint(pt1))
		return false;

	pt = pt1;
	fDist = pt1.DistanceTo(m_ptStart);
	return true;
}

//
//   Intersects two lines.  Returns true if intersection point exists,
//   false otherwise.  (*px, *py) will hold intersection point,
//   *onSegment[12] is true if point is on line segment[12].
//   px, py, onSegment[12] might be NULL.
//
bool CLine::Intersect(const CLine& line2, float *px, float *py, bool *onSegment1, bool *onSegment2, float fSmallGate) const
{
	float l1dx, l1dy, l2dx, l2dy, det, ldx1, ldy1, lambda1, lambda2;

	l1dx = m_ptEnd.x - m_ptStart.x;
	l1dy = m_ptEnd.y - m_ptStart.y;
	l2dx = line2.m_ptEnd.x - line2.m_ptStart.x;
	l2dy = line2.m_ptEnd.y - line2.m_ptStart.y;
	det = l1dy * l2dx - l1dx * l2dy;

	if (fabs(det) < fSmallGate)
		return false;

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

	return true;
}

//
//   将此线段与另外一条线段合并。
//
bool CLine::Merge(CLine& Line2, float fMaxAngDiff, float fMaxDistDiff, bool bExtMode)
{
	// 先判断是否平行，如果不平行直接返回FALSE
	if (!IsParallelTo(Line2, fMaxAngDiff))
		return FALSE;

	// 下面判断是否有相重叠的部分
	float lambda1, lambda2;

	// 计算线段Line2的起点、终点到本直线(不是线段!)的距离
	float fDist1 = DistanceToPoint(FALSE, Line2.m_ptStart, &lambda1);
	float fDist2 = DistanceToPoint(FALSE, Line2.m_ptEnd, &lambda2);

	// 如果距离超限，则不共线
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return FALSE;

	// 如果Line2的起点位于本线段的起点侧之外
	if (lambda1 < 0)
	{
		// 如果Line2的终点也位于本线段的起点侧之外
		if (lambda2 < 0)
		{
			// Line2完全位于本线段以外
			if (!bExtMode)
				return FALSE;
			else
			{
				// 构造包含两个线段的融合线段
				float fDist3 = m_ptEnd.DistanceTo(Line2.m_ptStart);
				float fDist4 = m_ptEnd.DistanceTo(Line2.m_ptEnd);
				
				if (fDist3 > fDist4)
					Create(Line2.m_ptStart, m_ptEnd);
				else
					Create(Line2.m_ptEnd, m_ptEnd);
			
				return TRUE;
			}
		}
		// Line2的终点位于本线段的终点侧之外
		else if (lambda2 > 1)
		{
			// 说明Line2包含了本线段
			*this = Line2;
			return TRUE;
		}
		else
		{
			// 说明Line2的起始点落在本线段(起始点方向)外面，而其结束点落在本线段上
			Create(Line2.m_ptStart, m_ptEnd);
			return TRUE;
		}
	}

	// 如果Line2的起点位于本线段的终点侧之外
	else if (lambda1 > 1)
	{
		// 如果Line2的终点也位于本线段的终点侧之外
		if (lambda2 > 1)
		{
			// Line2完全位于本线段以外
			if (!bExtMode)
				return FALSE;
			else
			{
				// 构造包含两个线段的融合线段
				float fDist3 = m_ptStart.DistanceTo(Line2.m_ptStart);
				float fDist4 = m_ptStart.DistanceTo(Line2.m_ptEnd);
				
				if (fDist3 > fDist4)
					Create(m_ptStart, Line2.m_ptStart);
				else
					Create(m_ptStart, Line2.m_ptEnd);
			
				return TRUE;
			}
		}
		// Line2的终点位于本线段的起点侧之外
		else if (lambda2 < 0)
		{
			// 说明Line2(逆向)包含了本线段(采用Line2，但不应改变本线段原来的方向)
			Create(Line2.m_ptEnd, Line2.m_ptStart);
			return TRUE;
		}
		else
		{
			// 说明Line2的起点落在本线段(终止点方向)外面，而其结束点落在本线段上
			Create(m_ptStart, Line2.m_ptStart);
			return TRUE;
		}
	}

	// Line2的起点位于本线段上
	else
	{
		// Line2的终点位于本线段的起点侧之外
		if (lambda2 < 0)
		{
			Create(Line2.m_ptEnd, m_ptEnd);
		}
		// 如果Line2的终点位于本线段的终点侧之外
		else if (lambda2 > 1)
		{
			Create(m_ptStart, Line2.m_ptEnd);
		}
		// 如果Line2的终点位于本线段之内(不必改变本线段)
		else
		{
		}
		return TRUE;
	}
	return TRUE;
}
//   Calculates angle from line1 to line2, counterclockwise, result will be in interval [-PI;PI].
//
CAngle CLine::AngleToLine(const CLine& line2) const
{
	return line2.m_angSlant - m_angSlant;
}

//
//   计算该直线与另一条无向直线的角度差(两个方向中较小的)。
//
//   说明：line2为一“无方向直线”，本函数计算从本有向直线开始，沿逆时针方向旋转，第一次与
//   无方向直线倾角一致时所旋转过的角度。
//
CAngle CLine::AngleToUndirectionalLine(const CLine& line2) const
{
	CAngle ang1 = line2.m_angSlant - m_angSlant;
	CAngle ang2 = !line2.m_angSlant - m_angSlant;
	if (ang1.m_fRad < ang2.m_fRad)
		return ang1;
	else
		return ang2;
}

//
//   计算该直线到指定点的距离。
//
float CLine::DistanceToPoint(const CPnt& pt) const
{
	return CLineBase::DistanceToPoint(pt);
}

//
//   计算该直线到指定点的距离。
//
float CLine::DistanceToPoint(bool bIsSegment, const CPnt& pt, float* pLambda, CPnt* pFootPoint) const
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
//   判断一个给定的点是否“触碰”到该直线。
//   返回值：
//      0 - 未触碰到直线
//      1 - 触碰到起点
//      2 - 触碰到终点
//      3 - 触碰到两个端点以外的直线部分
//
int CLine::PointHit(const CPnt& pt, float fDistGate)
{
	float Lambda;

	// 计算点到直线的投影距离
	float fDist = DistanceToPoint(false, pt, &Lambda);

	// 如果投影点位于线段之外，返回false
	if (Lambda < 0 || Lambda > 1)
		return 0;

	// 如果距离超限，也返回false
	else if (fDist > fDistGate)
		return 0;
	else
	{
		if (pt.DistanceTo(m_ptStart) < fDistGate)
			return 1;
		else if (pt.DistanceTo(m_ptEnd) < fDistGate)
			return 2;
		else
			return 3;
	}
}

//
//   将直线绕指定的中心点进行旋转。
//
void CLine::Rotate(float fAng, float fCx, float fCy)
{
	m_ptStart.Rotate(fAng, fCx, fCy);
	m_ptEnd.Rotate(fAng, fCx, fCy);

	Create(m_ptStart, m_ptEnd);
}

//
//   将直线绕指定的中心点进行旋转。
//
void CLine::Rotate(float fAng, CPnt ptCenter)
{
	m_ptStart.Rotate(fAng, ptCenter);
	m_ptEnd.Rotate(fAng, ptCenter);

	Create(m_ptStart, m_ptEnd);
}

//
//   变换到局部坐标系。
//
CLine CLine::TransformToLocal(CTransform& trans)
{
	CPnt pt1 = trans.GetLocalPoint(m_ptStart);
	CPnt pt2 = trans.GetLocalPoint(m_ptEnd);

	CLine Line(pt1, pt2);
	return Line;
}

//
//   变换到全局坐标系。
//
CLine CLine::TransformToGlobal(CTransform& trans)
{
	CPnt pt1 = trans.GetWorldPoint(m_ptStart);
	CPnt pt2 = trans.GetWorldPoint(m_ptEnd);

	CLine Line(pt1, pt2);
	return Line;
}

//
//   取得直线段的斜率k、Y轴截距b和(当直线垂直于X轴时的)X轴截距c。
//   返回值：
//     1 - 直线不垂直于X轴，k和b返回值有意义, c无意义
//     0 - 直线不垂直于X轴，k和b返回值有无意义, c有意义
//
int CLine::GetParam(float* k, float* b, float* c) const
{
	CLine lineXAxis(CPnt(0, 0), CAngle(0), FLT_MAX);

	// 直线不垂直于X轴，计算直线的斜率及在Y轴上的截距
	if (!IsVerticalTo(lineXAxis))
	{
		float K, B;
		
		// 计算斜率
		if (m_angSlant > PI)
			K = tan(!m_angSlant);
		else
			K = tan(m_angSlant);

		// 计算Y轴截距
		B = m_ptStart.y - K * m_ptStart.x;

		if (k != NULL)
			*k = K;

		if (b != NULL)
			*b = B;

		return 1;
	}

	// 直线垂直于X轴，计算直线在X轴上的截距
	else
	{
		if (c != NULL)
			*c = GetMidpoint().x;
		return 0;
	}
}

//
//   计算直线的两个端点中哪个距离指定的点pt更近，并在fDist中返回此近距离。
//   返回值：
//     0 - pt距离起点m_ptStart更近
//     1 - pt距离终点m_ptEnd更近
//
int CLine::FindNearPoint(const CPnt& pt, float* pDist) const
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

//
//   计算本线段到另一线段的投影。
//   返回值：
//       return - 如果有投影线，返回true，否则返回false
//       lnProj - 得到的投影线段
//       fProjDist - 本线段中点到另一条线段(another)的距离
//       fProjLen  - lnProj的长度
//
bool CLine::GetProjection(const CLine& another, CLine& lnProj, float& fProjDist, float& fProjLen) const
{
	float fLambda1, fLambda2;
	CPnt ptFoot1, ptFoot2;

	// 计算本线段的两个端点到another的投影点
	another.DistanceToPoint(false, m_ptStart, &fLambda1, &ptFoot1);
	another.DistanceToPoint(false, m_ptEnd, &fLambda2, &ptFoot2);

	// 如果m_ptStart的投影点落在another的起点侧以外
	if (fLambda1 < 0)
	{
		// 如果m_ptEnd的投影点落在another的起点侧以外，说明投影段不存在
		if (fLambda2 < 0)
			return false;

		// 如果m_ptEnd的投影点落在another内
		else if (fLambda2 <= 1)
			lnProj.Create(another.m_ptStart, ptFoot2);

		// 如果m_ptEnd的投影点落在another的结束点之外
		else
			lnProj = another;
	}

	// 如果m_ptStart的投影点落在another以内
	else if (fLambda1 < 1)
	{
		// 如果m_ptEnd的投影点落在another的起点侧以外
		if (fLambda2 < 0)
			lnProj.Create(another.m_ptStart, ptFoot1);

		// 如果m_ptEnd的投影点落在another内
		else if (fLambda2 <= 1)
			lnProj.Create(ptFoot1, ptFoot2);

		// 如果m_ptEnd的投影点落在another的结束点之外
		else
			lnProj.Create(ptFoot1, another.m_ptEnd);
	}

	// 如果m_ptStart的投影点落在another的结束点以外
	else
	{
		// 如果m_ptEnd的投影点落在another的起点侧以外
		if (fLambda2 < 0)
			lnProj = another;

		// 如果m_ptEnd的投影点落在another内
		else if (fLambda2 <= 1)
			lnProj.Create(another.m_ptEnd, ptFoot2);

		// 如果m_ptEnd的投影点落在another的结束点之外
		else
			return false;
	}

	fProjDist = another.DistanceToPoint(false, GetMidpoint());
	fProjLen = lnProj.Length();

	return true;
}

//
//   进行坐标正变换。
//
void CLine::Transform(const CFrame& frame)
{
	CPnt pt1 = m_ptStart;
	CPnt pt2 = m_ptEnd;

	// 分别对两个端点进行坐标变换
	pt1.Transform(frame);
	pt2.Transform(frame);

	// 重新生成直线段
	Create(pt1, pt2);
}

//
//   进行坐标逆变换。
//
void CLine::InvTransform(const CFrame& frame)
{
	CPnt pt1 = m_ptStart;
	CPnt pt2 = m_ptEnd;

	// 分别对两个端点进行坐标变换
	pt1.InvTransform(frame);
	pt2.InvTransform(frame);

	// 重新生成直线段
	Create(pt1, pt2);
}

#ifdef _MFC_VER

//
//   在屏幕上绘制此直线。
//
void CLine::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bBigVertex)
{
	CPen pen(PS_SOLID, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPnt pt1(m_ptStart.x, m_ptStart.y);
	CPnt pt2(m_ptEnd.x, m_ptEnd.y);

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
