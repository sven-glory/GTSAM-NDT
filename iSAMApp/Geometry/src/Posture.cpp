#include "stdafx.h"
#include <math.h>
#include "Geometry.h"
#include "Frame.h"
#include "ScrnRef.h"
#include "vector.h"
#include "Matrix.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   将点绕指定的中心点进行旋转。
//
void CPosture::Rotate(float fAng, float fCx, float fCy)
{
	CPnt::Rotate(fAng, fCx, fCy);
	fThita = CAngle::NormAngle(fThita + fAng);
}

//
//   将姿态绕指定的中心点进行旋转。
//
void CPosture::Rotate(float fAng, CPnt ptCenter)
{
	CPnt::Rotate(fAng, ptCenter);
	fThita = CAngle::NormAngle(fThita + fAng);
}

void CPosture::TransformToLocal(const CPosture *p1, CPosture *result)
{
	if (p1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R;
		float sina = sin(fThita), cosa = cos(fThita);

		v0.Set(x, y, fThita);
		v1.Set(p1->x, p1->y, p1->fThita);

		R.Set(
			cosa, sina, 0.0,
			-sina, cosa, 0.0,
			0.0, 0.0, 1.0);

		v0 *= -1.0;
		v0 += v1;
		v2 = R * v0;
		result->x = v2.d[0];
		result->y = v2.d[1];
		result->fThita = NormAngle(v2.d[2]);
	}
}

void CPosture::TransformToGlobal(const CPosture *p1, CPosture *result)
{
	if (p1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R;
		float sina = sin(fThita), cosa = cos(fThita);

		v0.Set(x, y, fThita);
		v1.Set(p1->x, p1->y, p1->fThita);
		R.Set(
			cosa, -sina, 0.0,
			sina, cosa, 0.0,
			0.0, 0.0, 1.0);

		v2 = R * v1;
		v2 += v0;
		result->x = v2.d[0];
		result->y = v2.d[1];
		result->fThita = NormAngle((float)v2.d[2]);
	}
}

//
//   计算姿态角到一条射线的夹角。
//
CAngle CPosture::AngleToLine(const CLine& ln) const
{
	return (ln.m_angSlant - fThita);
}

void CPosture::RotatePos(float angle, float cx, float cy)
{
	float sina = (float)sin(angle), cosa = (float)cos(angle);
	float dx = x - cx, dy = y - cy;

	x = cx + dx * cosa - dy * sina;
	y = cy + dx * sina + dy * cosa;
	fThita += angle;
}

//
//   在当前姿态的基础上，根据给定的速度向量，推算出一定时段后的新姿态。
//
CPosture CPosture::Deduce(const vector_velocity& vel, float interval)
{
	float dx = vel.vel_x * interval;
	float dy = vel.vel_y * interval;
	float s = (float)sin(fThita);
	float c = (float)cos(fThita);

	CPosture pstNew;
	pstNew.x = x + dx * c - dy * s;
	pstNew.y = y + dy * c + dx * s;
	pstNew.fThita = fThita + vel.vel_angle * interval;

	return pstNew;
}

//
//   进行坐标正变换。
//
void CPosture::Transform(const CFrame& frame)
{
	CPnt::Transform(frame);
	fThita = CAngle::NormAngle(fThita - frame.fThita);
}

//
//   进行坐标逆变换。
//
void CPosture::InvTransform(const CFrame& frame)
{
	CPnt::InvTransform(frame);
	fThita = CAngle::NormAngle(fThita + frame.fThita);
}

#ifdef _MFC_VER

//#define SCANNER_ARROW_LEN          0.15f
//#define SCANNER_RADIUS             0.04f

//
//   在屏幕上绘制此姿态。
//
void CPosture::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nCircleRadiusMm, int nArrowLenMm, int nWidth)
{
	CPen pen(PS_SOLID, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	// 根据预先设定的物理尺寸，计算箭头长度和圆半径
	int nArrowLen = nArrowLenMm / 1000.0f * ScrnRef.m_fRatio;
	int nRadius = nCircleRadiusMm / 1000.0f * ScrnRef.m_fRatio;

	if (nArrowLen < 10)
		nArrowLen = 10;

	if (nRadius < 4)
		nRadius = 4;

	CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());
	CPoint pntArrowTip;
	pntArrowTip.x = pnt.x + (int)(nArrowLen * cos(fThita));
	pntArrowTip.y = pnt.y - (int)(nArrowLen * sin(fThita));

	CRect r1(pnt.x - nRadius, pnt.y - nRadius, pnt.x + nRadius, pnt.y + nRadius);
	CPoint pntStart(pnt.x + nRadius, pnt.y);
	CPoint pntEnd(pnt.x + nRadius, pnt.y);
	pDC->Arc(&r1, pntStart, pntEnd);

	pDC->MoveTo(pnt);
	pDC->LineTo(pntArrowTip);
	pDC->SelectObject(&pOldPen);
}
#endif

vector_velocity EstimateVel(CPosture pst1, CPosture pst2, float interval)
{
	vector_velocity vel;

	float fThita = pst1.fThita;
	float c = (float)cos(fThita);
	float s = (float)sin(fThita);
	float dx = pst2.x - pst1.x;
	float dy = pst2.y - pst1.y;

	vel.vel_x = (dx * c + dy * s) / interval;
	vel.vel_y = (dy * c - dx * s) / interval;
	vel.vel_angle = (pst2.fThita - pst1.fThita) / interval;

	return vel;
}

CPosture operator + (const CPosture& pst1, const CPosture& pst2)
{
	CPosture pst = pst1;
	pst += pst2;

	return pst;
}

CPosture operator - (const CPosture& pst1, const CPosture& pst2)
{
	CPosture pst = pst1;
	pst -= pst2;

	return pst;
}
