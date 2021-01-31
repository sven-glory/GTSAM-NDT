#include "stdafx.h"
#include <math.h>
#include "Geometry.h"
#include "matrix.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


CPosture PoseZero(0.0f, 0.0f, 0.0f);

///////////////////////////////////////////////////////////////////////////////

//
//   将点绕指定的中心点进行旋转。
//
void CPosture::Rotate(float fAng, float fCx, float fCy)
{
	CPoint2d::Rotate(fAng, fCx, fCy);
	fThita = NormAngle(fThita + fAng);
}

//
//   将姿态绕指定的中心点进行旋转。
//
void CPosture::Rotate(float fAng, CPoint2d ptCenter)
{
	CPoint2d::Rotate(fAng, ptCenter);
	fThita = NormAngle(fThita + fAng);
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
			cosa,	   sina,   0.0,
		   -sina,   cosa,   0.0,
		   0.0,     0.0,    1.0);

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
			cosa,	   -sina,  0.0,
			sina,    cosa,   0.0,
			0.0,     0.0,    1.0);

		v2 = R * v1;
		v2 += v0;
		result->x = v2.d[0];
		result->y = v2.d[1];
		result->fThita = NormAngle(v2.d[2]);
	}
}

void CPosture::RotatePos(float angle, float cx, float cy)
{
	float sina = sin(angle), cosa = cos(angle);
	float dx = x - cx, dy = y - cy;

	x = cx + dx * cosa - dy * sina;
	y = cy + dx * sina + dy * cosa;
	fThita += angle;
}

//
//   在当前姿态的基础上，根据给定的速度向量，推算出一定时段后的新姿态。
//
CPosture CPosture::Deduce(vector_velocity& vel, float interval)
{
	float dx = vel.vel_x * interval;
	float dy = vel.vel_y * interval;
	float s = sin(fThita);
	float c = cos(fThita);

	CPosture pstNew;
	pstNew.x = x + dx * c - dy * s;
	pstNew.y = y + dy * c + dx * s;
	pstNew.fThita = fThita + vel.vel_angle * interval;

	return pstNew;
}


vector_velocity EstimateVel(CPosture pst1, CPosture pst2, float interval)
{
	vector_velocity vel;

	float fThita = pst1.fThita;
	float c = cos(fThita);
	float s = sin(fThita);
	float dx = pst2.x - pst1.x;
	float dy = pst2.y - pst1.y;

	vel.vel_x = (dx * c + dy * s) / interval;
	vel.vel_y = (dy * c - dx * s) / interval;
	vel.vel_angle = (pst2.fThita - pst1.fThita) / interval;

	return vel;
}

CPosture operator + (CPosture& pst1, CPosture& pst2)
{
	CPosture pst = pst1;
	pst += pst2;

	return pst;
}

CPosture operator - (CPosture& pst1, CPosture& pst2)
{
	CPosture pst = pst1;
	pst -= pst2;

	return pst;
}
