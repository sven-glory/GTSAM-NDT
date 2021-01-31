#include "stdafx.h"
#include "PostureGauss.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


CPostureGauss PoseGaussZero;

//
//   Adds relpos to pos.  Result is stored in *result. Adding is done component-wise.
//
CPostureGauss CPostureGauss::operator + (CPostureGauss& relpg)
{
	CPostureGauss result;

	result.x = x + relpg.x;
	result.y = y + relpg.y;
	result.fThita = CAngle::NormAngle2(fThita + relpg.fThita);

	result.m_Sigma = m_Sigma + relpg.m_Sigma;

	return result;
}

//
//   Transforms absolute pose prob p1 to relative coordinate system of pg0 
//   including the transformation of the error covariance matrix.
//
void CPostureGauss::TransformToLocal(CPostureGauss *pg1, CPostureGauss *result)
{
	if (pg1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R, S, tmp, M1, M2;
		float sina = sin(fThita), cosa = cos(fThita);

		v0.Set(x, y, fThita);
		v1.Set(pg1->x, pg1->y, pg1->fThita);
		R.Set(
			cosa,   sina,   0.0,
			-sina,  cosa,   0.0,
			0.0,	0.0,	1.0);

		v0 *= -1.0;
		v0 += v1;
		v2 = R * v0;
		result->x = v2.d[0];
		result->y = v2.d[1];
		result->fThita = CAngle::NormAngle2(v2.d[2]);

		S.Set(
			1.0,	0.0,	-sina * v2.d[0] - cosa * v2.d[1],
			0.0,	1.0,	cosa * v2.d[0] - sina * v2.d[1],
			0.0,	0.0,	1.0);

		tmp = S * m_Sigma;
		S.Transpose();
		M1 = tmp * S;
		M1 *= -1.0;
		M2 = pg1->m_Sigma + M1;
		tmp = R * M2;
		R.Transpose();
		result->m_Sigma = tmp * R;
		result->m_Sigma.Magic();
	}
}

//
//   Transforms relative pose prob pg1 to absolute pose prog using the
//   coordinate system of pg0.  The error covariance is also transformed.
//
void CPostureGauss::TransformToGlobal(CPostureGauss *pg1, CPostureGauss *result)
{
	if (pg1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R, S, tmp, M1, M2;
		float sina = sin(fThita), cosa = cos(fThita);

		v0.Set(x, y, fThita);
		v1.Set(pg1->x, pg1->y, pg1->fThita);
		R.Set(
			cosa,   -sina,  0.0,
			sina,   cosa,   0.0,
			0.0,	0.0,	1.0);
		S.Set(
			1.0,	0.0,	-sina * v1.d[0] - cosa * v1.d[1],
			0.0,	1.0,	cosa * v1.d[0] - sina * v1.d[1],
			0.0,	0.0,	1.0);

		v2 = R * v1;
		v2 += v0;
		result->x = v2.d[0];
		result->y = v2.d[1];
		result->fThita = CAngle::NormAngle2(v2.d[2]);

		tmp = R * pg1->m_Sigma;
		R.Transpose();
		M1 = tmp * R;
		tmp = S * m_Sigma;
		S.Transpose();
		M2 = tmp * S;
		result->m_Sigma = M1 + M2;
		result->m_Sigma.Magic();
	}
}

//
//   Computes negative of given pose gauss.
//
void CPostureGauss::Inverse()
{
	CPostureGauss pg0 = *this, pg1 = PoseGaussZero;

	pg0.m_Sigma = Matrix3Zero;
	pg1.m_Sigma = m_Sigma;
	pg0.TransformToLocal(&pg1, this);
}

//
//   Rotates pose gauss by given angle with rotation center (cx, cy).
//
void CPostureGauss::RotatePos(float angle, float cx, float cy)
{
	float sina = sin(angle), cosa = cos(angle);
	float dx = x - cx, dy = y - cy;
	Matrix3 Rot, tmp;

	x = cx + dx * cosa - dy * sina;
	y = cy + dx * sina + dy * cosa;
	fThita += angle;

	Rot.Set(
		   cosa, -sina, 0.0,
			sina, cosa, 0.0,
			0.0, 0.0, 1.0);

	tmp = Rot * m_Sigma;
	Rot.Transpose();
	m_Sigma = tmp * Rot;
}
