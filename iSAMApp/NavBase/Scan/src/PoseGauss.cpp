#include "stdafx.h"
#include "misc.h"
#include "posegauss.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


PoseGauss PoseGaussZero;

//
//   Adds relpos to pos.  Result is stored in *result. Adding is done component-wise.
//
PoseGauss PoseGauss::operator + (PoseGauss& relpg)
{
	PoseGauss result;

	result.m_pstMean.x = m_pstMean.x + relpg.m_pstMean.x;
	result.m_pstMean.y = m_pstMean.y + relpg.m_pstMean.y;
	result.m_pstMean.fThita = NormAngle(m_pstMean.fThita + relpg.m_pstMean.fThita);

	result.m_Sigma = m_Sigma + relpg.m_Sigma;

	return result;
}

float PoseInverseMahanalobisDistance(const CPosture *mean, 
	const Matrix3 *inv, const CPosture *p)
{
	Vector3 v, w;
	float d;

	if (mean == NULL || inv == NULL || p == NULL)
		return(HUGE_VAL);

	v.Set(mean->x - p->x, mean->y - p->y, NormAngle(mean->fThita - p->fThita));
	w = (Matrix3&)(*inv) * v;
	d = v * w;
	return d;
}

//
//   Returns Mahalanobis distance from pose p to pose prob pg.
//
float PoseGauss::MahanalobisDistance(const CPosture *p)
{
	Matrix3 inv;

	inv = m_Sigma;
	if (!inv.Inverse())
		return(HUGE_VAL);

	return PoseInverseMahanalobisDistance(&m_pstMean, &inv, p);
}

//
//   Returns TRUE if all diagonal variances fall below a certain threshold.
//
BOOL PoseGauss::AllDOFFixed()
{
	Matrix3 *sigma = &m_Sigma;
	float sxx = sigma->d[0][0];
	float syy = sigma->d[1][1];
	float saa = sigma->d[2][2];
	static const float ppmax = 100.0 * 100.0; 
	static const float aamax = DEG2RAD(5.0) * DEG2RAD(5.0);

	if (sxx < ppmax && syy < ppmax && saa < aamax)
		return TRUE;
	else
		return FALSE;
}

//
//   Makes sure that the error covariance contains at least some minimal error.
//
void PoseGauss::EnsureMinError()
{
	static const float sx = 20.0, sy = 20.0, sa = DEG2RAD(0.5);

	m_Sigma.d[0][0] = MAX(sx*sx, m_Sigma.d[0][0]);
	m_Sigma.d[1][1] = MAX(sy*sy, m_Sigma.d[1][1]);
	m_Sigma.d[2][2] = MAX(sa*sa, m_Sigma.d[2][2]);
}

#if 0
void PoseGauss::TransformToLocal(PoseGauss *pg1, PoseGauss *result)
{
	if (pg1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R, S, tmp, M1, M2;
		float sina = sin(m_pstMean.fThita), cosa = cos(m_pstMean.fThita);

		v0.Set(m_pstMean.x, m_pstMean.y, m_pstMean.fThita);
		v1.Set(pg1->m_pstMean.x, pg1->m_pstMean.y, pg1->m_pstMean.fThita);
		R.Set(
			cosa,   sina,   0.0,
			-sina,  cosa,   0.0,
			0.0,	0.0,	1.0);

		v0 *= -1.0;
		v0 += v1;
		v2 = R * v0;
		result->m_pstMean.x = v2.d[0];
		result->m_pstMean.y = v2.d[1];
		result->m_pstMean.fThita = NormAngle(v2.d[2]);

		S.Set(
			-cosa,  -sina,  -sina * v0.d[0] + cosa * v0.d[1],
			sina,   -cosa,  -cosa * v0.d[0] - sina * v0.d[1],
			0.0,	0.0,	-1.0);

		Matrix3Mult(&R, &pg1->m_Sigma, &tmp);
		R.Transpose();
		Matrix3Mult(&tmp, &R, &M1);
		Matrix3Mult(&S, &m_Sigma, &tmp);
		S.Transpose();
		Matrix3Mult(&tmp, &S, &M2);
		result->m_Sigma = M1 + M2;
		result->m_Sigma.Magic();
	}
}
#else

//
//   Transforms absolute pose prob p1 to relative coordinate system of pg0 
//   including the transformation of the error covariance matrix.
//
void PoseGauss::TransformToLocal(PoseGauss *pg1, PoseGauss *result)
{
	if (pg1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R, S, tmp, M1, M2;
		float sina = sin(m_pstMean.fThita), cosa = cos(m_pstMean.fThita);

		v0.Set(m_pstMean.x, m_pstMean.y, m_pstMean.fThita);
		v1.Set(pg1->m_pstMean.x, pg1->m_pstMean.y, pg1->m_pstMean.fThita);
		R.Set(
			cosa,   sina,   0.0,
			-sina,  cosa,   0.0,
			0.0,	0.0,	1.0);

		v0 *= -1.0;
		v0 += v1;
		v2 = R * v0;
		result->m_pstMean.x = v2.d[0];
		result->m_pstMean.y = v2.d[1];
		result->m_pstMean.fThita = NormAngle(v2.d[2]);

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
#endif

//
//   Transforms relative pose prob pg1 to absolute pose prog using the
//   coordinate system of pg0.  The error covariance is also transformed.
//
void PoseGauss::TransformToGlobal(PoseGauss *pg1, PoseGauss *result)
{
	if (pg1 != NULL && result != NULL)
	{
		Vector3 v0, v1, v2;
		Matrix3 R, S, tmp, M1, M2;
		float sina = sin(m_pstMean.fThita), cosa = cos(m_pstMean.fThita);

		v0.Set(m_pstMean.x, m_pstMean.y, m_pstMean.fThita);
		v1.Set(pg1->m_pstMean.x, pg1->m_pstMean.y, pg1->m_pstMean.fThita);
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
		result->m_pstMean.x = v2.d[0];
		result->m_pstMean.y = v2.d[1];
		result->m_pstMean.fThita = NormAngle(v2.d[2]);

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
void PoseGauss::Inverse()
{
	PoseGauss pg0 = *this, pg1 = PoseGaussZero;

	pg0.m_Sigma = Matrix3Zero;
	pg1.m_Sigma = m_Sigma;
	pg0.TransformToLocal(&pg1, this);
}

//
//   Rotates pose gauss by given angle with rotation center (cx, cy).
//
void PoseGauss::RotatePos(float angle, float cx, float cy)
{
	float sina = sin(angle), cosa = cos(angle);
	float dx = m_pstMean.x - cx, dy = m_pstMean.y - cy;
	Matrix3 Rot, tmp;

	m_pstMean.x = cx + dx * cosa - dy * sina;
	m_pstMean.y = cy + dx * sina + dy * cosa;
	m_pstMean.fThita += angle;

	Rot.Set(
		   cosa, -sina, 0.0,
			sina, cosa, 0.0,
			0.0, 0.0, 1.0);

	tmp = Rot * m_Sigma;
	Rot.Transpose();
	m_Sigma = tmp * Rot;
}

//
// Updates pose prob stored in pg by moving the robot r mm
// with heading h (relative to the robots current heading)
// and then rotating it by a.  For d and a an gaussian error is assumed.
// sigma_dist2, sigma_rot2 and sigma_drift2 are the corresponding 
// sigma values per 1000mm, per 2PI and per 1000mm.
//
void PoseGauss::DeadReckoning(float h, float r, float a,
										float sigma_rr, float sigma_aa, float sigma_dd)
{
	float sina, cosa;
	Matrix3 sigmaX, sigmaU, dFx, dFu, tmp;

	sina = sin(m_pstMean.fThita + h);
	cosa = cos(m_pstMean.fThita + h);
	sigmaX = m_Sigma;

	/* calculate new PoseGauss */
	m_pstMean.x += r * cosa;
	m_pstMean.y += r * sina;
	m_pstMean.fThita = NormAngle(m_pstMean.fThita + a);

	/* calculate dFx and dFu */
	dFx.Set(
		1.0,	0.0,	-r * sina,
		0.0,	1.0,	 r * cosa,
		0.0,	0.0,	1.0);

	dFu.Set(
		cosa,   0.0,	-r * sina,
		sina,   0.0,	r * cosa,
		0.0,	1.0,	1.0);

	/* calculate sigmaU */
	sigmaU.Set(
		   sigma_rr * fabs(r),   0.0,	0.0,
		   0.0,	sigma_aa * fabs(a) + sigma_dd * fabs(r), 0.0,
		   0.0,	0.0,	0.0);

	/* calculate new covariance */
	tmp = dFx * sigmaX;
	dFx.Transpose();
	sigmaX = tmp * dFx;
	tmp = dFu * sigmaU;
	dFu.Transpose();
	sigmaU = tmp * dFu;
	m_Sigma = sigmaX + sigmaU;
}

//
//   Same as PosDeadReckoning but uses start and end pose as parameters.
//
void PoseGauss::DeadReckoningPoses(CPosture *p1, CPosture *p2, float sigma_rr, 
											  float sigma_aa, float sigma_dd)
{
	if (p1 != NULL && p2 != NULL)
	{
		float dx = p2->x - p1->x;
		float dy = p2->y - p1->y;
		float da = NormAngle(p2->fThita - p1->fThita);
		float h = atan2(dy, dx) - p1->fThita;
		float d = _hypot(dx, dy);

		DeadReckoning(h, d, da, sigma_rr, sigma_aa, sigma_dd);
	}
}
	
bool PoseInverseKalmanFuse(CPosture *p1, Matrix3 *inv1,
	CPosture *p2, Matrix3 *inv2, PoseGauss *result)
{
	Matrix3 m;
	Vector3 v1, v2, v3, t1, t2, t3;
	float a;

	if (!p1 || !inv1 || !p2 || !inv2 || !result)
		return FALSE;

	a = NormAngle(p1->fThita);
	v1.Set(p1->x, p1->y, 0.0);
	v2.Set(p2->x, p2->y, NormAngle(p2->fThita - a));

	if (fabs(v2.d[2]) > DEG2RAD(30.0)) 
	{
		fprintf(stderr, 
			"PoseInverseKalmanFuse: incompatible observations!\n");
		fprintf(stderr, "(%.1f, %.1f, %.1f deg) and (%.1f, %.1f, %.1f deg)\n",
			v1.d[0], v1.d[1], RAD2DEG(v1.d[2] + a),
			v2.d[0], v2.d[1], RAD2DEG(v2.d[2] + a));
	}

	m = *inv1 + *inv2;

	if (!m.Inverse()) 
	{
		fprintf(stderr, "PoseInverseKalmanFuse: problem inverting m3!\n");
		return FALSE;
	}

	t1 = *inv1 * v1;
	t2 = *inv2 * v2;
	t3 = t1 + t2;
	v3 = m * t3;
	
	result->m_pstMean.x = v3.d[0];
	result->m_pstMean.y = v3.d[1];
	result->m_pstMean.fThita = NormAngle(v3.d[2] + a);
	result->m_Sigma = m;

	return TRUE;
}

#if 0
bool PoseGaussKalmanFuse(PoseGauss *pg1, PoseGauss *pg2,
		PoseGauss *pg3)
{
	Matrix3 m1, m2, m3;
	Vector3 v1, v2, v3, t1, t2, t3;
	float a, det1, det2;

	if(pg1 == NULL || pg2 == NULL || pg3 == NULL)
	return(FALSE);

	m1 = pg1->m_Sigma;
	m2 = pg2->m_Sigma;
	m1.Magic();	  /* Zero out angle-coupled elements */
	m2.Magic();

	det1 = m1.Determinante();
	det2 = m2.Determinante();

	if (NEAR_ZERO(det1)) {
	*pg3 = *pg1;			// pg1 is very accurate
	return TRUE;
	} else if (NEAR_ZERO(det2)) {
	*pg3 = *pg2;			// pg2 is very accurate
	return TRUE;
	}

	a = NormAngle(pg1->m_pstMean.fThita);
	v1.Set(pg1->m_pstMean.x, pg1->m_pstMean.y, 
	NormAngle(pg1->m_pstMean.fThita - a));
	v2.Set(pg2->m_pstMean.x, pg2->m_pstMean.y, 
	NormAngle(pg2->m_pstMean.fThita - a));

	if(fabs(NormAngle(pg2->m_pstMean.fThita - pg1->m_pstMean.fThita)) > DEG2RAD(30.0))
	{
	fprintf(stderr, 
		"PoseGaussKalmanFuse: incompatible observations!\n");
	fprintf(stderr, "(%.1f, %.1f, %.1f deg) and (%.1f, %.1f, %.1f deg)\n",
		v1.d[0], v1.d[1], RAD2DEG(v1.d[2] + a),
		v2.d[0], v2.d[1], RAD2DEG(v2.d[2] + a));
	}

	if (!m1.Inverse())
	{
	fprintf(stderr, "PoseGaussKalmanFuse: problem inverting m1!\n");
	*pg3 = *pg2;		// something wrong with pg1
	return(TRUE);
	}
	if (!m2.Inverse())
	{
	fprintf(stderr, "PoseGaussKalmanFuse: problem inverting m2!\n");
	*pg3 = *pg1;		// something wrong with pg2
	return(TRUE);
	}
	m3 = m1 + m2;
	m3.Magic();
	if (!m3.Inverse())
	{
	fprintf(stderr, "PoseGaussKalmanFuse: problem inverting m3!\n");
	*pg3 = (det1 < det2)? *pg1 : *pg2;  // return better estimate
	return(TRUE);
	}
	m3.Magic();

	t1 = m1 * v1;
	t2 = m2 * v2;
	t3 = t1 + t2;
	v3 = m3 * t3;
	
	pg3->m_pstMean.x = v3.d[0];
	pg3->m_pstMean.y = v3.d[1];
	pg3->m_pstMean.fThita = NormAngle(v3.d[2] + a);
	pg3->m_Sigma = m3;
	return(TRUE);
}
#else
bool PoseGaussKalmanFuse(PoseGauss *pg1, PoseGauss *pg2,
		PoseGauss *pg3)
{
	Matrix3 m1, m2;
	float det1, det2;

	if (!pg1 || !pg2 || !pg3)
		return FALSE;

	m1 = pg1->m_Sigma;
	m2 = pg2->m_Sigma;
	m1.Magic();	  /* Zero out angle-coupled elements */
	m2.Magic();

	det1 = m1.Determinante();
	det2 = m2.Determinante();

	if (NEAR_ZERO(det1)) 
	{
		*pg3 = *pg1;			// pg1 is very accurate
		return TRUE;
	}
	else if (NEAR_ZERO(det2)) 
	{
		*pg3 = *pg2;			// pg2 is very accurate
		return TRUE;
	}

	if (!m1.Inverse()) 
	{
		fprintf(stderr, "PoseGaussKalmanFuse: problem inverting m1!\n");
		*pg3 = *pg2;			// something wrong with pg1
		return TRUE;
	}

	if (!m2.Inverse()) 
	{
		fprintf(stderr, "PoseGaussKalmanFuse: problem inverting m2!\n");
		*pg3 = *pg1;			// something wrong with pg2
		return TRUE;
	}

	return PoseInverseKalmanFuse(&pg1->m_pstMean, &m1, &pg2->m_pstMean, &m2, pg3);
}
#endif

bool PoseGaussKalmanWithScanMatch(PoseGauss *newPoseGauss, 
	PoseGauss *old, PoseGauss *ref, PoseGauss *match) 
{
	PoseGauss pg, result;

	if (newPoseGauss == NULL || old == NULL || ref == NULL || match == NULL)
		return(FALSE);

	ref->TransformToGlobal(match, &pg);

	if (!PoseGaussKalmanFuse(old, &pg, &result))
		return FALSE;

	*newPoseGauss = result;

	return TRUE;
}

void PoseGaussUpdate(PoseGauss *cur, 
	PoseGauss *old, PoseGauss *newPoseGauss,
	float sigma_rr, float sigma_aa, float sigma_dd)
{
	CPosture curp, oldp;

	if (cur == NULL || old == NULL || newPoseGauss == NULL)
		return;

	curp = cur->m_pstMean;
	oldp = old->m_pstMean;
	*cur = *newPoseGauss;
	cur->DeadReckoningPoses(&oldp, &curp, sigma_rr, sigma_aa, sigma_dd);
}

