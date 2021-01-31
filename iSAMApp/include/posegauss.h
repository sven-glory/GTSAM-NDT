#ifndef POSEGAUSS_H
#define POSEGAUSS_H

#include "Geometry.h"
#include "matrix.h"

class PoseGauss : public CPosture
{
public:
	CPosture m_pstMean;
	Matrix3  m_Sigma;

public:
	PoseGauss()
	{
		m_pstMean.x = 0;
		m_pstMean.y = 0;
		m_pstMean.fThita = 0;

		m_Sigma.Set(0.0, 0.0, 0.0,
			           0.0, 0.0, 0.0,
			           0.0, 0.0, 0.0);
	}

	// Adds relpos to pos.  Result is stored in *result. Adding is done component-wise
	PoseGauss operator + (PoseGauss& relpos);

	// Returns Mahalanobis distance from pose p to pose prob pg
	float MahanalobisDistance(const CPosture *p);

	// Returns quality of position uncertainty.
	// A small value means low uncertainty, a large one, large uncertainty
	float Uncertainty() {return m_Sigma.d[2][2];}

	// Returns TRUE if all diagonal variances fall below a certain threshold
	BOOL AllDOFFixed();

	// Makes sure that the error covariance contains at least some minimal error
	void EnsureMinError();

	// Transforms absolute pose prob p1 to relative coordinate system of pg0 
	// including the transformation of the error covariance matrix.
	void TransformToLocal(PoseGauss *pg1, PoseGauss *result);

	// Transforms relative pose prob pg1 to absolute pose prog using the
	// coordinate system of pg0.  The error covariance is also transformed.
	void TransformToGlobal(PoseGauss *pg1, PoseGauss *result);

	// Computes negative of given pose gauss
	void Inverse();

	// Rotates pose gauss by given angle with rotation center (cx, cy)
	void RotatePos(float angle, float cx, float cy);

	// Updates pose prob stored in pg by moving the robot r mm
	// with heading h (relative to the robots current heading)
	// and then rotating it by a.  For d and a an gaussian error is assumed.
	// sigma_dist2, sigma_rot2 and sigma_drift2 are the corresponding 
	// sigma values per 1000mm, per 2PI and per 1000mm.
	void DeadReckoning(float h, float r, float a, float sigma_dist2, 
		float sigma_rot2, float sigma_drift2);

	// Same as PosDeadReckoning but uses start and end pose as parameters
	void DeadReckoningPoses(CPosture *from, CPosture *to, float sigma_rr, float sigma_aa, float sigma_dd);


};

extern PoseGauss PoseGaussZero;

float PoseInverseMahanalobisDistance(const CPosture *mean, 
	const Matrix3 *inv, const CPosture *p);
/*
** Like PoseGaussMahanalobisDistance but takes the inverse of
** the co-variance as second argument.  Use this function if
** you need to compute many mahanalobis distances using the
** same co-variance.  It's much faster!
*/

bool PoseGaussKalmanFuse(PoseGauss *pg1, PoseGauss *pg2, 
	PoseGauss *pg3);
/*
** Kalman fuses pose probs p1 and p2 to p3.
*/

bool PoseInverseKalmanFuse(CPosture *p1, Matrix3 *inv1,
	CPosture *p2, Matrix3 *inv2, PoseGauss *result);
/*
** Same as KalmanFuse but takes inverse matrices as input.
*/

bool PoseGaussKalmanWithScanMatch(PoseGauss *newpg, 
	PoseGauss *oldpg, PoseGauss *refpg,
	PoseGauss *relpg_from_scanmatch);
/*
** Kalman-filters dead-reckoning information in oldpg 
** with result from scan matching of scan taken at refpg 
** with scan taken at oldpg.  Returns result in newpg.
** Returns TRUE on success, FALSE on failure.
*/

void PoseGaussUpdate(PoseGauss *current, 
	PoseGauss *oldpg, PoseGauss *newpg,
	float sigma_rr, float sigma_aa, float sigma_dd);
/*
** Updates PoseGauss current.
** PoseGauss oldpg is an old pose estimation.
** PoseGauss newpg is the external (e.g. derived by scan matching) 
** estimation.  The information of oldpg has already been
** included in newpg (e.g. by Kalman-filtering).
** PoseGaussUpdate updates PoseGauss current according to the differences 
** of oldpg and newpg.
*/

#endif


