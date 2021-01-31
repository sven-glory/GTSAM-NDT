#ifndef POSEGAUSS_H
#define POSEGAUSS_H

#include "Geometry.h"
#include "MatrixLib.h"

class CPostureGauss : public CPosture
{
public:
	Matrix3  m_Sigma;

public:
	CPostureGauss()
	{
		m_Sigma.Set(0.0, 0.0, 0.0,
			           0.0, 0.0, 0.0,
			           0.0, 0.0, 0.0);
	}

	// Adds relpos to pos.  Result is stored in *result. Adding is done component-wise
	CPostureGauss operator + (CPostureGauss& relpos);

	// Returns quality of position uncertainty.
	// A small value means low uncertainty, a large one, large uncertainty
	float Uncertainty() {return m_Sigma.d[2][2];}

	// Transforms absolute pose prob p1 to relative coordinate system of pg0 
	// including the transformation of the error covariance matrix.
	void TransformToLocal(CPostureGauss *pg1, CPostureGauss *result);

	// Transforms relative pose prob pg1 to absolute pose prog using the
	// coordinate system of pg0.  The error covariance is also transformed.
	void TransformToGlobal(CPostureGauss *pg1, CPostureGauss *result);

	// Computes negative of given pose gauss
	void Inverse();

	// Rotates pose gauss by given angle with rotation center (cx, cy)
	void RotatePos(float angle, float cx, float cy);
};

extern CPostureGauss PoseGaussZero;

#endif


