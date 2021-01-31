//                        - SCP.CPP -
//
//   Implementation of class "CScp", which defines Single Cartesian-
//   Polynomial curves. This kinds of curves serve as lane-change path
//   in this AGVS.
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#include "stdafx.h"

#include <math.h>
#include "Geometry.h"
#include "Tools.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// NOTE: USE THE FOLLOWING DEFINITION TO INCLUDE TRAJECTORY CACULATION !
#define _TRAJ_POINT_

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CScp".

//
//   CScp: The constructor.
//
CScp::CScp(CPnt& ptStart, CPnt& ptEnd, CAngle& angLane)
{
	// Init the curve parameters
	m_ptStart = ptStart;
	m_ptEnd = ptEnd;
	m_angLane = angLane;

	// The local frame - origin at "m_ptStart", slant angle "m_angLane"
	m_Transform = CTransform(m_ptStart, m_angLane);

	// Get the coordinates of the end point in the local frame
	CPnt ptLocalEnd = m_Transform.GetLocalPoint(m_ptEnd);

	// Init "Xe" and "Ye" - "Xe" is the progress variable
	m_fXe = ptLocalEnd.x;
	m_fYe = ptLocalEnd.y;

	// Now, attempts to caculate the shifting angle
	CLine ln(ptStart, ptEnd);
	m_angShift = ln.SlantAngle() - angLane;
}

//
//   GetX: Get the X distance of the curve.
//
float CScp::GetX()
{
	return m_fXe;
}

//
//   SetCurX: Set the current X distance to specify the current point.
//
void CScp::SetCurX(float fX)
{
	// Step 1: Initializations


#if defined _TRAJ_POINT_
	//   Powers of the rate "X/Xe"
	float fRate = fX / m_fXe;
	float fRate_3 = fRate * fRate * fRate;
	float fRate_4 = fRate_3 * fRate;
	float fRate_5 = fRate_4 * fRate;
#endif


	//   Powers of "X"
	float fX_2 = fX * fX;
	float fX_3 = fX_2 * fX;
	float fX_4 = fX_3 * fX;

	//   Powers of "Xe"
	float fXe_3 = (float)pow(m_fXe, 3.0f);
	float fXe_4 = fXe_3 * m_fXe;
	float fXe_5 = fXe_4 * m_fXe;


#if defined _TRAJ_POINT_
	//  Caculate Y(X) at the current point (in local frame)
	float fY = m_fYe * (10*fRate_3 - 15*fRate_4 + 6*fRate_5);
#endif


	//  Caculate the 1st derivative of Y(X) at the current point
	float fY1 = m_fYe * (30*fX_2/fXe_3 - 60*fX_3/fXe_4 + 30*fX_4/fXe_5);

	//  Caculate the 2nd derivative of Y(X) at the current point
	float fY2 = m_fYe * (60*fX/fXe_3 - 180*fX_2/fXe_4 + 120*fX_3/fXe_5);

	//  Caculate the square of the 1st derivative of Y(X)
	float fY1_2 = fY1 * fY1;


#if defined _TRAJ_POINT_
	// Step 2: Obtain the coordinates of the current point
	m_ptLocal.x = fX;
	m_ptLocal.y = fY;

	m_pt = m_Transform.GetWorldPoint(m_ptLocal);
#endif


	// Step 3: Caculate the tangent angle at the current point
	m_angTangent0 = CAngle((float)atan(fY1));
	m_angTangent = m_angLane + m_angTangent0;

	// Step 4: Caculate the curvature of the curve at the current point
	m_fCurvature = fY2 / (float)pow(1 + fY1_2, 1.5f);
}

//
//   TrajFun: The trajectory generation function.
//
CPnt CScp::TrajFun()
{
	return m_pt;
}

//
//   TangentFun: The tangent angle generation function (IN LOCAL FRAME!).
//
CAngle CScp::TangentFun(bool bWorldFrame)
{
	if (bWorldFrame)
		return m_angTangent;
	else
		return m_angTangent0;
}

//
// The curvature generation function
//
float CScp::CurvatureFun()
{
	return m_fCurvature;
}

//
//   ShiftAngle: Get the curve's shift angle with respect to the lane.
//
CAngle& CScp::ShiftAngle()
{
	return m_angShift;
}

float CScp::NewtonRoot(float fXk, float fY)
{
	float fXk1 = fXk;

	do {
		fXk = fXk1;
		fXk1 = fXk - ScpFun(fXk, fY)/ScpFun_(fXk);
	} while (fabs(fXk1 - fXk) > 0.001f);
	return fXk1;
}

float CScp::ScpFun(float fXk, float fY)
{
	float k = fY/m_fYe;
	float t = fXk/m_fXe;
	float t3 = t*t*t;
	float t4 = t3 * t;
	float t5 = t4 * t;
	float fResult = 6*t5 - 15*t4 + 10*t3 - k;
	return fResult;
}

float CScp::ScpFun_(float fXk)
{
	float t = fXk/m_fXe;
	float t2 = t*t;
	float t3 = t2*t;
	float t4 = t3*t;
	float fResult = 30*t4 - 60*t3 + 30*t2;
	return fResult;
}

//
//   Find the X coordinate of the reference point.
//
bool CScp::FindRefX(CPnt& pt, float& fRefX, float& fErrX)
{
	CPnt ptLocal = m_Transform.GetLocalPoint(pt);

	if (((m_fXe > 0) && (ptLocal.x < 0)) || ((m_fXe < 0) && (ptLocal.x > 0)))
		fRefX = 0;
	else if (((m_fXe > 0) && (ptLocal.x > m_fXe)) || ((m_fXe < 0) && (ptLocal.x < m_fXe)))
		fRefX = m_fXe;
	else if (((m_fYe > 0) && (ptLocal.y < 0)) || ((m_fYe < 0) && (ptLocal.y > 0)))
		fRefX = 0;
	else if (((m_fYe > 0) && (ptLocal.y > m_fYe)) || ((m_fYe < 0) && (ptLocal.y < m_fYe)))
		fRefX = m_fXe;
	else if (fabs(ptLocal.y) < 0.0001f)
		fRefX = 0;
	else if (fabs(ptLocal.y - m_fYe) < 0.0001f)
		fRefX = m_fXe;
	else
		fRefX = NewtonRoot(ptLocal.x, ptLocal.y);

	fErrX = ptLocal.x - fRefX;

	return true;
}

//
//   Find the error in the Y direction.
//
bool CScp::FindErrY(CPnt& pt, float& fErrY)
{
	CPnt pt1 = m_Transform.GetLocalPoint(pt);
	float fAbsXe = (float)fabs(m_fXe);
	pt1.x = Limit(pt1.x, fAbsXe);
	SetCurX(pt1.x);
	fErrY = pt1.y - m_ptLocal.y;
	return true;
}
