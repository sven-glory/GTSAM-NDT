//                          - SPLINE.CPP -
//
//   Implementation of class "CSpline", which depicts piecewise
//   polynomial curve or spline curve.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 21
//

#include "stdafx.h"
#include <math.h>
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// NOTE: USE THE FOLLOWING DEFINITION TO INCLUDE CURAVTURE CACULATION !
#define _CURVATURE_

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSpline".

//
//   CSpline: The constructor.
//
CSpline::CSpline(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir) :
CSpp(ptCenter, ptStart, ptEnd, TurnDir)
{
	// Decide "Rb" - in this program, "Rb" is 108% of arc radius
	m_fRb = 1.08f * m_fRadius;

	// Caculate the angle "Beita"
	m_fBeita = (float)sqrt(10.0f * (m_fRb / m_fRadius - 1));
}

//
//   SetCurAngle: Set the current turn angle to specified a point.
//
void CSpline::SetCurAngle(float fPhi)
{
	// If the turn direction is "CLOCKWISE", the "Phi" angle should 
	//  be modified
	if (m_TurnDir == CLOCKWISE)
		fPhi = m_fTurnAngle - fPhi;

	if (fPhi < m_fBeita)
		Solve1(fPhi);

	else if (fPhi < m_fTurnAngle - m_fBeita)
	{
		// Step 2: Caculate the tangent angle at the current point
		m_angTangent = m_angStart + CAngle(PI/2 + fPhi);

#if defined _CURVATURE_
		// Step 3: Caculate the curvature at the current point
		m_fCurvature = 1 / m_fRb;
#endif

		// Step 4: Caculate the current point
		CPnt ptTemp((float)(m_fRb * cos(fPhi)), (float)(m_fRb * sin(fPhi)));  // Local point
		m_pt = m_Transform.GetWorldPoint(ptTemp);           // World point
	}

	else
		Solve2(fPhi);

	// If the turn direction is "CLOCKWISE", make some adjustments
	if (m_TurnDir == CLOCKWISE)
	{
		m_angTangent = !m_angTangent;

#if defined _CURVATURE_
		m_fCurvature = -m_fCurvature;
#endif

	}
}

//////////////////////////////////////////////////////////////////////////////
//   Two helper functions.

//
//   Solve1: A helper function for function "SetProgress".
//
void CSpline::Solve1(float fPhi)
{
	// Step 1: Initializations

	//   The powers of "Phi"
	float fPhi_2 = fPhi * fPhi;
	float fPhi_3 = fPhi_2 * fPhi;
	float fPhi_4 = fPhi_3 * fPhi;
	float fPhi_5 = fPhi_4 * fPhi;

	//   The powers of "Beita"
	float fBeita_2 = m_fBeita * m_fBeita;
	float fBeita_3 = fBeita_2 * m_fBeita;

	//   Caculate the polar radius of the current point
	float fR = m_fRadius * (1 + fPhi_2/2 - fPhi_3/(2*m_fBeita) +
				  fPhi_5 / (10 * fBeita_3));

	// Keep the current radius
	m_fCurRadius = fR;

	//   Caculate the 1st derivative of polar radius at the current point
	float fR1 = m_fRadius * (fPhi - 1.5f * fPhi_2 / m_fBeita +
					0.5f * fPhi_4 / fBeita_3);

	// Step 2: Caculate the current point
	CPnt ptTemp((float)(fR * cos(fPhi)), (float)(fR * sin(fPhi)));   // Local point
	m_pt = m_Transform.GetWorldPoint(ptTemp);      // World point

	// Step 3: Caculate the tangent angle at the current point
	m_angTangent = m_angStart + CAngle((float)(PI/2 + fPhi - atan(fR1/fR)));


#if defined _CURVATURE_

	float fR2 = m_fRadius * (1 - 3 * fPhi / m_fBeita + 2 * fPhi_3 / fBeita_3);

	// The polar radius to the 2nd power, namely
	float fR_2 = fR * fR;

	// The derivative of the polar radius to the 2nd power, namely
	float fR1_2 = fR1 * fR1;

	// Caculate the curvature at the current point
	m_fCurvature = (float)((fR_2 + 2*fR1_2 - fR * fR2) / pow((fR_2 + fR1_2), 1.5f));

#endif

}

//
//   Solve2: A helper function for function "SetProgress".
//
void CSpline::Solve2(float fPhi)
{
	// Step 1: Initializations

	//   The powers of "m_fTurnAngle - Phi"
	float fPhi_1 = m_fTurnAngle - fPhi;
	float fPhi_2 = fPhi_1 * fPhi_1;
	float fPhi_3 = fPhi_2 * fPhi_1;
	float fPhi_4 = fPhi_3 * fPhi_1;
	float fPhi_5 = fPhi_4 * fPhi_1;

	//   The powers of "Beita"
	float fBeita_2 = m_fBeita * m_fBeita;
	float fBeita_3 = fBeita_2 * m_fBeita;

	//   Caculate the polar radius of the current point
	float fR = m_fRadius * (1 + fPhi_2/2 - fPhi_3/(2*m_fBeita) +
				  fPhi_5 / (10 * fBeita_3));

	// Keep the current radius
	m_fCurRadius = fR;

	//   Caculate the 1st derivative of polar radius at the current point
	float fR1 = m_fRadius * (-fPhi_1 + 1.5f * fPhi_2 / m_fBeita -
					0.5f * fPhi_4 / fBeita_3);

	// Step 2: Caculate the current point
	CPnt ptTemp((float)(fR * cos(fPhi)), (float)(fR * sin(fPhi)));
	m_pt = m_Transform.GetWorldPoint(ptTemp);

	// Step 3: Caculate the tangent angle at the current point
	m_angTangent = m_angStart + CAngle((float)(PI/2 + fPhi - atan(fR1/fR)));


#if defined _CURVATURE_

	float fR2 = m_fRadius * (1 - 3 * fPhi_1 / m_fBeita + 2 * fPhi_3 / fBeita_3);

	// The polar radius to the 2nd power, namely
	float fR_2 = fR * fR;

	// The derivative of the polar radius to the 2nd power, namely
	float fR1_2 = fR1 * fR1;
	// Step 4: Caculate the curvature at the current point
	m_fCurvature = (float)((fR_2 + 2 * fR1_2 - fR * fR2) / pow((fR_2 + fR1_2), 1.5f));

#endif
}
