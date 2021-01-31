//                         - SPP.CPP -
//
//   Implementation of class "CSpp", which depicts Single Polar-
//   Polynomial(SPP) curve.
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
//   Implementation of class "CSpp".

//
//   CSpp: The constructor.
//
CSpp::CSpp(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir) :
CArc(ptCenter, ptStart, ptEnd, TurnDir)
{
}

//
//   SetCurAngle: Set the current turn angle to specified a point.
//
void CSpp::SetCurAngle(float fPhi)
{
	// If the turn direction is "CLOCKWISE", the "Phi" angle should 
	//  be modified
	if (m_TurnDir == CLOCKWISE)
		fPhi = m_fTurnAngle - fPhi;

	// Initializations
	float fPhi_2 = fPhi * fPhi;
	float fPhi_3 = fPhi_2 * fPhi;
	float fPhi_4 = fPhi_3 * fPhi;
	float fThita_2 = m_fTurnAngle * m_fTurnAngle;

	// Caculate the polar radius of the current point
	float fR = m_fRadius * (1 + fPhi_2/2 - fPhi_3/m_fTurnAngle +
									fPhi_4/(2*fThita_2));

	// Keep the current radius
	m_fCurRadius = fR;

	// Caculate the 1st derivative of the polar radius
	float fR1 = m_fRadius * (fPhi - 3*fPhi_2/m_fTurnAngle + 2*fPhi_3/fThita_2);

	// Caculate the tangent angle at the current point
	m_angTangent = m_angStart + CAngle((float)(PI/2+fPhi - atan(fR1/fR)));


#if defined _CURVATURE_
	
	// Caculate the 2nd derivative of the polar radius
	float fR2 = m_fRadius * (1 - 6*fPhi/m_fTurnAngle + 6*fPhi_2/fThita_2);

	// The polar radius to the 2nd power, namely
	float fR_2 = fR * fR;

	// The derivative of the polar radius to the 2nd power, namely
	float fR1_2 = fR1 * fR1;

	// Caculate the curvature at the current point
	m_fCurvature = (float)((fR_2 + 2*fR1_2 - fR * fR2) / pow((fR_2 + fR1_2), 1.5f));

#endif


	// Caculate the current point
	CPnt ptTemp((float)(fR * cos(fPhi)), (float)(fR * sin(fPhi)));     // Local point
	m_pt = m_Transform.GetWorldPoint(ptTemp);        // World point

	// If the turn direction is "CLOCKWISE", make some adjustments
	if (m_TurnDir == CLOCKWISE)
	{
		m_angTangent = !m_angTangent;


#if defined _CURVATURE_
		m_fCurvature = -m_fCurvature;
#endif


	}
}
