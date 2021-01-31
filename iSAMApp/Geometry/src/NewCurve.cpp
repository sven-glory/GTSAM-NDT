//                           - NEWCURVE.CPP -
//
//   Implementation of class "CNewCurve", which depicts Single Polar-
//   Polynomial(SPP) curve.
//
//   Author: Zhang Lei
//   Date:   2005. 12. 8
//

#include "stdafx.h"
#include <math.h>
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define K1          0.9f
#define K2          0.9f

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CNewCurve".

//
//   CNewCurve: The constructor.
//
CNewCurve::CNewCurve(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir) :
CArc(ptCenter, ptStart, ptEnd, TurnDir)
{
	CAngle angEnd = m_angStart + CAngle(m_fTurnAngle);

	// Decide P0 first
	X0 = ptCenter.x + m_fRadius * cos(m_angStart);
	Y0 = ptCenter.y + m_fRadius * sin(m_angStart);

	// Then decide P3
	X3 = ptCenter.x + m_fRadius * cos(angEnd);
	Y3 = ptCenter.y + m_fRadius * sin(angEnd);

	// P2 and P3 are calculate according to P0 and P1
	float h = (float)(m_fRadius * tan(m_fTurnAngle/2));

	X1 = X0 - K1 * h*sin(m_angStart);
	Y1 = Y0 + K1 * h*cos(m_angStart);

	X2 = X3 + K2 * h * sin(angEnd);
	Y2 = Y3 - K2 * h * cos(angEnd);

	// Calculate other constants
	A1 = 6*Y0 - 12*Y1 + 6*Y2;
	A2 = 6*Y1 - 12*Y2 + 6*Y3;

	B1 = -3*X0 + 3*X1;
	B2 = -6*X1 + 6*X2;
	B3 = -3*X2 + 3*X3;

	C1 = -3*Y0 + 3*Y1;
	C2 = -6*Y1 + 6*Y2;
	C3 = -3*Y2 + 3*Y3;

	D1 = -3*X0 + 3*X1;
	D2 = -6*X1 + 6*X2;
	D3 = -3*X2 + 3*X3;

	E1 = 6*X0 - 12*X1 + 6*X2;
	E2 = 6*X1 - 12*X2 + 6*X3;

	F1 =  9*X1 -  3*X0 - 9*X2 + 3*X3;
	F2 =  6*X0 - 12*X1 + 6*X2;
	F3 = -3*X0 +  3*X1;
	F4 =  9*Y1 -  3*Y0 - 9*Y2 + 3*Y3;
	F5 =  6*Y0 - 12*Y1 + 6*Y2;
	F6 = -3*Y0 +  3*Y1;
}

//
//   SetCurAngle: Set the current turn angle to specified a point.
//
void CNewCurve::SetCurAngle(float fPhi)
{
	float t, t_1, t2, t3, t_1_2, t_1_3, fDelta, x, y, Dx, Dy;

	// If the turn direction is "CLOCKWISE", the "Phi" angle should 
	//  be modified
	if (m_TurnDir == CLOCKWISE)
		fPhi = m_fTurnAngle - fPhi;

	// Now, calculate "t" according to the given "fPhi"
	if (fabs(fPhi) < 0.0001f)
	{
		t = 0;
		x = X0;
		y = Y0;

		Dx = F3;
		Dy = F6;
	}
	else
	{
		float fSlant = (float)tan(fPhi+m_angStart.m_fRad);

		t = fPhi/m_fTurnAngle;
		do {
			t_1 = 1 - t;
			t2 = t*t;
			t3 = t2*t;
			t_1_2 = t_1*t_1;
			t_1_3 = t_1_2 * t_1;

			x = X0*t_1_3 + X1*3*t*t_1_2 + X2*3*t2*t_1 + X3*t3;
			y = Y0*t_1_3 + Y1*3*t*t_1_2 + Y2*3*t2*t_1 + Y3*t3;

			Dx = F1*t2 + F2*t + F3;
			Dy = F4*t2 + F5*t + F6;

			fDelta = (y - m_ptCenter.y - fSlant*(x-m_ptCenter.x)) / (Dy - fSlant*Dx);
			t -= fDelta;

		} while (fabs(fDelta) > 0.001f);
		t += fDelta;
	}

	m_pt.x = x;
	m_pt.y = y;

   CLine ln(m_pt, m_ptCenter);
   m_fCurRadius = ln.Length();

	// Caculate the tangent angle at the current point
   m_angTangent = (float)atan2(Dy, Dx);

	if (fabs(t) < 0.0001 || fabs(1-t) < 0.0001)
		m_fCurvature = 0;
	else
	{
	   // Caculate the curvature at the current point
	   float u1 = (A1*t_1 + A2*t)/(B1*t_1_2 + B2*t*t_1 + B3*t2);
	   float u2 = (C1*t_1_2 + C2*t*t_1 + C3*t2) / (float)(pow(D1*t_1_2 + D2*t*t_1 + D3*t2, 2));
	   float u3 = E1*t_1 + E2*t;
   
	   float ff = u1 - u2*u3;
   
	   float DThita;
	   if (fabs(Dx) < 0.0001f)
			DThita = 0;
		else
			DThita = ff/(1+(Dy/Dx)*(Dy/Dx));

	   Ds = (float)sqrt(Dx*Dx + Dy*Dy);
   
	   m_fCurvature = DThita/Ds;
	}

	// If the turn direction is "CLOCKWISE", make some adjustments
	if (m_TurnDir == CLOCKWISE)
	{
		m_angTangent = !m_angTangent;

		m_fCurvature = -m_fCurvature;
	}
}
