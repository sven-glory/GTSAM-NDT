//                         - ARC.CPP -
//
//   Implementation of class "CArc", which depicts arc curve.
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

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArc".

//
//   CArc: The constructor.
//
CArc::CArc(CPoint2d& ptCenter, CPoint2d& ptStart, CPoint2d& ptEnd, CTurnDir TurnDir)
{
	m_ptCenter = ptCenter;
	m_ptStart = ptStart;
	m_ptEnd = ptEnd;
	m_TurnDir = TurnDir;

	if (TurnDir == CLOCKWISE)
	{
		m_ptStart = ptEnd;
		m_ptEnd = ptStart;
	}

	CLine StartLine(m_ptCenter, m_ptStart);
	CLine EndLine(m_ptCenter, m_ptEnd);

	m_fCurRadius = m_fRadius = StartLine.Length();
	m_angStart = StartLine.SlantAngle();

	CAngle angTurn = EndLine.SlantAngle() - m_angStart;

	// Get the curve's turn angle
	m_fTurnAngle = angTurn.m_fRad;
	m_Transform = CTransform(m_ptCenter, m_angStart);
}

//
//   SetCurAngle: Set the current turn angle to specified a point.
//
void CArc::SetCurAngle(float fPhi)
{
	if (m_TurnDir == CLOCKWISE)
		fPhi = m_fTurnAngle - fPhi;

	m_fCurvature = 1.0f / m_fRadius;

	m_angTangent = m_angStart + CAngle(PI/2+fPhi);

	CPoint2d ptTemp((float)(m_fRadius * cos(fPhi)), (float)(m_fRadius * sin(fPhi)));     // Local point
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
