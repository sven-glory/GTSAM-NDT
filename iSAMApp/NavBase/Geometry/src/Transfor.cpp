//                        - TRANSFOR.CPP -
//
//   Implementation of class "CTransform", which describe how to do
//   coordinates transformation between the world frame and a local
//   frame.
//
//   Author: Zhang Lei
//   Date:   2001. 9. 11
//

#include "stdafx.h"
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTransform".

//
//   CTransform: Overloaded constructor.
//
CTransform::CTransform(CPoint2d& ptOrigin, CAngle& angSlant)
{
	CPosture::SetPosture(ptOrigin, angSlant);
}

//
//   Init the origin and slant angle of the local frame.
//
void CTransform::Init(CPosture& pstLocal)
{
	CPosture::SetPosture(pstLocal.x, pstLocal.y, pstLocal.fThita);
}	

void CTransform::Create(CLine& line1, CLine& line2)
{
	CTransform trans1;

	float angle = line1.GetAngle() - line2.GetAngle();
	trans1.Create(0, 0, angle);

	CPoint2d pnt1 = trans1.GetWorldPoint(line2.m_ptStart);

	float x = line1.m_ptStart.x - pnt1.x;
	float y = line1.m_ptStart.y - pnt1.y;

	Create(x, y, angle);
}

//
//   GetWorldPoint: A transformation from the local frame to the
//   world frame.
//
CPoint2d CTransform::GetWorldPoint(CPoint2d& ptLocal)
{
	CPoint2d pt;
	pt.x = x + ptLocal.x * (float)cos(fThita) - ptLocal.y * (float)sin(fThita);
	pt.y = y + ptLocal.y * (float)cos(fThita) + ptLocal.x * (float)sin(fThita);

	return pt;
}

//
//   GetWorldPoint: A reverse transformation from the world frame to the
//   local frame.
//
CPoint2d CTransform::GetLocalPoint(CPoint2d& ptWorld)
{
	CPoint2d pt;
	float fDx = ptWorld.x - x;
	float fDy = ptWorld.y - y;

	pt.x = fDx * (float)cos(fThita) + fDy * (float)sin(fThita);
	pt.y = fDy * (float)cos(fThita) - fDx * (float)sin(fThita);

	return pt;
}

//
//   GetWorldPosture: A posture transformation from the local frame to
//   the world frame.
//
CPosture CTransform::GetWorldPosture(CPosture& pstLocal)
{
	CPosture pst;

	CAngle ang = fThita + pstLocal.fThita;
	pst.SetPnt(GetWorldPoint(pstLocal.GetPoint2dObject()));
	pst.SetAngle(ang);
	return pst;
}

//
//   GetLocalPosture: A reverse posture transformation from the world
//   frame to the local frame.
//
CPosture CTransform::GetLocalPosture(CPosture& pstWorld)
{
	CPosture pst;

	CAngle ang = pstWorld.fThita - fThita;
	pst.SetPnt(GetLocalPoint(pstWorld.GetPoint2dObject()));
	pst.SetAngle(ang);

	return pst;
}
