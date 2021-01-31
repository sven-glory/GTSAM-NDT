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

CTransform::CTransform(const CPosture& pstLocal)
{
	Init(pstLocal);
}

//
//   CTransform: Overloaded constructor.
//
CTransform::CTransform(const CPnt& ptOrigin, const CAngle& angSlant)
{
	CPosture::SetPosture(ptOrigin, angSlant);
}

//
//   Init the origin and slant angle of the local frame.
//
void CTransform::Init(const CPosture& pstLocal)
{
	CPosture::SetPosture(pstLocal.x, pstLocal.y, pstLocal.fThita);
}	

void CTransform::Create(CLine& line1, CLine& line2)
{
	CTransform trans1;

	float angle = line1.SlantAngleRad() - line2.SlantAngleRad();
	trans1.Create(0, 0, angle);

	CPnt pnt1 = trans1.GetWorldPoint(line2.m_ptStart);

	float x = line1.m_ptStart.x - pnt1.x;
	float y = line1.m_ptStart.y - pnt1.y;

	Create(x, y, angle);
}

//
//   GetWorldPoint: A transformation from the local frame to the
//   world frame.
//
CPnt CTransform::GetWorldPoint(const CPnt& ptLocal) const
{
	CPnt pt = ptLocal;
	pt.x = x + ptLocal.x * (float)cos(fThita) - ptLocal.y * (float)sin(fThita);
	pt.y = y + ptLocal.y * (float)cos(fThita) + ptLocal.x * (float)sin(fThita);

	return pt;
}

//
//   GetWorldPoint: A reverse transformation from the world frame to the
//   local frame.
//
CPnt CTransform::GetLocalPoint(const CPnt& ptWorld) const
{
	CPnt pt = ptWorld;         // 先复制所有的字段
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
CPosture CTransform::GetWorldPosture(const CPosture& pstLocal) const
{
	CPosture pst;

	CAngle ang = fThita + pstLocal.fThita;
	pst.SetPnt(GetWorldPoint(pstLocal));
	pst.SetAngle(ang);
	return pst;
}

//
//   GetLocalPosture: A reverse posture transformation from the world
//   frame to the local frame.
//
CPosture CTransform::GetLocalPosture(const CPosture& pstWorld) const
{
	CPosture pst;

	CAngle ang = pstWorld.fThita - fThita;
	pst.SetPnt(GetLocalPoint(pstWorld));
	pst.SetAngle(ang);

	return pst;
}

//
//   将一段直线从局部坐标系到变换到世界坐标系中。
//
CLine CTransform::GetWorldLine(const CLine& lnLocal) const
{
	CPnt ptStart = GetWorldPoint(lnLocal.m_ptStart);
	CPnt ptEnd = GetWorldPoint(lnLocal.m_ptEnd);
	CLine ln(ptStart, ptEnd);
	return ln;
}

//
//   将一段直线从世界坐标系到变换到局部坐标系中。
//
CLine CTransform::GetLocalLine(const CLine& lnWorld) const
{
	CPnt ptStart = GetLocalPoint(lnWorld.m_ptStart);
	CPnt ptEnd = GetLocalPoint(lnWorld.m_ptEnd);
	CLine ln(ptStart, ptEnd);
	return ln;
}
