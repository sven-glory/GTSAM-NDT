#include "stdafx.h"
#include "Frame.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   实现二维坐标系类型"CFrame"。

CFrame::CFrame(const CPosture& pstLocal)
{
	Create(pstLocal);
}

//
//   根据原点坐标和倾角构造坐标系。
//
CFrame::CFrame(const CPnt& ptOrigin, const CAngle& angSlant)
{
	CPosture::SetPosture(ptOrigin, angSlant);
}

CFrame::CFrame(float _x, float _y, float _t)
{
	CPosture::SetPosture(_x, _y, _t);
}

//
//   根据给定的姿态构造坐标系。
//
void CFrame::Create(const CPosture& pstLocal)
{
	CPosture::SetPosture(pstLocal.x, pstLocal.y, pstLocal.fThita);
}	

//
//   定义“*=”运算。
//
CFrame& CFrame::operator *= (const CFrame& another)
{
	*this = *this * another;
	return *this;
}

//
//   定义“*”运算。
//
CFrame CFrame::operator * (const CFrame& another)
{
	CFrame frm = another;
	frm.InvTransform(*this);

	return frm;
}
