//                           - SCRNREF.CPP -
//
//   Implementation of class "CScreenReference".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 26
//

#include "stdafx.h"
#include "ScrnRef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CScreenReference".

CScreenReference::CScreenReference(USHORT uWidth, USHORT uHeight, float fRatio,
											  CPoint2d& ptCenter)
{
	m_uWidth = uWidth;
	m_uHeight = uHeight;
	m_fRatio = fRatio;
	m_ptCenter = ptCenter;
}

CScreenReference::CScreenReference()
{
	m_uWidth = 0;
	m_uHeight = 0;
	m_fRatio = 1.0;
	m_ptCenter.x = 0;
	m_ptCenter.y = 0;
}

//
//   Set the pointer that will map to the center of the view port.
//
void CScreenReference::SetCenterPoint(CPoint2d& pt)
{
	m_ptCenter = pt;
}

//
//   设置视口的左上角点所对应的物理点位置
void CScreenReference::SetLeftTopPoint(CPoint2d& pt)
{
	m_ptCenter.x = pt.x + m_uWidth/m_fRatio/2;
	m_ptCenter.y = pt.y - m_uHeight/m_fRatio/2;
}

//
//   Set the display ratio.
//
void CScreenReference::SetRatio(float fRatio)
{
	m_fRatio = fRatio;
}

//
//   Set the size of the view port.
//
void CScreenReference::SetViewPort(USHORT uWidth, USHORT uHeight)
{
	m_uWidth = uWidth;
	m_uHeight = uHeight;
}

#ifdef _MSC_VER
//
//   Get the world coordinates of the specified window point.
//
CPoint2d CScreenReference::GetWorldPoint(CPoint& pnt)
{
	CPoint2d pt;

	CPoint2d ptLeftTop = GetLeftTopPoint();
	pt.x = (float)( pnt.x/m_fRatio + ptLeftTop.x);
	pt.y = (float)(-pnt.y/m_fRatio + ptLeftTop.y);

	return pt;
}

// Get the window coordinates of the specified world point
CPoint CScreenReference::GetWindowPoint(CPoint2d& pt)
{
	CPoint pnt;

	CPoint2d ptLeftTop = GetLeftTopPoint();
	pnt.x = (int)((pt.x - ptLeftTop.x) * m_fRatio);
	pnt.y = (int)((ptLeftTop.y - pt.y) * m_fRatio);
	return pnt;
}
#endif

/////////////////////////////////////////////////////////////////////////////
//   Helper functions

CPoint2d CScreenReference::GetLeftTopPoint()
{
	CPoint2d pt;
	pt.x = m_ptCenter.x - m_uWidth/(2*m_fRatio);
	pt.y = m_ptCenter.y + m_uHeight/(2*m_fRatio);
	return pt;
}
