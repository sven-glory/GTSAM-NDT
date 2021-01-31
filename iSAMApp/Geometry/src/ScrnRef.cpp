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
											  const CPnt& ptCenter)
{
	m_uWidth = uWidth;
	m_uHeight = uHeight;
	m_fRatio = fRatio;
	m_ptCenter = ptCenter;
}

CScreenReference::CScreenReference()
{
	m_uWidth = 1;
	m_uHeight = 1;
	m_fRatio = 1.0;
	m_ptCenter.x = 0;
	m_ptCenter.y = 0;
}

//
//   Set the pointer that will map to the center of the view port.
//
void CScreenReference::SetCenterPoint(const CPnt& pt)
{
	m_ptCenter = pt;
}

//
//   设置视口的左上角点所对应的物理点位置。
//
void CScreenReference::SetLeftTopPoint(const CPnt& pt)
{
	m_ptCenter.x = pt.x + m_uWidth/m_fRatio/2;
	m_ptCenter.y = pt.y - m_uHeight/m_fRatio/2;
}

//
//   设置视口内一点到世界坐标系内某一点的对应关系。
//
void CScreenReference::SetPointMapping(const CPoint& ptWindow, const CPnt& ptWorld)
{
	// 计算视口中心点对应的世界点坐标
	int dx = ptWindow.x - m_uWidth / 2;
	int dy = ptWindow.y - m_uHeight / 2;

	m_ptCenter.x = ptWorld.x - dx / m_fRatio;
	m_ptCenter.y = ptWorld.y + dy / m_fRatio;

#if 0
	// 计算虚拟视口的大小
	m_nVPortWidth = GetWorldWidth() * m_fRatio;
	m_nVPortHeight = GetWorldHeight() * m_fRatio;
#endif

	// 计算真实视口的左上角处所对应的虚拟视口中点的坐标
	m_nLeft = 0;
	m_nTop = 0;
}

//
//   设置视口的指定点所对应的物理点位置
//
void CScreenReference::SetPointMappping(int x, int y, CPnt& pt)
{
	int dx = x - m_uWidth / 2;
	int dy = y - m_uHeight / 2;
	m_ptCenter.x = pt.x - dx / m_fRatio;
	m_ptCenter.y = pt.y + dy / m_fRatio;
}

//
//   设置显示比例尺。
//
void CScreenReference::SetRatio(float fRatio)
{
	m_fRatio = fRatio;
}

//
//   设置世界对象的范围。
//
void CScreenReference::SetWorldRange(const CPnt& ptWorldLeftTop, const CPnt& ptWorldRightBottom)
{
	m_ptWorldLeftTop = ptWorldLeftTop;
	m_ptWorldRightBottom = ptWorldRightBottom;
}

//
//   设置视口的大小。
//
void CScreenReference::SetViewPort(USHORT uWidth, USHORT uHeight)
{
	m_uWidth = uWidth;
	m_uHeight = uHeight;
}

void CScreenReference::SetViewPortDiff(USHORT uWidthDiff, USHORT uHeightDiff)
{
	m_uWidth += uWidthDiff;
	m_uHeight += uHeightDiff;
}

#ifdef _MFC_VER
//
//   取得视口内指定位置所对应的世界点的坐标。
//
CPnt CScreenReference::GetWorldPoint(const CPoint& pntWindow) const
{
	CPnt pt;

	CPnt ptLeftTop = GetLeftTopPoint();
	pt.x = (float)( pntWindow.x/m_fRatio + ptLeftTop.x);
	pt.y = (float)(-pntWindow.y/m_fRatio + ptLeftTop.y);

	return pt;
}

//
//   取得指定的世界点所对应的视口点的位置。
//
CPoint CScreenReference::GetWindowPoint(const CPnt& ptWorld) const
{
	CPoint pnt;

	CPnt ptLeftTop = GetLeftTopPoint();
	pnt.x = (int)((ptWorld.x - ptLeftTop.x) * m_fRatio + 0.5f);
	pnt.y = (int)((ptLeftTop.y - ptWorld.y) * m_fRatio + 0.5f);
	return pnt;
}
#endif
//
//   取得视口左上角处所对应的世界点的坐标。
//
CPnt CScreenReference::GetLeftTopPoint() const
{
	CPnt pt;
	pt.x = m_ptCenter.x - m_uWidth / (2 * m_fRatio);
	pt.y = m_ptCenter.y + m_uHeight / (2 * m_fRatio);
	return pt;
}
