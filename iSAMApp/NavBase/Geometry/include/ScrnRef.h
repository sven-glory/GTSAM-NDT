//                           - SCRNREF.H -
//
//   The interface of class "CScreenReference".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#ifndef __CScreenReference
#define __CScreenReference

#include "Geometry.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CScreenReference".
class DllExport CScreenReference
{
public:
	USHORT m_uWidth;       // Width of the view port
	USHORT m_uHeight;      // Height of the view port
	float  m_fRatio;       // Display ratio
	CPoint2d   m_ptCenter;     // The pointer mapping to the center of the view port

public:
	CScreenReference(USHORT uWidth, USHORT uHeight, float fRatio, CPoint2d& ptCenter);
	CScreenReference();

	// Set the pointer that will map to the center of the view port
	void SetCenterPoint(CPoint2d& pt);

	// 设置视口的左上角点所对应的物理点位置
	void SetLeftTopPoint(CPoint2d& pt);

	// Set the display ratio
	void SetRatio(float fRatio);

	// Set the size of the view port
	void SetViewPort(USHORT uWidth, USHORT uHeight);

	// Get the coordinates of the left-top point
	CPoint2d GetLeftTopPoint();

#ifdef _MSC_VER
	// Get the world coordinates of the specified window point
	CPoint2d GetWorldPoint(CPoint& pnt);

	// Get the window coordinates of the specified world point
	CPoint GetWindowPoint(CPoint2d& pt);
#endif
};
#endif
