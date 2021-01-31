//                           - SCRNREF.H -
//
//   定义“视口影射”类。
//
//

#pragma once

#include "Geometry.h"
#include "misc.h"

//////////////////////////////////////////////////////////////////////////////
//   定义“视口影射”类。
class DllExport CScreenReference
{
public:
	// 视口端变量
	USHORT   m_uWidth;               // 实际视口的宽度
	USHORT   m_uHeight;              // 实际视口的高度
	int      m_nLeft;                // 实际视口最左侧对应于虚拟视口的X坐标
	int      m_nTop;                 // 实际视口最上侧对应于虚拟视口的Y坐标

	int      m_nVPortWidth;          // 虚拟视口的宽度
	int      m_nVPortHeight;         // 虚拟视口的宽度

	// 世界端变量
	CPnt m_ptCenter;             // 影射到视口中心处的世界点的坐标
	CPnt m_ptWorldLeftTop;       // 世界对象的左上角点坐标
	CPnt m_ptWorldRightBottom;   // 世界对象的右下角点坐标
	float    m_fRatio;               // 从世界到视口端的显示比例尺

public:
	CScreenReference(USHORT uWidth, USHORT uHeight, float fRatio, const CPnt& ptCenter);
	CScreenReference();

	// 设置视口的中心角点所对应的物理点位置
	void SetCenterPoint(const CPnt& pt);

	// 设置视口的左上角点所对应的物理点位置
	void SetLeftTopPoint(const CPnt& pt);

	// 设置世界对象的左上角点坐标
	void SetWorldLeftTop(const CPnt& ptWorldLeftTop) { m_ptWorldLeftTop = ptWorldLeftTop; }

	// 设置世界对象的右下角点坐标
	void SetWorldRightBottom(const CPnt& ptWorldRightBottom) { m_ptWorldRightBottom = ptWorldRightBottom; }

	// 设置世界对象的范围
	void SetWorldRange(const CPnt& ptWorldLeftTop, const CPnt& ptWorldRightBottom);

	// 设置视口内一点到世界坐标系内某一点的对应关系
	void SetPointMapping(const CPoint& ptWindow, const CPnt& ptWorld);

	// 设置显示比例尺
	void SetRatio(float fRatio);

	// 设置视口的大小
	void SetViewPort(USHORT uWidth, USHORT uHeight);

	void SetViewPortDiff(USHORT uWidthDiff, USHORT uHeightDiff);
	// 取得视口左上角处所对应的世界点的坐标
	CPnt GetLeftTopPoint() const;

	// 取得视口中心处所对应的世界点的坐标
	CPnt GetCenterPoint() const { return m_ptCenter; }

	// 取得世界对象的宽度
	float GetWorldWidth() const { return m_ptWorldRightBottom.x - m_ptWorldLeftTop.x; }

	// 取得世界对象的高度
	float GetWorldHeight() const { return  m_ptWorldLeftTop.y - m_ptWorldRightBottom.y; }

#ifdef _MFC_VER
	// 取得视口内指定位置所对应的世界点的坐标
	CPnt GetWorldPoint(const CPoint& pntWindow) const;
	void SetPointMappping(int x, int y, CPnt& pt);
	// 取得指定的世界点所对应的视口点的位置
	CPoint GetWindowPoint(const CPnt& ptWorld) const;
#endif
};
