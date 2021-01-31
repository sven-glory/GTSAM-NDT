#ifndef __CScanPoint
#define __CScanPoint

#include "Geometry.h"

#define SCAN_OUT_OF_RANGE              99999L

///////////////////////////////////////////////////////////////////////////////
//   对应于每一条激光扫描线的扫描数据。
class CLaserBeam
{
public:
	float m_fDist;          // 扫描距离(单位:m)
	int m_nIntensity;       // 反光强度数据

public:
	CLaserBeam()
	{
		m_fDist = 0;
		m_nIntensity = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
// “CScanPoint”类定义了二维扫描点的概念。
class CScanPoint : public CPnt
{
public:
	short m_nLineID;     // 对应的直线段的编号(如果该点落在一条直线段上),否则-1
	int   m_nIntensity;  // 反光强度
	bool  m_bDelete;     // 删除标志
	bool  m_bHighReflective; // 强反光标志

public:
	CScanPoint(float _x = 0, float _y = 0, unsigned _id = 0) :
		CPnt(_x, _y, _id)
	{
		m_nLineID = -1;
		m_nIntensity = 0;
		m_bDelete = false;
		m_bHighReflective = false;
	}

	// 将该扫描点标为“距离超限”
	void SetOutOfRange()
	{
		x = y = 0;
		a = 0;
		r = SCAN_OUT_OF_RANGE;
		m_bDelete = false;
	}
};
#endif
