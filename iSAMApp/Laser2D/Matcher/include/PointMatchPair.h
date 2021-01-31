#ifndef __CPointMatchPair
#define __CPointMatchPair

#include "Geometry.h"

///////////////////////////////////////////////////////////////////////////////
//   "CPointMatchPair"类的定义。
class CPointMatchPair
{
public:
	CPnt m_ptLocal;               // 在局部坐标系中的点
	CPnt m_ptWorld;               // 在全局坐标系中的点
	int      m_nLocalId;              // 局部直线段的ID号
	int      m_nWorldId;              // 全局直线段的ID号
	CPnt m_ptLocalToWorld;      // 局部坐标点影射到全局坐标系后的位置

public:
	CPointMatchPair(int nLocalId, int nWorldId, const CPnt& pnt1, const CPnt& pnt2)
	{
		Create(nLocalId, nWorldId, pnt1, pnt2);
	}

	CPointMatchPair() {}

	void Create(int nLocalId, int nWorldId, const CPnt& pnt1, const CPnt& pnt2)
	{
		m_nLocalId = nLocalId;
		m_nWorldId = nWorldId;
		m_ptLocal = pnt1;
		m_ptWorld = pnt2;
	}

	// 根据此点对匹配对生成用于“最小二乘法”的参数
	bool MakeLeastSquareData(float fCoefA[2][4], float fCoefB[2]);

	// 根据此点对匹配对生成用于“最小二乘法”的参数(特殊情况)
	bool MakeLeastSquareData1(float fCoefA[2][2], float fCoefB[2], float fSin, float fCos);
};
#endif
