#ifndef __CLineMatchPair
#define __CLineMatchPair

#include "Geometry.h"

///////////////////////////////////////////////////////////////////////////////
//   "CLineMatchPair"类的定义。
class CLineMatchPair
{
private:
	float m_fDistToOrigin;         // 在局部坐标系中，原点到该直线的距离

public:
	CLine m_lnLocal;               // 在局部坐标系中的直线段
	CLine m_lnWorld;               // 在全局坐标系中的直线段
	int   m_nLocalId;              // 局部直线段的ID号
	int   m_nWorldId;              // 全局直线段的ID号
	CPosture m_pstOdometry;        // 在全局坐标系中的Odometry姿态
	CAngle m_ang;                  // 结果角度值(即第一个变量sin(x), 第二个变量cos(x)中的x)

public:
#if 0
	CLineMatchPair(const CLine& lnLocal, const CLine& lnWorld, CPosture& pstOdometry, CAngle& ang)
	{
		Create(lnLocal, lnWorld, pstOdometry, ang);
	}
#endif

	CLineMatchPair() {}

	// 生成直线匹配对
	void Create(int nLocalId, int nWorldId, const CLine& lnLocal, const CLine& lnWorld, CPosture& pstOdometry, CAngle& ang)
	{
		m_nLocalId = nLocalId;
		m_nWorldId = nWorldId;
		m_lnLocal = lnLocal;
		m_lnWorld = lnWorld;
		m_pstOdometry = pstOdometry;
		m_ang = ang;

		CPnt ptOrigin(0, 0);
		m_fDistToOrigin = m_lnLocal.DistanceToPoint(false, ptOrigin);
	}

	// 根据此直线匹配对生成用于“最小二乘法”的参数
	bool MakeLeastSquareData(float fCoefA[3][4], float fCoefB[3]);
};
#endif
