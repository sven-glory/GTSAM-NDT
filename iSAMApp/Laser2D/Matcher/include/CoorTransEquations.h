#ifndef __CCoorTransEquations
#define __CCoorTransEquations

#include "LinearEquations.h"

///////////////////////////////////////////////////////////////////////////////
//   定义“坐标变换方程组”类。
class CCoorTransEquations : public CLinearEquations
{
private:
	// 坐标变换方程组的特殊处理过程
	bool SpecialProcess(float fSin, float fCos, float& x, float& y);

public:
	CCoorTransEquations(int nMaxRows, int nCols);

	// 求解最小二乘法
	virtual bool LeastSquareSolve(float* pX, int nNum);
};
#endif
