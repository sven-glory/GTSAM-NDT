#include "stdafx.h"
#include "LineMatchPair.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   根据此直线匹配对生成用于“最小二乘法”的参数。
//   最小二乘法的矩阵形式为 A*X = B，本函数将生成A矩阵中的两行及B列向量中的两行。
//
bool CLineMatchPair::MakeLeastSquareData(float fCoefA[3][4], float fCoefB[3])
{
	// 在世界坐标系中计算Odometry姿态到该直线的距离
	float A = m_lnWorld.a;
	float B = m_lnWorld.b;
	float C = m_lnWorld.c;

	float fJudge = A * m_pstOdometry.x + B * m_pstOdometry.y + C;
	int nSign = (fJudge >= 0) ? 1 : -1;

	fCoefA[0][0] = 0;
	fCoefA[0][1] = 0;
	fCoefA[0][2] = A;
	fCoefA[0][3] = B;

	fCoefB[0] = nSign * m_fDistToOrigin * sqrt(A * A + B * B) - C;

	// 角度的正统
	fCoefA[1][0] = 1;
	fCoefA[1][1] = 0;
	fCoefA[1][2] = 0;
	fCoefA[1][3] = 0;

	fCoefB[1] = sin(m_ang);

	// 角度的余弦
	fCoefA[2][0] = 0;
	fCoefA[2][1] = 1;
	fCoefA[2][2] = 0;
	fCoefA[2][3] = 0;

	fCoefB[2] = cos(m_ang);

	return true;
}
