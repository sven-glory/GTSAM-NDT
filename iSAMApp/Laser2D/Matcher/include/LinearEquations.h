#ifndef __CLinearEquations
#define __CLinearEquations

#include "CMatrix.h"

///////////////////////////////////////////////////////////////////////////////
//   最小二乘法。
class CLinearEquations
{
public:
	int m_nMaxRows;      // 最在允许的行数量
	int m_nRows;         // 实际的行数量
	int m_nCols;         // 列的数量

	CMatrix A;           // 最小二乘法等式左侧矩阵
	CMatrix B;           // 最小二乘法等式右侧列向量

public:
	CLinearEquations(int nMaxRows, int nCols);
	CLinearEquations();

	// 定义线性方向组的维度
	bool Create(int nMaxRows, int nCols);

	// 重新开始设置矩阵参数
	void Start();

	// 加入一行约束条件
	bool AddRow(float* pa, float b);

	// 取得方程组的行的数量
	int GetRows() const { return m_nRows; }

	// 取得方程组的列的数量
	int GetCols() const { return m_nCols; }

	// 实现对两个线性方程组的合并
	bool Merge(const CLinearEquations& another);

	// 求解最小二乘法
	virtual bool LeastSquareSolve(float* pX, int nNum);

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
