#include "stdafx.h"
#include <math.h>
#include "LinearEquations.h"
#include "DebugTrace.h"
///////////////////////////////////////////////////////////////////////////////

//
//   设置最大可容纳的行数，同时列数在此确定(不可更改)。
//
CLinearEquations::CLinearEquations(int nMaxRows, int nCols)
{
	Create(nMaxRows, nCols);
}

CLinearEquations::CLinearEquations()
{
	m_nMaxRows = m_nRows = m_nCols = 0;
}

//
//   定义线性方向组的维度。
//
bool CLinearEquations::Create(int nMaxRows, int nCols)
{
	A.Create(nMaxRows, nCols);
	B.Create(nMaxRows, 1);

	m_nMaxRows = nMaxRows;
	m_nRows = 0;
	m_nCols = nCols;

	return true;
}

//
//   开始设置矩阵参数。
//
void CLinearEquations::Start()
{
	m_nRows = 0;
}

//
//   加入一行约束条件。
//
bool CLinearEquations::AddRow(float* pa, float b)
{
	// 核对是否超过了最大行数
	if (m_nRows + 1 >= m_nMaxRows)
		return false;

	// 依次设置矩阵系数
	for (int i = 0; i < m_nCols; i++)
		A.SetAt(m_nRows, i, *pa++);

	B.SetAt(m_nRows, 0, b);
	
	// 行数增加
	m_nRows++;
	return true;
}

//
//   实现对两个线性方程组的合并。
//
bool CLinearEquations::Merge(const CLinearEquations& another)
{
	// 首先核对方程组的列数是否相同
	if (m_nCols != another.m_nCols)
		return false;

	// 依次增加矩阵行数据
	for (int j = 0; j < another.GetRows(); j++)
	{
		// 如果行数超限，超过的部分会不会被加入
		if (m_nRows++ >= m_nMaxRows)
		{
			m_nRows = m_nMaxRows;
			break;
		}

		for (int i = 0; i < m_nCols; i++)
			A.SetAt(m_nRows + j, i, another.A.GetAt(j, i));

		B.SetAt(m_nRows, 0, another.B.GetAt(j, 0));
	}

	return true;
}

//
//   求解最小二乘法。
//   结果保存在pX指向的缓冲区中，缓冲区大小nNum必须不小于m_nCols。
//
bool CLinearEquations::LeastSquareSolve(float* pX, int nNum)
{
	if (pX == NULL || nNum < m_nCols)
		return false;

	// 确定A、B矩阵的行数
	A.m_nRow = m_nRows;
	B.m_nRow = m_nRows;

	CMatrix matrix_1 = A.Transpose();
	CMatrix matrix_2 = matrix_1 * A;

	// 行列式的值
	float det = matrix_2.Determinant();
	if (det == 0)
		return false;

	CMatrix matrix_p = matrix_2.GetAccompany() * matrix_1 * B;

	// 得到结果向量，将它返回到给定的缓冲区中
	for (int i = 0; i < m_nCols; i++)
		*pX++ = matrix_p.GetAt(i, 0) / det;

	return true;
}

#ifdef _MSC_VER
void CLinearEquations::Dump()
{
	DebugTrace(_T("Least square method - A:\n"));
	A.Dump();

	DebugTrace(_T("Least square method - B:\n"));
	B.Dump();
}
#endif
