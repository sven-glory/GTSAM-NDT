//                                         - MATRIX.H -
//
//   CMatrix类的接口定义。
//

#ifndef __CMatrix
#define __CMatrix

#include <afxtempl.h>

//////////////////////////////////////////////////////////////////////////////
//   CMatrix类的接口定义
class CMatrix
{
public:
	int m_nRow;
	int m_nCol;
	CArray<float, float&> m_Array;

private:
	void Copy(const CMatrix& m);

public:
	CMatrix();
	~CMatrix() {}

	// 生成矩阵
	void Create(int nRow, int nCol);

	// 矩阵是否是一个方阵
	BOOL IsSquare();

	// 求矩阵的伴随矩阵
	CMatrix GetAccompany()const;

	// 求矩阵的行列式
	float Determinant() const;

	CMatrix GetRemainder(int nRow, int nCol)const;

	// 是否可以求逆
	BOOL CanContrary()const;

	// 求矩阵的转置矩阵
	CMatrix T()const;

	// 求矩阵的逆矩阵
	CMatrix operator ~();

	CMatrix operator / (CMatrix & m);
	CMatrix operator / (float m);
	CMatrix operator * (const CMatrix & m);
	CMatrix operator * (float m);

	BOOL CanMutiply(const CMatrix & m)const;

	BOOL CanAddSub(const CMatrix & m)const;

	CMatrix(const CMatrix &);

	CMatrix operator + (const CMatrix & m);

	CMatrix operator - (const CMatrix & m);

	CMatrix & operator = (const CMatrix &m);

	CMatrix & operator = (float m);

	float GetAt(int nRow, int nCol);

	void Dump();
};
#endif
