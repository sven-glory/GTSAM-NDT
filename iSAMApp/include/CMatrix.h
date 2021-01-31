//                                         - MATRIX.H -
//
//   CMatrix��Ľӿڶ��塣
//

#ifndef __CMatrix
#define __CMatrix

#include <afxtempl.h>

//////////////////////////////////////////////////////////////////////////////
//   CMatrix��Ľӿڶ���
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

	// ���ɾ���
	void Create(int nRow, int nCol);

	// �����Ƿ���һ������
	BOOL IsSquare();

	// �����İ������
	CMatrix GetAccompany()const;

	// ����������ʽ
	float Determinant() const;

	CMatrix GetRemainder(int nRow, int nCol)const;

	// �Ƿ��������
	BOOL CanContrary()const;

	// ������ת�þ���
	CMatrix T()const;

	// �����������
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
