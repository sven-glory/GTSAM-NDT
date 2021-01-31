#ifndef __CLineFeature
#define __CLineFeature

#include "Geometry.h"
#include "Feature.h"
#include <afxwin.h>

///////////////////////////////////////////////////////////////////////////////
//   ���塰���������ࡣ

class CLineFeature : public CFeature, public CLine
{
public:
    float m_fNormalX;
	 float m_fNormalY;          // normal vector
    float m_fDistOrigin;       // ��ԭ��ľ���

    long  m_lStart;
	 long  m_lEnd;              // point numbers in scan
    float m_fSigma2;           // ƥ�����

	bool  m_bSelected;         // �������߶��Ƿ�ѡ��(������Ļ�༭����)

public:
	CLineFeature();

	virtual bool Load(FILE* file);
	virtual bool Save(FILE* file);

	// �����߶�
	void Create(CPnt& ptStart, CPnt& ptEnd);

	// �����߶�
	void Create(CLine& ln);
	
	// ��ת�߶εķ���(����㡢�յ�Ե�)
	void Reverse();
	
	// ����ֱ�߶�ͶӰ����һֱ�߶Σ��õ���ͶӰ�߶δ���lnResult
	bool ProjectToLine(CLineFeature& ln, CLineFeature& lnResult);


	// ����������ƽ��
	virtual void Move(float fX, float fY);

	// ������������ת
	virtual void Rotate(CAngle ang, CPnt ptCenter);

	// ѡ��/ȡ��ѡ�д��߶�
	void Select(bool bOn) {m_bSelected = bOn;}

#ifdef _MSC_VER
	// (����Ļ������)����ָ���ĵ��Ƿ����߶���
	//bool HitTest(CScreenReference& ScrnRef, CPoint point);

	// ����Ļ����ʾ��ֱ������
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif

};
#endif
