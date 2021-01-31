#ifndef __CPointFeature
#define __CPointFeature

#include "Geometry.h"
#include "Feature.h"

// ��������
#define GENERIC_POINT_FEATURE      0        // һ������
#define FLAT_REFLECTOR_FEATURE     1        // ƽ�淴�������
#define CYLINDER_FEATURE           2        // Բ������
#define CORNER_FEATURE             3        // �ǵ�
#define SHORT_LINE_FEATURE         4        // ��ֱ������
#define EDGE_FEATURE               5        // ��Ե����
///////////////////////////////////////////////////////////////////////////////
//   ���塰���������ࡣ

class CPointFeature : public CFeature
{
protected:
	int      m_nType;               // ������������
	CPoint2d m_ptCenter;            // ������λ��

public:
	CPointFeature();

	// ���õ�����������
	void SetType(int nType) { m_nType = nType; }

	// ȡ�õ�����������
	int GetType() { return m_nType; }

	// ȡ�õ�����������λ��
	CPoint2d& GetCenterPoint() { return m_ptCenter; }

	// ��������λ��
	void SetCenterPoint(const CPoint2d& ptCenter) { m_ptCenter = ptCenter; }

	// ����һ������
	virtual CPointFeature* Duplicate();

	// ���ļ���װ��������Ĳ���
	virtual bool Load(FILE* fp);

	// ���������Ĳ������浽�ļ���
	virtual bool Save(FILE* fp);

#ifdef _MSC_VER
	// ����Ļ�ϻ��ƴ˵�����
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif
};
#endif
