#include "stdafx.h"
#include "PointFeature.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPointFeature::CPointFeature()
{
	m_nType = GENERIC_POINT_FEATURE;
}

//
//   ����һ������
//
CPointFeature* CPointFeature::Duplicate()
{
	CPointFeature* p = new CPointFeature;
	*p = *this;
	return p;
}

//   ���ļ���װ��������Ĳ�����
//
bool CPointFeature::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t", &m_ptCenter.x, &m_ptCenter.y);

	return true;
}

//
//   ���������Ĳ������浽�ļ��С�
//
bool CPointFeature::Save(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t", m_nType, m_ptCenter.x, m_ptCenter.y);

	return true;
}

#ifdef _MSC_VER
//
//   ����Ļ�ϻ��ƴ˵�������
//
void CPointFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize)
{
	m_ptCenter.Draw(ScrnRef, pDC, crColor, 5);
}
#endif
