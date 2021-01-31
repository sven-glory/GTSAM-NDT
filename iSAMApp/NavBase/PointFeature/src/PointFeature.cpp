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
//   生成一个复本
//
CPointFeature* CPointFeature::Duplicate()
{
	CPointFeature* p = new CPointFeature;
	*p = *this;
	return p;
}

//   从文件中装入点特征的参数。
//
bool CPointFeature::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t", &m_ptCenter.x, &m_ptCenter.y);

	return true;
}

//
//   将点特征的参数保存到文件中。
//
bool CPointFeature::Save(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t", m_nType, m_ptCenter.x, m_ptCenter.y);

	return true;
}

#ifdef _MSC_VER
//
//   在屏幕上绘制此点特征。
//
void CPointFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize)
{
	m_ptCenter.Draw(ScrnRef, pDC, crColor, 5);
}
#endif
