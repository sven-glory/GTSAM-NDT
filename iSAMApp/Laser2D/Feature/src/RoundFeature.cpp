#include "stdafx.h"
#include "RoundFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CRoundFeature::CRoundFeature()
{
	m_nSubType = ROUND_FEATURE;
}

//
//   生成一个复本
//
CPointFeature* CRoundFeature::Duplicate() const
{
	CRoundFeature* p = new CRoundFeature;
	*p = *this;
	return p;
}

//
//   设置柱状特征的参数
//
void CRoundFeature::SetParam(float fRadius)
{
	m_fRadius = fRadius;
}

//
//   针对给定的点云，在规定的角度范围内，检测点云中是否含有该柱状物，并返回它的中心位置。
//
bool CRoundFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter)
{
	return true;
}

//
//   从文件中装入圆柱特征的参数。
//
int CRoundFeature::LoadText(FILE* fp)
{
	if (!CPointFeature::Load(fp))
		return -1;

	float fRadius;
	if (fscanf(fp, "%f\n", &fRadius) != 1)
		return -1;

	SetParam(fRadius);
	
	return m_nSubType;
}

//
//   将圆柱特征的参数保存到文件中。
//
int CRoundFeature::SaveText(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t", m_nSubType, x, y);
	fprintf(fp, "%f\n", m_fRadius);

	return m_nSubType;
}

//
//   从二进制文件中装入点特征的参数。
//
int CRoundFeature::LoadBinary(FILE* fp)
{
	if (CPointFeature::LoadBinary(fp) < 0)
		return -1;

	float fRadius;
	if (fread(&fRadius, sizeof(float), 1, fp) != 1)
		return -1;

	SetParam(fRadius);

	return m_nSubType;
}

//
//   将点特征的参数保存到二进制文件中。
//
int CRoundFeature::SaveBinary(FILE* fp)
{
	if (CPointFeature::SaveBinary(fp) < 0)
		return -1;

	if (fwrite(&m_fRadius, sizeof(float), 1, fp) != 1)
		return -1;

	return m_nSubType;
}

#ifdef _MSC_VER
//
//   在屏幕上绘制此圆柱特征。
//
void CRoundFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize)
{
	Draw(ScrnRef, pDC, (128, 128, 0), 5);
}
#endif
