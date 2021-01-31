#include "stdafx.h"
#include "PointFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPointFeature::CPointFeature(float _x, float _y)
{
	x = _x;
	y = _y;
	m_nSubType = GENERIC_POINT_FEATURE;
	m_nMatchId = -1;
}

//
//   生成一个复本
//
CPointFeature* CPointFeature::Duplicate() const
{
	CPointFeature* p = new CPointFeature;
	*p = *this;
	return p;
}

//
//   判断该点特征是否被指定的扫描线照到，并返回扫描点坐标。该函数主要用于仿真。
//
bool CPointFeature::HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist)
{
	CPnt pt;
	float f;

	// 构造一个0.1米半径的圆
	CCircle circle(*this, 0.1f);

	// 判断两条线段是否有交点
	bool bHit = circle.IntersectLineAt(lnRay, pt, f);

	// 如果有交点，并且入射角合格
	if (bHit)
	{
		ptHit = pt;
		fDist = f;
		return true;
	}

	// 没有交点，或反面不可用，或入射角超限，返回false
	return false;
}

//
//   针对给定的点云，在规定的角度范围内，检测点云中是否含有该点特征，并返回它的中心位置。
//
bool CPointFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, 
									float fEndAngle, CPnt* ptCenter)
{
	return false;
}

//
//   从文件中装入点特征的参数。
//   返回值：
//     -1  : 读取失败
//     >=0 : 点特征的类型编号
//
int CPointFeature::LoadText(FILE* fp)
{
	if (fscanf(fp, "%f\t%f\n", &x, &y) != 2)
		return -1;

	return m_nSubType;
}

//
//   将点特征的参数保存到文件中。
//   返回值：
//     -1  : 写入失败
//     >=0 : 点特征的类型编号
//
int CPointFeature::SaveText(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\n", m_nSubType, x, y);
	return m_nSubType;
}

//
//   从二进制文件中装入点特征的参数。
//
int CPointFeature::LoadBinary(FILE* fp)
{
	// 读入点的坐标
	float f[2];
	if (fread(f, sizeof(float), 2, fp) != 2)
		return -1;

	x = f[0];
	y = f[1];

	return m_nSubType;
}

//
//   将点特征的参数保存到二进制文件中。
//
int CPointFeature::SaveBinary(FILE* fp)
{
	if (fwrite(&m_nSubType, sizeof(int), 1, fp) != 1)
		return -1;

	if (fwrite(&x, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&y, sizeof(float), 1, fp) != 1)
		return -1;

	return m_nSubType;
}

#define POINT_FEATURE_RADIUS_MM        25     // 假定点特征半径为25mm

#ifdef _MFC_VER
//
//   在屏幕上绘制此点特征。
//
void CPointFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
	int nPointSize, int nLineWidth)
{
	int nRadius = POINT_FEATURE_RADIUS_MM / 1000.0f * ScrnRef.m_fRatio;

	if (nRadius < nPointSize)
		nRadius = nPointSize;

	Draw(ScrnRef, pDC, crColor, nRadius, nLineWidth);
}

//
//   在屏幕上绘制此点特征的ID号。
//
void CPointFeature::PlotId(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor)
{
	CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());

	CString str;
	str.Format(_T("%d"), id);

	COLORREF crOldColor = pDC->SetTextColor(crColor);
	pDC->TextOut(pnt.x - 20, pnt.y - 20, str);
	pDC->SetTextColor(crOldColor);
}

#elif defined QT_VERSION

//
//   在屏幕上绘制此点特征。
//
void CPointFeature::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crSelected,
	int nPointSize, int nLineWidth)
{
	int nRadius = POINT_FEATURE_RADIUS_MM / 1000.0f * ScrnRef.m_fRatio;

	if (nRadius < nPointSize)
		nRadius = nPointSize;

	Draw(ScrnRef, pPainter, crColor, nRadius, nLineWidth);
}

//
//   在屏幕上绘制此点特征的ID号。
//
void CPointFeature::PlotId(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor)
{
	CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());

	QString str = QString::number(id);
	pPainter->setPen(crColor);
	pPainter->drawText(pnt.x - 20, pnt.y - 20, str);
}
#endif
