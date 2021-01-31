#include "stdafx.h"
#include "FeatureSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPointFeatureSet& CFeatureSet::GetPointFeatureSet()
{
	CPointFeatureSet* p = dynamic_cast<CFeatureSet*>(this);
	return *p;
}

//
//   清除数据。
//
void CFeatureSet::Clear()
{
	m_pstObserver.SetPosture(0, 0, 0);
	CPointFeatureSet::Clear();
}

//
//   从文件中装入特征图。
//
int CFeatureSet::LoadText(FILE* fp)
{
	Clear();

	// 装入观测器姿态
	float x, y, thita;
	if (fscanf(fp, "%f\t%f\t%f\n", &x, &y, &thita) != 3)
		return 0;

	// 装入所有特征点
	return CPointFeatureSet::LoadText(fp);
}

//
//    将特征图存入文件中。
//
int CFeatureSet::SaveText(FILE* fp)
{
	// 保存观测器姿态
	fprintf(fp, "%f\t%f\t%f\n", m_pstObserver.x, m_pstObserver.y, m_pstObserver.fThita);

	return CPointFeatureSet::SaveText(fp);
}

//
//   从二进制文件中装入特征集合。
//
int CFeatureSet::LoadBinary(FILE* fp)
{
	Clear();

	// 装入观测器姿态
	float f[3];
	if (fread(f, sizeof(float), 3, fp) != 3)
		return 0;

	m_pstObserver.x = f[0];
	m_pstObserver.y = f[1];
	m_pstObserver.fThita = f[2];

	return CPointFeatureSet::LoadBinary(fp);
}

//
//   将特征集合存入二进制文件中。
//
int CFeatureSet::SaveBinary(FILE* fp)
{
	// 保存观测器姿态
	float f[3] = { m_pstObserver.x, m_pstObserver.y, m_pstObserver.fThita };
	if (fwrite(f, sizeof(float), 3, fp) != 3)
		return 0;

	return CPointFeatureSet::SaveBinary(fp);
}

//
//   从原始扫描点云生成特征集合。
//
bool CFeatureSet::CreateFromRawScan(const CScan& Scan, CFeatureCreationParam& Param)
{
	// 生成反光板特征
	if (!CPointFeatureSet::CreateFromScan(Scan, &Param.m_RefParam))
			return false;

	return true;
}

//
//   通过对给定特征模型的采样来生成特征采样集合。
//
bool CFeatureSet::CreateFromFeatureMap(const CFeatureSet& FeatureMap, const CPosture& pstScanner)
{
	return true;
}

//
//   以给定的点为中心，以指定的范围为半径，截取出一个特征子集。
//
bool CFeatureSet::GetSubset(CPnt& ptCenter, float fRange, CFeatureSet& Subset)
{
	// 先截取点特征的子集
	CPointFeatureSet::GetSubset(ptCenter, fRange, Subset);

	// 截取线段特征的子集
	Subset.SetObserverPosture(m_pstObserver);

	return true;
}

//
//   进行坐标正变换。
//
void CFeatureSet::Transform(const CFrame& frame)
{
	m_pstObserver.Transform(frame);
	CPointFeatureSet::Transform(frame);
}

//
//   进行坐标逆变换。
//
void CFeatureSet::InvTransform(const CFrame& frame)
{
	m_pstObserver.InvTransform(frame);
	CPointFeatureSet::InvTransform(frame);
}

#ifdef _MFC_VER
//
//   绘制全局图。
//
void  CFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF clrFeature, bool bShowId)
{
	CPointFeatureSet::Plot(ScrnRef, pDc, clrFeature, clrFeature, bShowId);
}
#endif