#include "stdafx.h"
#include "ShortLineFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CShortLineFeature::CShortLineFeature() 
{
	m_nSubType = SHORT_LINE_FEATURE;
	m_fWidth = 0.1f;
	m_fAngle = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;    // 双面可用
	m_fMaxIncidenceAngle = PI/2;     // 最大可用入射角为90度
	m_nMinNumPoints = 2;             // 最少点数为2
	m_fMaxLength = 300;              // 线段最大长度0.3m
}

//
//   生成一个复本
//
CPointFeature* CShortLineFeature::Duplicate() const
{
	CShortLineFeature* p = new CShortLineFeature;
	*p = *this;
	return p;
}

//
//   设置短直线段特征的参数。
//
void CShortLineFeature::SetParam(float fWidth, float fAngle, int nWhichSideToUse, 
											float fMaxIncidenceAngle, int nMinNumPoints,
											float fMaxLength)
{
	m_fWidth = fWidth;
	m_fAngle = fAngle;
	m_nWhichSideToUse = nWhichSideToUse;
	m_fMaxIncidenceAngle = fMaxIncidenceAngle;
	m_nMinNumPoints = nMinNumPoints;
	m_fMaxLength = fMaxLength;

	// 构造对应的直线段
	CAngle ang(m_fAngle);
	m_ln.Create(GetPntObject(), ang, m_fWidth/2);
	m_ln.Resize(m_fWidth/2, 0);
}

//
//   判断短直线段是否被指定的扫描线照到，并返回扫描点坐标。该函数主要用于仿真。
//
bool CShortLineFeature::HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist)
{
	CPnt pt;
	float f;

	// 判断两条线段是否有交点
	bool bHit = lnRay.IntersectLineAt(m_ln, pt, f);
	
	// 如果有交点，并且入射角合格
	if (bHit && CheckInRay(lnRay))
	{
		ptHit = pt;
		fDist = f;
		return true;
	}

	// 没有交点，或反面不可用，或入射角超限，返回false
	return false;
}

//
//   针对给定的点云，在规定的角度范围内，检测点云中是否含有该短直线段特征，并返回它的中心位置。
//   注意：只有在规定的面及入射角度内有反射。
//
bool CShortLineFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter)
{
	CScanPointCloud* pCloud = pScan->GetScanPointCloudPointer();

	// ------------- 先根据入射面、入射角等分析此特征是否可能被发现  --------------------
	
	// 构造从激光头到特征中心的直线段
	CLine lnRay(pst.GetPntObject(), GetPntObject());
	if (!CheckInRay(lnRay))
		return false;

	// ------------------- 下面开始分析点云是否含有该特征  ----------------------------

	int nFeatureStart = -1;    // 短直线特征段的起点位置
	int nFeatureEnd = -1;      // 短直线特征段的结束位置

	// 计算激光器到此特征的距离
	float fEstimatedDist = DistanceTo(pst.GetPntObject());
	float fLastR;

	// 在给定的范围内搜索此特征
	for (int i = 0; i < pCloud->m_nCount; i++)
	{
		CScanPoint& sp = pCloud->m_pPoints[i];

		// 如果极角小于起始角，直接跳过
		if (sp.a < fStartAngle)
			continue;

		// 如果极角已超过终止角，结束搜索
		else if (sp.a > fEndAngle)
			break;

		// 如果距离误差超过门限，则认为此点不是特征
		else if (fabs(fEstimatedDist - sp.r) > FEATURE_REG_DISTANCE_WINDOW)
		{
			// 如果已在段中，视为此段结束
			if (nFeatureStart >= 0 && nFeatureEnd < 0)
			{
				// 如已满足短直线点数要求，视为发现了一个有效的整段
				if (i - nFeatureStart >= m_nMinNumPoints)
				{
					nFeatureEnd = i - 1;
				}
				// 否则，放弃不完整的段
				else
				{
					nFeatureStart = nFeatureEnd = -1;
				}
			}
			continue;
		}

		// 如果连续找到多个不断续的点，可认为这是一个对应于短直线的点云片段
		else if (nFeatureStart < 0)
		{
			// 记录段的起始位置和当前点的极径
			nFeatureStart = i;
			fLastR = sp.r;
		}

		// 如果点与点之间不连续，则视为当前段结束
		else if (fabs(sp.r - fLastR) > SHORT_LINE_MAX_DIST_CHANGE)
		{
			if (nFeatureEnd < 0)
			{
				// 如已满足短直线点数要求，视为发现了一个有效的整段
				if (i - nFeatureStart >= m_nMinNumPoints)
				{
					nFeatureEnd = i - 1;

					// break - 临时处理!
					break;
				}
				// 否则，放弃不完整的段
				else
				{
					nFeatureStart = nFeatureEnd = -1;
				}
			}
		}

		// 如果连续段过长，说明也不是合格的短直线特征
		else if (i - nFeatureStart > MAX_POINT_NUM_IN_SHORT_LINE)
		{
			nFeatureStart = nFeatureEnd = -1;
		}
		else
		{
			fLastR = sp.r;
		}
	}

	// 如果没找到特征，返回false
	if (nFeatureStart < 0)
		return false;
	else if (nFeatureEnd < 0)
	{
		nFeatureEnd = pCloud->m_nCount - 1;
	}

	// 统计合格点云片段的重心点
	float fSumX = 0;
	float fSumY = 0;
	int nSegCount = nFeatureEnd - nFeatureStart + 1;

	for (int i = nFeatureStart; i <= nFeatureEnd; i++)
	{
		CScanPoint& sp = pCloud->m_pPoints[i];
		fSumX += sp.x;
		fSumY += sp.y;
	}

	// 重心点
	CPnt ptCenterGravity(fSumX / nSegCount, fSumY / nSegCount);

	*ptCenter = ptCenterGravity;
	CLine ln(pst.GetPntObject(), ptCenterGravity);
	CAngle ang = ln.SlantAngle() - pst.GetAngle();

	ptCenter->r = ln.Length();
	ptCenter->a = CAngle::NormAngle2(ang.m_fRad);

	return true;
}

//
//   生成一个复本。
//
CPointFeature* CShortLineFeature::Duplicate()
{
	CShortLineFeature* p = new CShortLineFeature;
	*p = *this;
	return (CPointFeature*)p;
}

//
//   判断指定的入射光线是否可用。
//
bool CShortLineFeature::CheckInRay(CLine& lnRay)
{
	CAngle angReverseRay = !lnRay.SlantAngle();

	// 判断入射光线从哪一面照射到线段上(如果差角小于PI，说明照到正面)
	CAngle angDiff = angReverseRay - m_ln.SlantAngle();
	
	CAngle angNormal;        // 线段法向角

	// 如果从正面入射
	if (angDiff.m_fRad < PI)
	{
		// 如果正面可用，计算法向角
		if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY ||
			 m_nWhichSideToUse == FEATURE_DIR_BOTH_SIDES)
			angNormal = m_ln.SlantAngle() + PI/2;
		else
			return false;    // 正面不可用
	}

	// 如果入射来自反面，要看反面是否允许使用
	else if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
	{
		return false;       // 反面不可用
	}

	// 反面可用
	else
	{
		angNormal = m_ln.SlantAngle() - PI/2;
	}

	// 计算实际入射角，如果入射角不符合要求，说明此入射光线不可用
	float fIncidenceAngle = angReverseRay.GetDifference(angNormal);
	if (fIncidenceAngle > m_fMaxIncidenceAngle)
		return false;
	else
		return true;
}

//
//   从文本文件中装入短直线段特征参数。
//   返回值：
//     -1  : 读取失败
//     >=0 : 点特征的类型编号
//
int CShortLineFeature::LoadText(FILE* fp)
{
	if (CPointFeature::LoadText(fp) < 0)
		return -1;

	float fWidth, fAngle, fMaxIncidenceAngle, fMaxLength;
	int nWhichSideToUse, nMinNumPoints;

	if (fscanf(fp, "%f\t%f\t%d\t%f\t%d\t%f\n", &fWidth, &fAngle, &nWhichSideToUse, 
		&fMaxIncidenceAngle, &nMinNumPoints, &fMaxLength) != 6)
		return -1;
	
	// 设置相应参数
	SetParam(fWidth, fAngle, nWhichSideToUse, fMaxIncidenceAngle, nMinNumPoints,
		fMaxLength);

	return m_nSubType;
}

//
//   将短直线段特征参数保存到文本文件中。
//   返回值：
//     -1  : 写入失败
//     >=0 : 点特征的类型编号
//
int CShortLineFeature::SaveText(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t", m_nSubType, x, y);
	fprintf(fp, "%f\t%f\t%d\t%f\t%d\t%f\n", m_fWidth, m_fAngle, m_nWhichSideToUse, 
		m_fMaxIncidenceAngle, m_nMinNumPoints, m_fMaxLength);

	return m_nSubType;
}

//
//   从二进制文件中装入点特征的参数。
//
int CShortLineFeature::LoadBinary(FILE* fp)
{
	if (CPointFeature::LoadBinary(fp) < 0)
		return -1;

	float fWidth, fAngle, fMaxIncidenceAngle, fMaxLength;
	int nWhichSideToUse, nMinNumPoints;

	if (fread(&fWidth, sizeof(float), 1, fp) != 1)
		return -1;

	if (fread(&fAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fread(&nWhichSideToUse, sizeof(int), 1, fp) != 1)
		return -1;

	if (fread(&fMaxIncidenceAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fread(&nMinNumPoints, sizeof(int), 1, fp) != 1)
		return -1;

	if (fread(&fMaxLength, sizeof(float), 1, fp) != 1)
		return -1;

	// 设置相应参数
	SetParam(fWidth, fAngle, nWhichSideToUse, fMaxIncidenceAngle, nMinNumPoints,
		fMaxLength);

	return m_nSubType;
}

//
//   将点特征的参数保存到二进制文件中。
//
int CShortLineFeature::SaveBinary(FILE* fp)
{
	if (CPointFeature::SaveBinary(fp) < 0)
		return -1;

	if (fwrite(&m_fWidth, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&m_fAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&m_nWhichSideToUse, sizeof(int), 1, fp) != 1)
		return -1;

	if (fwrite(&m_fMaxIncidenceAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&m_nMinNumPoints, sizeof(int), 1, fp) != 1)
		return -1;

	if (fwrite(&m_fMaxLength, sizeof(float), 1, fp) != 1)
		return -1;

	return m_nSubType;
}

//
//   进行坐标正变换。
//
void CShortLineFeature::Transform(const CFrame& frame)
{
	CPointFeature::Transform(frame);
	m_ln.Transform(frame);
}

//
//   进行坐标逆变换。
//
void CShortLineFeature::InvTransform(const CFrame& frame)
{
	CPointFeature::InvTransform(frame);
	m_ln.InvTransform(frame);
}

#ifdef _MFC_VER

//
//   在屏幕上绘制此点特征。
//
void CShortLineFeature::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF crColor, COLORREF crSelected, int nSize)
{
//	m_ptCenter.Draw(ScrnRef, pDC, RGB(255, 0, 255), 5);

	CPen Pen(PS_SOLID, 3, crColor);
	CPen* pOldPen = pDc->SelectObject(&Pen);

	// 绘制直线段
	CPnt& ptStart = m_ln.m_ptStart;
	CPnt& ptEnd = m_ln.m_ptEnd;

	CPoint pnt1 = ScrnRef.GetWindowPoint(ptStart);
	CPoint pnt2 = ScrnRef.GetWindowPoint(ptEnd);

	pDc->MoveTo(pnt1);
	pDc->LineTo(pnt2);

	pDc->SelectObject(pOldPen);

}
#endif
