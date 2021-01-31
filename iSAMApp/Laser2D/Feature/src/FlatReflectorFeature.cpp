#include "stdafx.h"
#include "FlatReflectorFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define REFLECTOR_INTENSITY_GATE           5000

///////////////////////////////////////////////////////////////////////////////

//
//   构造函数。
//
CFlatReflectorFeature::CFlatReflectorFeature(float _x, float _y)
{
	x = _x;
	y = _y;
	m_nSubType = FLAT_REFLECTOR_FEATURE;
	m_fWidth = 0.1f;
	m_fAngle = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;    // 仅正面可用
	m_fMaxIncidenceAngle = PI / 2;     // 最大可用入射角为90度
	m_nMinNumPoints = 2;             // 最少点数为2
}

//
//   构造函数。
//
CFlatReflectorFeature::CFlatReflectorFeature() 
{
	m_nSubType = FLAT_REFLECTOR_FEATURE;
	m_fWidth = 0.1f;
	m_fAngle = 0;
	m_nWhichSideToUse = FEATURE_DIR_FRONT_SIDE_ONLY;    // 仅正面可用
	m_fMaxIncidenceAngle = PI / 2;     // 最大可用入射角为90度
	m_nMinNumPoints = 2;             // 最少点数为2
//	m_fMaxLength = 300;              // 线段最大长度0.3m
}

//
//   针对给定的点云，在规定的角度范围内，检测点云中是否含有该短直线段特征，并返回它的中心位置。
//   注意：只有在规定的面及入射角度内有反射。
//
bool CFlatReflectorFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter)
{
	CScanPointCloud* pCloud = pScan->GetScanPointCloudPointer();

	// ------------- 先根据入射面、入射角等分析此特征是否可能被发现  --------------------

	// 构造从激光头到特征中心的直线段
	CLine lnRay(pst.GetPntObject(), GetPntObject());
//	if (!CheckInRay(lnRay))
//		return false;

	int nFeatureStart = -1;    // 反光板特征段的起点位置
	int nFeatureEnd = -1;      // 反光板特征段的结束位置

	// 计算激光器到此特征的距离
	float fEstimatedDist = DistanceTo(pst.GetPntObject());

	// 找到起始点的序号
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
			if (nFeatureStart >= 0)
			{
				nFeatureEnd = i - 1;     // 标记反光板段结束位置

				// break - 临时处理！
				break;
			}
			continue;
		}
		// 如果光强度超过门限，并且极径差不超过门限，则认为是发现了反光板
		else if (sp.m_nIntensity > REFLECTOR_INTENSITY_GATE)
		{
			// 如果是第一次发现
			if (nFeatureStart < 0)
			{
				nFeatureStart = i;
			}

			// 如果第一个段已经结束，却再一次发现反光板，说明开放区内有多个反光板，则此特征不可用
			else if (nFeatureEnd >= 0)
			{
				return false;
			}
		}

		// 如果已发现了特征段，而现在激光扫到了非反光板，则视为一个完整反光板发现完毕
		else if (nFeatureStart >= 0 && nFeatureEnd < 0)
		{
			nFeatureEnd = i - 1;     // 标记反光板段结束位置

////////// 临时处理!!
			break;
		}
	}

	// 如果没找到特征，返回false
	if (nFeatureStart < 0)
		return false;

	int nMiddle = (nFeatureStart + nFeatureEnd)/2;
	CScanPoint& spCenter = pCloud->m_pPoints[nMiddle];

	*ptCenter = spCenter.GetPntObject();
	ptCenter->a = CAngle::NormAngle(ptCenter->a);

	return true;
}

//
//   生成一个复本。
//
CPointFeature* CFlatReflectorFeature::Duplicate() const
{
	CFlatReflectorFeature* p = new CFlatReflectorFeature;
	*p = *this;
	return (CPointFeature*)p;
}


#ifdef _MSC_VER

//
//   在屏幕上绘制此点特征。
//
void CFlatReflectorFeature::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF crColor, COLORREF crSelected, int nSize,
	int nShowDisabled, bool bShowSuggested)
{
	// 淡色显示
	if (!IsEnabled() && nShowDisabled == DISABLED_FEATURE_UNDERTONE)
	{
		BYTE r = GetRValue(crColor) / 3;
		BYTE g = GetGValue(crColor) / 3;
		BYTE b = GetBValue(crColor) / 3;

		crColor = RGB(r, g, b);
	}

	// 在此，支持“微距观察模式”(即在放大倍数很大的情况下，每个点也放大显示)
	nSize = 5;
	if (ScrnRef.m_fRatio > 500)
		nSize = 5 * ScrnRef.m_fRatio / 500;

	// 画出该反光板
	CPnt::Draw(ScrnRef, pDc, crColor, nSize);

	// 如果需要显示“推荐特征”属性，在此添加红色外圈
	if (bShowSuggested && IsSuggested())
		CPnt::Draw(ScrnRef, pDc, RGB(255, 0, 0), nSize, 2);
}
#endif
