#include <stdafx.h>
#include "PointMatcher.h"
#include "Misc.h"
#include "DebugTrace.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   设置参考特征图。
//
void CPointMatcher::SetRefFeatures(CPointFeatureSet* pPointFeatures)
{
	m_pRefPointFeatures = pPointFeatures;
	if (pPointFeatures == NULL)
		return;

	// 在此给所有参考点赋予ID号，以便后面进行匹配时形成匹配对
	for (int i = 0; i < m_pRefPointFeatures->size(); i++)
		m_pRefPointFeatures->at(i)->id = i;
}

//
//   设置当前特征图。
//
void CPointMatcher::SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures)
{
	m_pstOdometry = pstOdometry;
	m_pCurPointFeatures = pPointFeatures;

	if (pPointFeatures == NULL)
		return;

	// 在此给所有当前点赋予ID号，以便后面进行匹配时形成匹配对
	for (int i = 0; i < m_pCurPointFeatures->size(); i++)
		m_pCurPointFeatures->at(i)->id = i;
}

//
//   坐标变换方程组的特殊处理过程。当变量为(x, y, sin(thita), cos(thita))的坐标变换方程组采用
//   最小二乘法来进行求解时，如果sin(thita)或cos(thita)的值大于1，结果将不正确。在此情况下，通过
//   强制sin(thita)或cos(thita)为1，可以另外得到一组程，通过最小二乘法求解这个方程组的最优解，
//   得到坐标变换矩阵。
//
bool CPointMatcher::SpecialProcess(float fSin, float fCos, float& x, float& y)
{
	float a[2][2], b[2];
	float f[2];

	int nPointPairs = m_PointMatchList.GetCount();
	CLinearEquations* pLsm = new CLinearEquations(nPointPairs * 2 + 1, 2);

	pLsm->Start();

	for (int i = 0; i < nPointPairs; i++)
	{
		// 根据点对生成两行矩阵数据
		m_PointMatchList.GetPair(i).MakeLeastSquareData1(a, b, fSin, fCos);

		// 将这两行数据加入最小二乘求解器中
		if (!pLsm->AddRow(a[0], b[0]))
			return false;

		if (!pLsm->AddRow(a[1], b[1]))
			return false;
	}

	bool bResult = pLsm->LeastSquareSolve(f, 2);

	x = f[0];
	y = f[1];

	delete pLsm;

	return bResult;
}

//
//   进行快速匹配，找到从“局部特征”-->“全局特征”的坐标变换。
//   返回值：
//      > 0 : 匹配成功
//      < 0 : 错误代码(-2:特征不足; -3:方程矩阵异常)
//
int CPointMatcher::QuickMatch(CTransform& trans)
{
	// 分别对点和直线段进行配准
	QuickRegisterPoints();

	// 仅当发现的匹配点对不少于3对时，才视为配准成功
	if (m_PointMatchList.GetCount() < 2)
		return FM_FEATURES_NOT_ENOUGH;

	// 进行点集匹配
	if (!m_PointMatchList.FindTransform())
		return FM_MATRIX_ERROR;

	trans = m_PointMatchList.GetTransform();

	return FM_OK;
}

//
//   进行本地局部匹配。
//
int CPointMatcher::LocalMatch(CTransform& trans)
{
	// 清空配准表
	m_PointMatchList.Clear();

	for (int i = 0; i < LINE_COMPARE_COUNT; i++)
	{
		if (CoarseRegister(m_Param.m_fLineEqualLimit[i]))
			break;
	}

	// 如果没发现匹配，定位失败
	if (m_PointMatchListSet.GetCount() == 0)
		return FM_NOT_MATCHED;                // 找到不到匹配集

	// 在有解的情况下，优选出最优解
	for (int i = 0; i < m_PointMatchListSet.GetCount(); i++)
		m_PointMatchListSet.GetList(i).FindTransform();

	// 在匹配表集合中选出最优的那一个
	int index = m_PointMatchListSet.FindBestMatch();
	if (index >= 0)
	{
		m_PointMatchList = m_PointMatchListSet.GetList(index);                 // 确定了最优匹配表
		trans = m_PointMatchList.m_Trans;                              // 返回坐标变换
	}
	else
		return FM_FEATURES_NOT_ENOUGH;      // 匹配对数小于5

	return FM_OK;
}

//
//   进行匹配，找到从“局部特征”-->“全局特征”的坐标变换。
//   返回值：
//      > 0 : 匹配成功
//      < 0 : 错误代码(-2:特征不足; -3:方程矩阵异常)
//
int CPointMatcher::Match(CTransform& trans)
{
	// 先尝试进行快速匹配
	int nResult = QuickMatch(trans);
	if (nResult >= 0)
		return nResult;
	else
		return LocalMatch(trans);
}

//
//   对所有的点特征进行快速配准，并生成按极径从小到大顺序的匹配表。
//
bool CPointMatcher::QuickRegisterPoints()
{
	// 清空配准表
	m_PointMatchList.Clear();

	// 逐点进行配准
	for (int i = 0; i < m_pRefPointFeatures->GetCount(); i++)
	{
		// 从参考点集中取出下一点
		CPointFeature* pntWorld = m_pRefPointFeatures->at(i);
		if ((m_nOption == FM_OPTION_LOCALIZATION && !pntWorld->IsEnabled()) ||
			 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pntWorld->IsBad()))
			continue;

		pntWorld->UpdatePolar(m_pstOdometry);

		for (int j = 0; j < m_pCurPointFeatures->GetCount(); j++)
		{
			// 从局部点集中取出下一点
			CPointFeature* pntLocal = m_pCurPointFeatures->at(j);
			if ((m_nOption == FM_OPTION_LOCALIZATION && !pntLocal->IsEnabled()) ||
				 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pntLocal->IsBad()))
				continue;

			// 计算出极径和极角差距
			float fRangeDiff = fabs(pntWorld->r - pntLocal->r);
			float fAngleDiff = AngleDiff(pntWorld->a, pntLocal->a);
			float fDist = pntLocal->DistanceTo(*pntWorld);

			// 计算由于车体运动引起的观察角变化(它需要与m_fAngleEqualLimit一起作为角度差别门限)
			float fAng = 0;// (float)fabs(0.3f / pntWorld->r);

			// 仅当极径在规定的范围内，且极径差、极角差不超过规定的门限时，才视为找到一对配准点
			if (pntLocal->r < m_Param.m_fMaxRange &&
				pntLocal->r > m_Param.m_fMinRange &&
				fAngleDiff < (m_Param.m_fAngleEqualLimit + fAng) &&
				fRangeDiff < m_Param.m_fRangeEqualLimit &&
				fDist < m_Param.m_fRangeEqualLimit)
			{
				// 向匹配表中添加此点对(按极径从小到大顺序)
				CPointMatchPair pair(j, i, *pntLocal, *pntWorld);
				pair.m_ptLocal.id = j;
				pair.m_ptWorld.id = i;
				m_PointMatchList.AddInRadiusOrder(pair);
			}
		}
	}

	// 在此应用NearestN参数，对参与匹配运算的反光板数量进行限制
	m_PointMatchList.LimitMatchPairNum(m_Param.m_nAppliedClosestPoints);
//	m_PointMatchList.Filter();
	return true;
}

//
//   根据当前匹配表中的数据，尝试直接对两个点云进行配准。
//
short CPointMatcher::DirectRegister(CTransform trans, CPointMatchList* tab)
{
	for (int i = 0; i < m_pCurPointFeatures->size(); i++)
	{
		// 从本地扫描点集中取一点
		CPointFeature& pnt1 = *m_pCurPointFeatures->at(i);

		if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt1.IsEnabled()) ||
			 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt1.IsBad()))
			continue;

		// 如果此本地点已在匹配表中，则跳过此点
		if (tab->SearchByLocalPoint(pnt1) >= 0)
			continue;

		// 将该点直接变换到参考点集坐标系中
		CPnt pnt2 = trans.GetWorldPoint(pnt1);

		// 将此点与参考点集中的所有点进行比对，看是不是有与它近乎于重合的点
		for (int j = 0; j < m_pRefPointFeatures->size(); j++)
		{
			CPointFeature& pnt3 = *m_pRefPointFeatures->at(j);

			if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt3.IsEnabled()) ||
				 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt3.IsBad()))
				continue;

			// 如果此世界点已在匹配表中，则跳过此点
			if (tab->SearchByWorldPoint(pnt3) >= 0)
				continue;

#ifdef USE_LENGTH_SQUARE_COMPARE
			float cost = pnt2.Distance2To(pnt3);

			if (cost < (m_Param.m_fSamePointMaxDist*m_Param.m_fSamePointMaxDist))
			{
				CPointMatchPair pair(pnt1, pnt3);
				tab->Add(pair);
				break;
			}
#else
			float cost = pnt2.DistanceTo(pnt3);

			// 如果发现近乎于重合的点，视为发现一对匹配点对
			if (cost < m_Param.m_fSamePointMaxDist)
			{
				CPointMatchPair pair(i, j, pnt1, pnt3);
				tab->Add(pair);
				break;
			}
#endif
		}
	}

	// 看看匹配点对的数量是否足够
	if (tab->GetCount() >= m_Param.m_nLeastMatchCount)
		return 1;

	return 0;
}

//
//   在全局范围内进行粗略地配准，并将结果保存于m_PointMatchListSet中。
//
bool CPointMatcher::CoarseRegister(float equal_limit)
{
	float fEqualLimit, fLineValidLimit;

	m_PointMatchListSet.Clear();

	int nLocalCount = m_pCurPointFeatures->size();
	int nRefCount = m_pRefPointFeatures->GetCount();

	fEqualLimit = equal_limit;
	fLineValidLimit = m_Param.m_fMinLineLen;

	for (int i = 0; i < nLocalCount - 1; i++)
	{
		// 取m_LocalLayer点集中的第一点
		CPointFeature& pnt11 = *m_pCurPointFeatures->at(i);
		if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt11.IsEnabled()) ||
			 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt11.IsBad()))
			continue;

		for (int j = i + 1; j < nLocalCount; j++)
		{
			// 取m_LocalLayer点集中的第二点
			CPointFeature& pnt12 = *m_pCurPointFeatures->at(j);
			if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt12.IsEnabled()) ||
				(m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt12.IsBad()))
				continue;

			for (int m = 0; m < nRefCount - 1; m++)
			{
				// 取world点集中的第一点
				CPointFeature& pnt21 = *m_pRefPointFeatures->at(m);
				if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt21.IsEnabled()) ||
					(m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt21.IsBad()))
					continue;

				for (int n = m + 1; n < nRefCount; n++)
				{
					// 取world点集中的第二点
					CPointFeature& pnt22 = *m_pRefPointFeatures->at(n);
					if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt22.IsEnabled()) ||
						(m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt22.IsBad()))
						continue;

					// dist1为pnt11和pnt12之间的距离
					float dist1 = pnt11.DistanceTo(pnt12);

					// dist2为pnt21和pnt22之间的距离
					float dist2 = pnt21.DistanceTo(pnt22);

					// 计算两条线段的长度差
					float dist_err = (float)fabs(dist1 - dist2);

					// 如两条线段长度差太大，则此两条线段匹配失败
					if (dist_err > fEqualLimit)
						continue;

					// 如这两条线段之中任一线段过短，也不符合匹配条件
					if (dist1 < fLineValidLimit || dist2 < fLineValidLimit)
						continue;

					CPointMatchPair pair1(i, m, pnt11, pnt21);
					CPointMatchPair pair2(j, n, pnt12, pnt22);
					CPointMatchPair pair3(i, n, pnt11, pnt22);
					CPointMatchPair pair4(j, m, pnt12, pnt21);

					// 如点对已在当匹配表中包含，则跳过
					if (m_PointMatchListSet.Search(pair1, pair2) >= 0 || m_PointMatchListSet.Search(pair3, pair4) >= 0)
						continue;

					CLine line1(pnt11, pnt12);
					CLine line2(pnt21, pnt22);

					CTransform trans;
					trans.Create(line2, line1);

					CPointMatchList tab;
					tab.Add(pair1);
					tab.Add(pair2);

					if (DirectRegister(trans, &tab) > 0)
					{
						tab.SetTransform(trans);
						m_PointMatchListSet.Add(tab);
						continue;
					}

					CLine line3(pnt22, pnt21);
					trans.Create(line3, line1);

					tab.Clear();
					tab.Add(pair3);
					tab.Add(pair4);

					if (DirectRegister(trans, &tab) > 0)
					{
						tab.SetTransform(trans);
						m_PointMatchListSet.Add(tab);
					}
				}
			}
		}
	}

	return (m_PointMatchListSet.GetCount() >= 3);
}

bool CPointMatcher::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\n", &m_pstOdometry.x, &m_pstOdometry.y, &m_pstOdometry.fThita);

	if (m_pRefPointFeatures != NULL)
		delete m_pRefPointFeatures;

	if (m_pCurPointFeatures != NULL)
		delete m_pCurPointFeatures;

	m_pRefPointFeatures = new CPointFeatureSet;
	m_pRefPointFeatures->LoadText(fp);

	m_pCurPointFeatures = new CPointFeatureSet;
	m_pCurPointFeatures->LoadText(fp);

	return true;
}

bool CPointMatcher::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\n", m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);
//	m_pRefPointFeatures->Save(fp);
//	m_pCurPointFeatures->Save(fp);
	return true;
}
