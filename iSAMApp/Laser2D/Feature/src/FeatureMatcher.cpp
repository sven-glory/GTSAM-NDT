#include <stdafx.h>
#include "FeatureMatcher.h"
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
void CFeatureMatcher::SetRefFeatures(CPointFeatureSet* pPointFeatures, CLineFeatureSet* pLineFeatures)
{
	m_PointMatcher.SetRefFeatures(pPointFeatures);
	m_LineMatcher.SetRefFeatures(pLineFeatures);
}

//
//   设置当前特征图。
//
void CFeatureMatcher::SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures,
	CLineFeatureSet* pLineFeatures)
{
	m_pstOdometry = pstOdometry;

	m_PointMatcher.SetCurFeatures(pstOdometry, pPointFeatures);
	m_LineMatcher.SetCurFeatures(pstOdometry, pLineFeatures);
}

//
//   进行快速匹配，找到从“局部特征”-->“全局特征”的坐标变换。
//   返回值：
//      > 0 : 匹配成功
//      < 0 : 错误代码(-2:特征不足; -3:方程矩阵异常)
//
int CFeatureMatcher::QuickMatch(CTransform& trans, int nOption)
{
#if MATCH_OPTION == MATCH_WITH_POINTS_ONLY
	m_PointMatcher.SetMatchOption(nOption);
	return m_PointMatcher.QuickMatch(trans);
	
#elif MATCH_OPTION == MATCH_WITH_LINES_ONLY
	return m_LineMatcher.QuickMatch(trans);
#endif
}

//
//   进行本地局部匹配。
//
int CFeatureMatcher::LocalMatch(CTransform& trans, int nOption)
{
#if MATCH_OPTION == MATCH_WITH_POINTS_ONLY
	m_PointMatcher.SetMatchOption(nOption);
	return m_PointMatcher.LocalMatch(trans);

#elif MATCH_OPTION == MATCH_WITH_LINES_ONLY
	return -2;
#endif
}

//
//   进行匹配，找到从“局部特征”-->“全局特征”的坐标变换。
//   返回值：
//      > 0 : 匹配成功
//      < 0 : 错误代码(-2:特征不足; -3:方程矩阵异常)
//
int CFeatureMatcher::Match(CTransform& trans, int nOption)
{
	// 先尝试进行快速匹配
	int nResult = QuickMatch(trans, nOption);
	if (nResult >= 0)
		return nResult;
	else
		return LocalMatch(trans, nOption);
}

bool CFeatureMatcher::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\n", &m_pstOdometry.x, &m_pstOdometry.y, &m_pstOdometry.fThita);

	m_PointMatcher.Load(fp);              // 注意：此处读入的格式有变化！！
	m_LineMatcher.Load(fp);

	return true;
}

bool CFeatureMatcher::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\n", m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);
	fprintf(fp, "0\n");

	m_PointMatcher.Save(fp);
	m_LineMatcher.Save(fp);

	return true;
}
