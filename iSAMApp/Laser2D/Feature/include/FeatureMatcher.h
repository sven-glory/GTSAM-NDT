#ifndef __CFeatureMatcher
#define __CFeatureMatcher

#include <vector>
#include "Geometry.h"
#include "LocalizationParam.h"
#include "PointMatchListSet.h"
#include "LineMatchList.h"
#include "CoorTransEquations.h"
#include "FeatureSet.h"
#include "PointMatcher.h"
#include "LineMatcher.h"
#include "Frame.h"

using namespace std;

#define MATCH_WITH_POINTS_ONLY                    1
#define MATCH_WITH_LINES_ONLY                     2
#define MATCH_WITH_ALL_FEATURES                   3

#define MATCH_OPTION                              MATCH_WITH_POINTS_ONLY

#define FM_OK                                      1
#define FM_FEATURES_NOT_ENOUGH                    -2
#define FM_MATRIX_ERROR                           -3
#define FM_NOT_MATCHED                            -4
#define FEATURE_MATCH_UNKOWN_ERROR                -10

//////////////////////////////////////////////////////////////////////////////
//   “特征匹配类”的说明。
class CFeatureMatcher
{
private:
	CPosture            m_pstOdometry;
	CLocalizationParam  m_Param;        // 参数
	CCoorTransEquations  m_Equations;
	CTransform m_trans;

public:
	CPointMatcher     m_PointMatcher;
	CLineMatcher      m_LineMatcher;

private:
	// 进行快速匹配
	int QuickMatch(CTransform& trans, int nOption);

	// 进行本地局部匹配
	int LocalMatch(CTransform& trans, int nOption);

public:
	CFeatureMatcher() : m_Equations(MAX_LINE_MATCH_COUNT * 3, 4) 
	{
	}

	// 设置定位参数
	void SetLocalizationParam(CLocalizationParam& Param) { m_Param = Param; }

	// 设置参考特征图
	void SetRefFeatures(CPointFeatureSet* pPointFeatures, CLineFeatureSet* pLineFeatures);

	// 设置当前特征
	void SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures, 
		CLineFeatureSet* pLineFeatures);

	// 进行匹配
	int Match(CTransform& trans, int nOption = FM_OPTION_LOCALIZATION);

	CPointMatchList& GetPointMatchList() { return m_PointMatcher.GetPointMatchList(); }

	bool Load(FILE* fp);
	bool Save(FILE* fp);
};
#endif
