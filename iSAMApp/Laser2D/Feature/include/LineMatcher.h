#ifndef __CLineMatcher
#define __CLineMatcher

#include <vector>
#include "Geometry.h"
#include "LocalizationParam.h"
#include "LineMatchList.h"
#include "CoorTransEquations.h"
#include "FeatureSet.h"

using namespace std;

#define FM_OK                                      1
#define FM_FEATURES_NOT_ENOUGH                    -2
#define FM_MATRIX_ERROR                           -3
#define FM_NOT_MATCHED                            -4
#define FEATURE_MATCH_UNKOWN_ERROR                -10

//////////////////////////////////////////////////////////////////////////////
//   “特征匹配类”的说明。
class CLineMatcher
{
private:
	CPosture             m_pstOdometry;
	CLocalizationParam   m_Param;        // 参数
	CCoorTransEquations  m_Equations;
	CTransform           m_trans;

public:
	CLineFeatureSet      m_RefLineFeatures;
	CLineFeatureSet      m_CurLineFeatures;
	CLineMatchList       m_LineMatchList;

public:
	int QuickRegisterLines();

	// 进行快速匹配
	int QuickMatch(CTransform& trans);

	// 进行本地局部匹配
	int LocalMatch(CTransform& trans);

public:
	CLineMatcher() : m_Equations(40, 4) 
	{
	}

	// 设置定位参数
	void SetLocalizationParam(CLocalizationParam& Param) { m_Param = Param; }

	// 设置参考特征图
	void SetRefFeatures(CLineFeatureSet* pLineFeatures);

	// 设置当前特征
	void SetCurFeatures(CPosture& pstOdometry, CLineFeatureSet* pLineFeatures);

//	CPointMatchList& GetPointMatchList() { return m_PointMatcher.GetPointMatchList(); }

	bool Load(FILE* fp);
	bool Save(FILE* fp);
};
#endif
