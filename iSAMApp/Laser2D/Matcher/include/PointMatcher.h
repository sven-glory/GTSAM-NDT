#ifndef __CPointMatcher
#define __CPointMatcher

#include <vector>
#include "Geometry.h"
#include "LocalizationParam.h"
#include "PointMatchListSet.h"
#include "CoorTransEquations.h"
#include "FeatureSet.h"

using namespace std;

#define FM_OK                                      1
#define FM_FEATURES_NOT_ENOUGH                    -2
#define FM_MATRIX_ERROR                           -3
#define FM_NOT_MATCHED                            -4
#define FEATURE_MATCH_UNKOWN_ERROR                -10

#define FM_OPTION_LOCALIZATION                    1       // 定位应用
#define FM_OPTION_EVALUATE_FEATURES               2       // 评估点特征

//////////////////////////////////////////////////////////////////////////////
//   “特征匹配类”的说明。
class CPointMatcher
{
private:
	CPosture            m_pstOdometry;
	CLocalizationParam  m_Param;        // 参数
	CPointMatchListSet  m_PointMatchListSet;
	CCoorTransEquations m_Equations;
	CTransform m_trans;
	int                 m_nOption;
	float               m_fAverageErr;   // 关于对应点的匹配误差
	float               m_fScore;        // 匹配得分(0~1)

public:
	CPointFeatureSet* m_pRefPointFeatures;
	CPointFeatureSet* m_pCurPointFeatures;
	CPointMatchList     m_PointMatchList;

private:
	bool CreateUseFlags();
	void ClearUseFlags();

public:
	bool QuickRegisterPoints();

	// 坐标变换方程组的特殊处理过程
	bool SpecialProcess(float fSin, float fCos, float& x, float& y);
	
	// 进行快速匹配
	int QuickMatch(CTransform& trans);

	// 进行本地局部匹配
	int LocalMatch(CTransform& trans);

	// 根据当前匹配表中的数据，尝试直接对两个点云进行配准
	short DirectRegister(CTransform trans, CPointMatchList* tab);

	// 在全局范围内进行粗略地配准，并将结果保存于m_PointMatchListSet中
	bool CoarseRegister(float equal_limit);

public:
	CPointMatcher() : m_Equations(40, 4) 
	{
		m_pRefPointFeatures = NULL;
		m_pCurPointFeatures = NULL;
		m_nOption = FM_OPTION_LOCALIZATION;
	}

	// 设置匹配选项
	void SetMatchOption(int nOption) { m_nOption = nOption; }

	// 设置定位参数
	void SetLocalizationParam(CLocalizationParam& Param) { m_Param = Param; }

	// 设置参考特征图
	void SetRefFeatures(CPointFeatureSet* pPointFeatures);

	// 设置当前特征
	void SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures);

	// 进行匹配
	int Match(CTransform& trans);

	CPointMatchList& GetPointMatchList() { return m_PointMatchList; }

	bool Load(FILE* fp);
	bool Save(FILE* fp);
};
#endif
