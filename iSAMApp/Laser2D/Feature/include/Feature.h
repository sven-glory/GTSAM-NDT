#ifndef __CFeature
#define __CFeature

#include "Geometry.h"
#include "ScrnRef.h"

// 特征类型
#define FEATURE_TYPE_POINT                    1        // 点特征
#define FEATURE_TYPE_LINE                     2        // 直线特征

// 特征的观察方向定义
#define FEATURE_DIR_FRONT_SIDE_ONLY           1        // 只使用直线正面
#define FEATURE_DIR_BACK_SIDE_ONLY            2        // 只使用直线反面
#define FEATURE_DIR_BOTH_SIDES                3        // 使用直线正反两面

#define SIASUN_MATCHER_ANGLE_WINDOW           (15*PI/180)   // +/-15度开放角
#define SIASUN_MATCHER_DIST_WINDOW            (400)         // +/-400mm距离

#define FEATURE_PARAM_NUM                     10       // 预留的扩展参数的数量

// 对于被“禁止”的特征的显示方式
#define DISABLED_FEATURE_NOT_SHOWN            0        // 不显示禁止项
#define DISABLED_FEATURE_UNDERTONE            1        // 淡色显示禁止项
#define DISABLED_FEATURE_NORMAL               2        // 正常显示禁止项

///////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。
class CFeature
{
public:
	int      m_nType;                        // 特征的类型
	int      m_nSubType;                     // 特征的子类型
	int      m_nFeatureId;                   // 特征的编号
	bool     m_bBad;                         // 特征是否是“差评”
	bool     m_bEnabled;                     // 特征的标记数据
	bool     m_bSuggested;                   // 是否是“推荐特征”
	int      m_nParam[FEATURE_PARAM_NUM];    // 供扩展使用的附加整数参数
	float    m_fParam[FEATURE_PARAM_NUM];    // 供扩展使用的附加浮点数参数

public:
	CFeature() 
	{
		m_nType = 0;
		m_nSubType = 0;
		m_nFeatureId = 0;
		m_bBad = false;
		m_bSuggested = false;
		m_bEnabled = true;

		// 扩展参数初始化
		for (int i = 0; i < FEATURE_PARAM_NUM; i++)
		{
			m_nParam[i] = 0;
			m_fParam[i] = 0;
		}
	}

	// 设置特征的类型
	void SetFeatureType(int nType) { m_nType = nType; }

	// 取得特征的类型
	int GetFeatureType() const { return m_nType; }

	// 设置特征的子类型
	void SetSubType(int nSubType) { m_nSubType = nSubType; }

	// 取得特征的子类型
	int GetSubType() const { return m_nSubType; }

	// 判断是否是一个“差评”特征
	bool IsBad() const { return m_bBad; }

	// 判断特征是否被使能
	bool IsEnabled() const { return m_bEnabled; }

	// 使能/禁止该特征
	void Enable(bool bTrueOrFalse) { m_bEnabled = bTrueOrFalse; }	

	// 判断该特征是否是“推荐特征”
	bool IsSuggested() const { return m_bSuggested; }

	// 设置/取消“差评”
	void SetBad(bool bTrueOrFalse) { m_bBad = bTrueOrFalse; }

	// 推荐/不推荐该特征
	void Suggest(bool bTrueOrFalse) { m_bSuggested = bTrueOrFalse; }

	// 设置扩展参数
	bool SetIntParam(int nIdx, int nParam)
	{
		if (nIdx >= FEATURE_PARAM_NUM)
			return false;

		m_nParam[nIdx] = nParam;
		return true;
	}

	// 设置浮点数扩展参数值
	bool SetFloatParam(int nIdx, float fParam)
	{
		if (nIdx >= FEATURE_PARAM_NUM)
			return false;

		m_fParam[nIdx] = fParam;
		return true;
	}

	virtual int LoadText(FILE* fp) { return 0; }
	virtual int SaveText(FILE* fp) { return 0; }
	virtual int LoadBinary(FILE* fp) { return 0; }
	virtual int SaveBinary(FILE* fp) { return 0; }

#ifdef _MSC_VER
//	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize) = 0;
#endif
};
#endif
