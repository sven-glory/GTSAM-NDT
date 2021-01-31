#ifndef __CFeature
#define __CFeature

#include "Geometry.h"
#include "ScrnRef.h"

#define FEATURE_DIR_FRONT_SIDE_ONLY           1        // 只使用短直线正面
#define FEATURE_DIR_BACK_SIDE_ONLY            2        // 只使用短直线反面
#define FEATURE_DIR_BOTH_SIDES                3        // 使用短直线正反两面

#define SIASUN_MATCHER_ANGLE_WINDOW           (15*PI/180)   // +/-15度开放角
#define SIASUN_MATCHER_DIST_WINDOW            (400)         // +/-400mm距离

///////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。

class CFeature
{
public:
	int      m_nType;               // 特征的类型编号
	int      m_nID;                 // 此特征的ID号
	bool     m_bDelete;             // 是否将被删除

public:
	CFeature() 
	{
		m_nType = 0;
		m_nID = 0;
		m_bDelete = false;
	}

	virtual bool Load(FILE* fp) = 0;
	virtual bool Save(FILE* fp) = 0;

#ifdef _MSC_VER
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize) = 0;
#endif
};

#endif
