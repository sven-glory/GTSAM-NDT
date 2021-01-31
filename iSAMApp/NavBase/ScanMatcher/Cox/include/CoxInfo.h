#ifndef __CoxInfo
#define __CoxInfo

#include "scan.h"
#include "LineFeatureSet.h"

//
//   CPointCoxInfo类定义了一个扫描点的Cox数据。
//
class CPointCoxInfo
{
public:
	CScanPoint* m_pScanPoint;        // 指向扫描点的指针
	CLineFeature*       m_pLine;             // 扫描点所对应的直线段的指针
	float       m_dDist;             // 扫描点到直线的距离值
	CPoint2d    m_ptTarget;          // 扫描点到直线段上最近点

public:
	CPointCoxInfo()
	{
		m_pScanPoint = NULL;
		m_pLine = NULL;
	}
};

class CCoxInfoArray
{
public:
	CPointCoxInfo* m_pCoxBuf;
	int            m_nSize;
	int            m_nMatchingCount;

public:
	// 构造函数
	CCoxInfoArray();

	// 析构函数
	~CCoxInfoArray();

	BOOL Create(int nSize);

	CLineFeature* GetMatchingLine(const CPoint2d& pt);

	CCoxInfoArray* Duplicate();

	void Dump();
};
#endif
