#ifndef __CoxInfo
#define __CoxInfo

#include "scan.h"
#include "LineFeatureSet.h"

//
//   CPointCoxInfo�ඨ����һ��ɨ����Cox���ݡ�
//
class CPointCoxInfo
{
public:
	CScanPoint* m_pScanPoint;        // ָ��ɨ����ָ��
	CLineFeature*       m_pLine;             // ɨ�������Ӧ��ֱ�߶ε�ָ��
	float       m_dDist;             // ɨ��㵽ֱ�ߵľ���ֵ
	CPoint2d    m_ptTarget;          // ɨ��㵽ֱ�߶��������

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
	// ���캯��
	CCoxInfoArray();

	// ��������
	~CCoxInfoArray();

	BOOL Create(int nSize);

	CLineFeature* GetMatchingLine(const CPoint2d& pt);

	CCoxInfoArray* Duplicate();

	void Dump();
};
#endif
