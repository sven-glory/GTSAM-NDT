#include "stdafx.h"
#include "CoxInfo.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CCoxInfoArray::CCoxInfoArray()
{
	m_pCoxBuf = NULL;
	m_nSize = 0;
	m_nMatchingCount = 0;
}

CCoxInfoArray::~CCoxInfoArray()
{
	if (m_pCoxBuf != NULL)
		delete []m_pCoxBuf;
}

BOOL CCoxInfoArray::Create(int nSize)
{
	ASSERT(m_pCoxBuf == NULL);

	CPointCoxInfo* p = new CPointCoxInfo[nSize];
	if (p == NULL)
		return FALSE;
	else
	{
		m_nSize = m_nMatchingCount = nSize;
		m_pCoxBuf = p;
	}

	return TRUE;
}

CLineFeature* CCoxInfoArray::GetMatchingLine(const CPoint2d& pt)
{
	for (int i = 0; i < m_nMatchingCount; i++)
	{
		if (m_pCoxBuf[i].m_pScanPoint == NULL)
			continue;

		if (m_pCoxBuf[i].m_pScanPoint->DistanceTo(pt) < 200)
		{
			return m_pCoxBuf[i].m_pLine;
		}
	}

	return NULL;
}

CCoxInfoArray* CCoxInfoArray::Duplicate()
{
	CCoxInfoArray* pNewObj = new CCoxInfoArray;
	if (pNewObj->Create(m_nSize) == NULL)
		return NULL;

	for (int i = 0; i < m_nSize; i++)
		pNewObj->m_pCoxBuf[i] = m_pCoxBuf[i];

	return pNewObj;
}

void CCoxInfoArray::Dump()
{
	for (int i = 0; i < m_nSize; i++)
	{
		CPointCoxInfo* pCoxInfo = &(m_pCoxBuf[i]);
		if (pCoxInfo->m_pScanPoint != NULL && pCoxInfo->m_pLine != NULL)
		TRACE(_T("Point(%.2f, %.2f) Matching: (%.2f, %.2f) - (%.2f, %.2f)\n"), 
			pCoxInfo->m_pScanPoint->x, 
			pCoxInfo->m_pScanPoint->y, 
			pCoxInfo->m_pLine->m_ptStart.x, 
			pCoxInfo->m_pLine->m_ptStart.y, 
			pCoxInfo->m_pLine->m_ptEnd.x, 
			pCoxInfo->m_pLine->m_ptEnd.y);
	}
}
