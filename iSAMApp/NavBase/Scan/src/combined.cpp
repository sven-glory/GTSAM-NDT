#include "stdafx.h"
#include "misc.h"
#include "matrix.h"
#include "scan.h"
#include "filter.h"
#include "LineFeatureSet.h"
#include "sl.h"
#include "CoxMatcher.h"
#include "idc.h"
#include "combined.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

int nPreferedMethod = 1;

CCoxMatcher cd;

#define DEFAULT_MAXITERATIONS 60 //20 
#define DEFAULT_EPSD 1.0 //2.0
#define DEFAULT_EPSA DEG2RAD(0.05) //DEG2RAD(0.1)

short slMaxIterations = DEFAULT_MAXITERATIONS;
float slEpsD = DEFAULT_EPSD, slEpsA = DEFAULT_EPSA;


///////////////////////////////////////////////////////////////////////////////

/*
** Same as slCoxScanMatch but you can provide line scans for
** better speed.
*/
int slCoxScanMatch2(CScan *refScan, CScan *curScan, PoseGauss *match, float *error)
{
	return cd.Match(refScan, curScan, match, error);
}

///////////////////////////////////////////////////////////////////////////////

/* 
** Same as slCombinedScanMatch but you can provide line scans for
** better speed 
*/
int slCombinedScanMatch2(CScan *refScan, CScan *curScan, PoseGauss *match, float *error)
{
	float refp, curp;
	int rc;

	if (!refScan || !curScan)
		return SLERR_NOSCANS;

	// ������ԡ��ο��������͡���ǰɨ�衱������ֱ�߶��ܳ����������㼯�ܻ��Ƴ��ȵı�ֵ
	refp = refScan->m_pLineFeatures->TotalLength() / refScan->TotalLength();
	curp = curScan->m_pLineFeatures->TotalLength() / curScan->TotalLength();

	// ��������е�ֱ�߱Ƚ϶�(��������͡�����)�������Cox�㷨
	if (nPreferedMethod == 0)
	{
		// �������ɨ�輯����ֱ�߶�����������ϴ�����Բ���Cox�㷨
		if (MIN(refp, curp) > SLCOMBINED_LINE_PERCENTAGE) 
		{
			rc = slCoxScanMatch2(refScan, curScan, match, error);
		}
		// ��������е�ֱ�߱Ƚ���(�ǡ�������͡�����)�������IDC�㷨
		else
		{
//			rc = slIDCScanMatch(refScan, curScan, match, error);
			rc = -1;
		}
	}
	else if (nPreferedMethod == 1)
		rc = slCoxScanMatch2(refScan, curScan, match, error);
	else if (nPreferedMethod == 2)
	{
		rc = slIDCScanMatch(refScan, curScan, match, error);
	}
	return rc;
}

/*
** returns OK or error code.
** On OK match will hold pose prob of scan match relative to
** pose of refScan, error reflects how good the match is: near 0 -> good.
*/
int slCombinedScanMatch(CScan *refScan, CScan *curScan, PoseGauss *match, float *error)
{
	int rc;

	DWORD dwStart = GetTickCount();

	if (!refScan || !curScan)
		return SLERR_NOSCANS;

	rc = slCombinedScanMatch2(refScan, curScan, match, error);
//	TRACE("Match result: %.2f, %.2f, %.2f\n", match->m_pstMean.x, match->m_pstMean.y, match->m_pstMean.fThita /PI*180);

//	TRACE("Time elapsed = %dms\n\n", (GetTickCount() - dwStart));

	return rc;
}

