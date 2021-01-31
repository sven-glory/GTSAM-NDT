#include "stdafx.h"
#include <time.h>
#include "SimuScanner.h"
#include "ScanPointCloud.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define NOISE_RATIO       0.01f

///////////////////////////////////////////////////////////////////////////////

BOOL CSimuScanner::LoadWorldModel(char* strFileName)
{
	FILE* fp = fopen(strFileName, "rt");
	if (fp == NULL)
		return FALSE;

	fscanf(fp, "%d\t%d\n", &m_nRefLineCount, &m_nCircleCount);

	for (int i = 0; i < m_nRefLineCount; i++)
	{
		int nX1, nY1, nX2, nY2;

		fscanf(fp, "%d\t%d\t%d\%d\n", &nX1, &nY1, &nX2, &nY2);
		/*float fX1, fY1, fX2, fY2;
		fX1 = (float)nX1 / 1000;
		fX2 = (float)nX2 / 1000;
		fY1 = (float)nY1 / 1000;
		fY2 = (float)nY2 / 1000;*/
		m_ln[i] = CLine(CPnt(nX1, nY1), CPnt(nX2, nY2));
		//m_ln[i] = CLine(CPnt(fX1, fY1), CPnt(fX2, fY2));
	}

	for (int i = 0; i < m_nCircleCount; i++)
	{
		int nX1, nY1, nRadius;

		fscanf(fp, "%d\t%d\t%d\n", &nX1, &nY1, &nRadius);
		m_circle[i] = CCircle(CPnt(nX1, nY1), nRadius / 1.0f);
	}

	if (!m_MovingObjSet.Load(fp))
		return FALSE;

	fclose(fp);

	srand( (unsigned)time( NULL ) );

	m_MovingObjSet.Start();
////////////////////////

	CScanPointCloud* pCloud = new CScanPointCloud(m_nRefLineCount * 2);

	for (int i = 0; i < m_nRefLineCount; i++)
	{
		pCloud->m_pPoints[2*i].x = m_ln[i].m_ptStart.x;
		pCloud->m_pPoints[2*i].y = m_ln[i].m_ptStart.y;

		pCloud->m_pPoints[2*i+1].x = m_ln[i].m_ptEnd.x;
		pCloud->m_pPoints[2*i+1].y = m_ln[i].m_ptEnd.y;
	}
	m_fLeftMost = pCloud->LeftMost();
	m_fRightMost = pCloud->RightMost();
	m_fTopMost = pCloud->TopMost();
	m_fBottomMost = pCloud->BottomMost();

	m_ptCenter.x = (m_fLeftMost + m_fRightMost)/2;
	m_ptCenter.y = (m_fTopMost + m_fBottomMost)/2;

	m_fSizeX = pCloud->Width();
	m_fSizeY = pCloud->Height();

	delete pCloud;

	return TRUE;
}

void CSimuScanner::AddNoise(CPnt& ptIn, float fNoise, CPnt& ptOut)
{
	int nAng = rand() % 360;
	CAngle ang(nAng*PI/180);
	
	int nRadius = (int)(fNoise);
	if (nRadius == 0)
		nRadius = 1;
	nRadius = rand() % nRadius;
	if (nRadius == 0)
		nRadius = 1;

	float fRadius = nRadius;

	CLine ln(ptIn, ang, fRadius);
	ptOut = ln.m_ptEnd;
}

void CSimuScanner::AddNoise(float fDistIn, float& fDistOut)
{
	float fNoise = fDistIn * NOISE_RATIO;
	int nAmp = (int)(fNoise);
	int nNoise = (nAmp == 0) ? 0 : rand() % nAmp;
	nNoise -= nAmp/2;
	fDistOut = fDistIn + nNoise;
}

//
//   进行一次扫描。
//
int CSimuScanner::Scan(CPnt& ptScanner, float fStartAng, float fEndAng)
{
	int nDetectCount = 0;
	int nNum = (int)((fEndAng - fStartAng) / m_fAngReso);

	for (int i = 0; i < nNum; i++)
	{
		CAngle ang(fStartAng + i * m_fAngReso);
		CLine ScanLine(ptScanner, ang, DEFAULT_SCAN_MAX_RANGE);

		BOOL bFound = FALSE;
		float fMinDist = DEFAULT_SCAN_MAX_RANGE;
		CPnt ptClosest;

		for (int j = 0; j < m_nRefLineCount; j++)
		{
			float fDist;
			CPnt pt;
			if (ScanLine.IntersectLineAt(m_ln[j], pt, fDist))
			{
				if (fDist < fMinDist)
				{
					bFound = TRUE;
					ptClosest = pt;
					fMinDist = fDist;
				}
			}
		}

		for (int j = 0; j < m_nCircleCount; j++)
		{
			float fDist;
			CPnt ptNear;
			if (m_circle[j].IntersectLineAt(ScanLine, ptNear, fDist))
			{
				if (fDist < fMinDist)
				{
					bFound = TRUE;
					ptClosest = ptNear;
					fMinDist = fDist;
				}
			}
		}

		for (int j = 0; j < m_MovingObjSet.m_nCount; j++)
		{
			float fDist;
			CPnt ptNear;
			if (m_MovingObjSet.m_Obj[j].m_Circle.IntersectLineAt(ScanLine, ptNear, fDist))
			{
				if (fDist < fMinDist)
				{
					bFound = TRUE;
					ptClosest = ptNear;
					fMinDist = fDist;
				}
			}
		}

		// 如果发现扫描返回点，则记录最近的那个点
		if (bFound)
		{
			float fMinDist1 = fMinDist;
			AddNoise(fMinDist, fMinDist1);
			m_fDist[nDetectCount] = fMinDist1;
		}
		else
			m_fDist[nDetectCount] = DEFAULT_SCAN_MAX_RANGE;

		m_nDist[nDetectCount] = (int)(m_fDist[nDetectCount]);

		TRACE(_T("%.3f "), m_fDist[nDetectCount]/1000.0f);
		nDetectCount++;
	}

	TRACE(_T("\n"));
	return nDetectCount;
}

void CSimuScanner::GetWorldSize(CPnt& ptCenter, float& fSizeX, float& fSizeY)
{
	CScanPointCloud* pCloud = new CScanPointCloud(m_nRefLineCount * 2);

	for (int i = 0; i < m_nRefLineCount; i++)
	{
		pCloud->m_pPoints[2*i].x = m_ln[i].m_ptStart.x;
		pCloud->m_pPoints[2*i].y = m_ln[i].m_ptStart.y;

		pCloud->m_pPoints[2*i+1].x = m_ln[i].m_ptEnd.x;
		pCloud->m_pPoints[2*i+1].y = m_ln[i].m_ptEnd.y;
	}
	float fLeftMost = pCloud->LeftMost();
	float fRightMost = pCloud->RightMost();
	float fTopMost = pCloud->TopMost();
	float fBottomMost = pCloud->BottomMost();

	ptCenter.x = (fLeftMost + fRightMost)/2;
	ptCenter.y = (fTopMost + fBottomMost)/2;

	fSizeX = pCloud->Width();
	fSizeY = pCloud->Height();

	delete pCloud;
}

void CSimuScanner::DrawWorldModel(CScreenReference& ScrnRef, CDC* pDc, COLORREF color)
{
	CPen Pen(PS_SOLID, 1, color);
	CPen* pOldPen = pDc->SelectObject(&Pen);

	for (int i = 0; i < m_nRefLineCount; i++)
	{
		CPnt ptStart = m_ln[i].m_ptStart;
		CPnt ptEnd = m_ln[i].m_ptEnd;

		CPoint pnt1 = ScrnRef.GetWindowPoint(ptStart);
		CPoint pnt2 = ScrnRef.GetWindowPoint(ptEnd);

		pDc->MoveTo(pnt1);
		pDc->LineTo(pnt2);

	}
	pDc->SelectObject(pOldPen);

	for (int i = 0; i < m_nCircleCount; i++)
		m_circle[i].Draw(ScrnRef, pDc, color, 1);

	m_MovingObjSet.NewDraw();
}

void CSimuScanner::DrawMovingObj(CScreenReference& ScrnRef, CDC* pDc, COLORREF color)
{
	m_MovingObjSet.Draw(ScrnRef, pDc, color, 1);
}