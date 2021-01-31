#ifndef __CSimuScanner
#define __CSimuScanner

#include "Geometry.h"
#include "ScrnRef.h"
//#include "Scan.h"
#include "MovingObj.h"

#define SCANNER_RESO       0.50f
#define SCANNER_RESO_RAD   (SCANNER_RESO/180.f*PI)
#define SCAN_COUNT         ((int)(360/SCANNER_RESO))
#define MAX_LINE_NUM       2000
#define MAX_CIRCLE_NUM     1000

#define DEFAULT_SCAN_MAX_RANGE         30000

class CSimuScanner
{
private:
	CPnt m_ptScanner;
	int  m_nRefLineCount;
	int  m_nCircleCount;
	CLine m_ln[MAX_LINE_NUM];
	CCircle m_circle[MAX_CIRCLE_NUM];


	float m_fAngReso;

public:
	int   m_nScanPointCount;
	float m_fDist[SCAN_COUNT];
	short m_nDist[SCAN_COUNT];

	float m_fSizeX;
	float m_fSizeY;
	CPnt  m_ptCenter;
	float m_fLeftMost;
	float m_fRightMost;
	float m_fTopMost;
	float m_fBottomMost;
	float m_fWidth;
	float m_fHeight;

	CMovingObjSet m_MovingObjSet;

private:
	void AddNoise(CPnt& ptIn, float fNoise, CPnt& ptOut);
	void AddNoise(float fDistIn, float& fDistOut);

public:
	CSimuScanner(float fAngReso = SCANNER_RESO_RAD)
	{
		m_fAngReso = fAngReso;
		VERIFY(LoadWorldModel("world.txt"));
	}

	BOOL LoadWorldModel(char* strFileName);

	int Scan(CPnt& ptScanner, float fStartAng, float fEndAng);

	void GetWorldSize(CPnt& ptCenter, float& fSizeX, float& fSizeY);

	void DrawWorldModel(CScreenReference& ScrnRef, CDC* pDc, COLORREF color);
	
	void DrawMovingObj(CScreenReference& ScrnRef, CDC* pDc, COLORREF color);

	float Width() {return m_fRightMost - m_fLeftMost;}

	float Height() {return m_fTopMost - m_fBottomMost;}

	void Plot(CDC* pDC, COLORREF crColor);

};
#endif
