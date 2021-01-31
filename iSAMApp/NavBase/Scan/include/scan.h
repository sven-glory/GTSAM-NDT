#ifndef __CScan
#define __CScan

#include <vector>
#include "ScanPointCloud.h"
#include "matrix.h"
#include "posegauss.h"
#include "misc.h"
#include "ScrnRef.h"
#include "LineFeatureSet.h"
#include <afxwin.h>

using namespace std;

#define DEFAULT_SCAN_MAX_RANGE         30000
#define PROJECT_MAX_REDUCTION          1.0

class CEdgePoint : public CPnt
{
public:
	int m_nEdgeType;

public:
	CEdgePoint() 
	{
		m_nEdgeType = -1;
	}
};

class CCornerPointsPair
{
public:
	int nIndex1;
	int nIndex2;

public:
	CCornerPointsPair(int _nIndex1, int _nIndex2)
	{
		nIndex1 = _nIndex1;
		nIndex2 = _nIndex2;
	}

	bool operator == (const CCornerPointsPair& another)
	{
		if ((nIndex1 == another.nIndex1 && nIndex2 == another.nIndex2) ||
			 (nIndex1 == another.nIndex2 && nIndex2 == another.nIndex1))
			return true;
		else
			return false;
	}
};

///////////////////////////////////////////////////////////////////////////////
//   ��CScan����ʵ���ˡ���ά����ɨ�衱�ĸ��

class CScan : public CScanPointCloud
{
public:
	PoseGauss m_poseScanner;                    /* estimated robot position */
	CPosture  sc_ScannerPos;             /* scanner pos relative to robots pos system */
	float     m_fStartAng;                 /* scanner start scan angle position */
	float     m_fEndAng;                   /* scanner end scan angle position */
	CLineFeatureSet* m_pLineFeatures;
	CLineFeatureSet* m_pLineFeaturesMerged;
	vector<CPnt> m_ptCorners;
	vector<CEdgePoint> m_ptEdges;
	vector<CCornerPointsPair> m_CornerPairs;

	//CCriticalSection         m_crit;

public:
	void SetProperties();

public:
	CScan() {}

	// �������nNum�����CScan����
	CScan(int nNum);

	~CScan();

	// ���ء�=��������
	void operator = (const CScan& Scan);

	// �����������
	void Clear();

	// ����һ���µ�ɨ��
	BOOL Create(int nNum);

	// ��������ֱ������
	bool CreateLineFeatures();
	
	// �������е�����
	bool CreatePointFeatures(/*CPosture& pstScanner*/);

	void SortByAngle();

	// ȡ��ָ��ɨ��ͷ�ľ�����̬
	void GetScannerAbsPos(CPosture& pos);

	// ��������ɨ�����ݵ�ɨ��ͷ֮��ľ���
	float ScannerDistance(CScan& scan2);

	// ����ռ䲢���Ƶ�ǰɨ��
	CScan *Duplicate();

	// ����ǰɨ������һ��ɨ�輯���кϲ�
	void Merge(CScan& scan2, BOOL bNewPointsOnly = TRUE);

	// ��ȫ�����ƶ�ָ���ľ���
	void Move(float dx, float dy);

	// ��ȫ������תָ������̬
	void Rotate(float angle);

	void RotatePos(float angle, float cx, float cy);

	// converts raw range readings to scan, taking into account interlaced scan and
	// robot velocities.  If deinterlace is TRUE then only every interlace-th
   // scan point is generated. 
	CScan* CreateFromPolarRanges(short num, short interlace, bool deinterlace, 
		const USHORT *r, const PoseGauss *robotpos, float tvel, float rvel, 
		const CPosture *relpos, float start_a, float end_a, USHORT maxrange, 
		float scanTime);

	CScan* PolarRangesToScan(short num, const USHORT *r, const PoseGauss *robotPos, 
		const CPosture *relPose, float startA, float endA, USHORT maxRange, const int* m_nIntensityData);

	// �жϸ�ɨ�輯�Ƿ����ָ����ɨ���
	BOOL ContainScanPoint(const CScanPoint& sp, float fThreshHoldDist = 50);

	// �鿴�����ĵ��Ƿ���ֱ�߼���ĳ��ֱ�ߵĶ˵㣬�����ظö˵�
	bool VerifyEdgePoint(CPnt& pt);

	// �жϸ����ĵ��Ƿ���ĳ���ǵ�������
	bool PointTooCloseToSomeCorner(CPnt& pt);

	// ���ļ��ж�ȡɨ������
	BOOL LoadFromFile(FILE* fp);

	// ��ɨ�����ݱ��浽�ļ�
	BOOL SaveToFile(FILE* fp);

	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crColorLine, COLORREF crSelectedLine,
		BOOL bShowPoints, BOOL bShowLineScan, BOOL bShowScanner, int nPointSize = 1, bool bShowInWorldFrame = false);

	// �������ṩ�����ĵ�Ͱ뾶�Ը�ɨ���������
//	void Reduce(CPoint2d& ptCenter, float dRadius);
};

/*
** Sorts scan points in increasing angle.
*/
void SortScanPoints(CScanPoint *sp, long num);

//CScan* NewSimulateScan(CPosture& poseScanner, float dViewAngle = PI*2, CPosture* pAssumeScannerPos = NULL);
CScan* NewSimulateScan(CPosture& poseScanner, float fStartAngle, float fEndAngle, CPosture* pAssumeScannerPos = NULL);

#endif

