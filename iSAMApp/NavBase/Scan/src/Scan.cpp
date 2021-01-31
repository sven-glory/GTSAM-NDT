#include "stdafx.h"
#include "misc.h"
//#include "config.h"
#include "scan.h"
#include "LineFeatureSet.h"
#include "filter.h"
#include "SimuScanner.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


/*extern */CPosture PoseZero;
extern PoseGauss PoseGaussZero;

#define ARROW_LEN     500          // 0.5m

#define MKSHORT(x, y) ((short)(x) + (short)(y) * 256)
#define BDEG2RAD(x) ((x) * PI / 32768.0)
#define RAD2BDEG(x) (long)(NormAngle((float)x) / PI * 32768.0)

/* file format tags */

#define SCAN_BEGIN                "2D-Scan"
#define SCAN_ROBOTPOS             "RobotPos: "
#define SCAN_ROBOTSIGMA           "RobotSigma: "
#define SCAN_RELPOS               "ScannerRelPos: "
#define SCAN_NUM	                "NumPoints: "
#define SCAN_DATA                 "DATA"

#define RANGE_JUMP_GATE            350      // �жϼ����Ƿ������������ֵ
#define MIN_CORNER_ANGLE           (PI/6)   // �γɽǵ������ֱ�ߵ���С�н�
#define MIN_EDGE_ANGLE             (PI/6)   // ɨ�������Ե�����������ɼнǵ���Сֵ
#define MIN_LINE_FEATURE_LEN       450      // ֻ�г����˳��ȵ�ֱ�߶β��������ɽǵ�
#define MAX_HIDDEN_LINE_LEN        5000     // �����ֱ�߶ε��ӳ��߳����˾��룬���ɵĽ��㲻����
#define DIST_IGNORE_SHORT_LINE     10000    // ���ɨ�������ڴ˾��룬�����Է��ֵļ���ֱ��

float ScanMaxRange = DEFAULT_SCAN_MAX_RANGE;
static ulong scanId = 1;

CSimuScanner SimuScanner;

///////////////////////////////////////////////////////////////////////////////

//
//   �������nNum�����CScan����
//
CScan::CScan(int nNum) : CScanPointCloud(nNum)
{
	m_pLineFeatures = NULL;
	m_pLineFeaturesMerged = NULL;
}

CScan::~CScan()
{
	if (m_pLineFeatures != NULL)
		delete m_pLineFeatures;

	if (m_pLineFeaturesMerged != NULL)
		delete m_pLineFeaturesMerged;
}

//
//   ���ء�=��������
//
void CScan::operator = (const CScan& Scan)
{
	*(GetScanPointCloudPointer()) = *(((CScan&)Scan).GetScanPointCloudPointer());

	m_poseScanner = Scan.m_poseScanner;
	m_fStartAng = Scan.m_fStartAng;
	m_fEndAng = Scan.m_fEndAng;

	if (m_pLineFeatures != NULL)
		delete m_pLineFeatures;
	m_pLineFeatures = NULL;

	if (Scan.m_pLineFeatures != NULL)
		m_pLineFeatures = Scan.m_pLineFeatures->Duplicate();


	if (m_pLineFeaturesMerged != NULL)
		delete m_pLineFeaturesMerged;
	m_pLineFeaturesMerged = NULL;

	if (Scan.m_pLineFeaturesMerged != NULL)
		m_pLineFeaturesMerged = Scan.m_pLineFeaturesMerged->Duplicate();

}

//
//   ����������ݡ�
//
void CScan::Clear()
{
	// �����������
	CScanPointCloud::Clear();

	if (m_pLineFeatures != NULL)
	{
		delete m_pLineFeatures;
		m_pLineFeatures = NULL;
	}

	if (m_pLineFeaturesMerged != NULL)
	{
		delete m_pLineFeaturesMerged;
		m_pLineFeaturesMerged = NULL;
	}

	// ���������������
	// ..
}

//
//   ����һ���µ�ɨ�衣
//
BOOL CScan::Create(int nNum)
{
	// �ڴ�Ӧ���������ݳ�Ա��ʼ��Ϊ0
	Clear();

	// Ϊ���Ʒ���ռ�
	if (!CScanPointCloud::Create(nNum))
		return FALSE;

	return TRUE;
}

//
//   ��������ɨ�����ݵ�ɨ��ͷ֮��ľ��롣
//
float CScan::ScannerDistance(CScan& scan2)
{
	CPosture p1, p2;

	GetScannerAbsPos(p1);
	scan2.GetScannerAbsPos(p2);

	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;

	return sqrt(dx*dx + dy*dy);
}

//
//   ȡ��ָ��ɨ��ͷ�ľ�����̬��
//
void CScan::GetScannerAbsPos(CPosture& pos)
{
	const CPosture *p = &m_poseScanner.m_pstMean;
	const CPosture *rel = &sc_ScannerPos;
	float sina = sin(p->fThita);
	float cosa = cos(p->fThita);
	float dx = rel->x;
	float dy = rel->y;
	float da = rel->fThita;

	pos.x = p->x + dx * cosa - dy * sina;
	pos.y = p->y + dx * sina + dy * cosa;
	pos.fThita = p->fThita + da;
}

//
//   ���ݵ������ݼ��㡰ƽ����Χ���͡����Χ����
//
void CScan::SetProperties()
{
	float sum = 0.0;
	float maxd = 0.0;

	for (int i = 0; i < m_nCount; i++)
	{
		float d = m_pPoints[i].r;
		maxd = MAX(d, maxd);
		sum += d;
	}
}


//
//   ����ռ䲢���Ƶ�ǰɨ�衣
//
CScan *CScan::Duplicate()
{
	CScan *scan = new CScan(m_nCount);
	if (scan == NULL)
		return NULL;

	*scan = *this;
	return scan;
}

//
//   ����ǰɨ������һ��ɨ�輯���кϲ���
//
void CScan::Merge(CScan& scan2, BOOL bNewPointsOnly)
{
	CScanPointCloud* p1 = GetScanPointCloudPointer();
	CScanPointCloud* p2 = scan2.GetScanPointCloudPointer();

	// �ȸ���ԭ���ĵ��ơ�
	CScanPointCloud CopyThis(*p1);

	// ���ԭ��������
	p1->Clear();

	// ���·���ռ��Ա������������Ƶ�����
	VERIFY(p1->Create(CopyThis.m_nCount + p2->m_nCount));

	int i;

	// ���ƻ�ԭ������
	for (i = 0; i < CopyThis.m_nCount; i++)
		m_pPoints[i] = CopyThis.m_pPoints[i];

	// ���������µ�������
	int nNewCount = CopyThis.m_nCount;
	for (i = 0; i < p2->m_nCount; i++)
	{
		if (bNewPointsOnly && ContainScanPoint(p2->m_pPoints[i]))
			continue;

		m_pPoints[nNewCount++] = p2->m_pPoints[i];
	}
	m_nCount = nNewCount;
}

//
//   ��ȫ�����ƶ�ָ���ľ��롣
//
void CScan::Move(float dx, float dy)
{
	if ((dx == 0) && (dy == 0))
		return;

	// �ƶ���������
	CScanPointCloud::Move(dx, dy);

	// �ƶ�����ͷ�ľ�ֵ��
	m_poseScanner.m_pstMean.Move(dx, dy);
}

static int spcmp(const void *v1, const void *v2)
{
	const CScanPoint *sp1 = (CScanPoint *)v1, *sp2 = (CScanPoint *)v2;

	/* float da = NormAngle(sp1->a - sp2->a); not a sorting order */
	float da = sp1->a - sp2->a;

	if (da < 0.0)
		return -1;
	else if (da > 0.0)
		return 1;

	return 0;
}

//
//   �ҵ���ɨ��㼯�е����С�ֱ��ɨ���ߡ���
//
bool CScan::CreateLineFeatures()
{
	if (m_pLineFeatures != NULL)
	{
		delete m_pLineFeatures;
		m_pLineFeatures = NULL;
	}

	m_pLineFeatures = new CLineFeatureSet(*this);
//	m_pLineFeatures->Simplify();

	m_pLineFeaturesMerged = m_pLineFeatures->Duplicate();
	m_pLineFeaturesMerged->Simplify();

	return (m_pLineFeatures != NULL);
}

//
//   ���ݸ����ļ���ͷ��̬����ָ֤���ĵ��Ƿ���һ����Ч�ı�Ե�㣬
//   �鿴�����ĵ��Ƿ���ֱ�߼���ĳ��ֱ�ߵĶ˵㣬�����ظñ�Ե�������ֵ��
//
bool CScan::VerifyEdgePoint(CPnt& pt)
{
	CPnt pt1;
	CAngle angScanner(pt.a);

	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		CLineFeature& ln = m_pLineFeatures->m_pLines[i];
	
		if (ln.m_ptStart.DistanceTo(pt) < 30)
		{
			CAngle& ang = ln.SlantAngle();

			// ȡ��ɨ������ֱ�����ɵļн�
			float fAngDiff = ang.GetDifference(angScanner);

			// ����˼н�̫С��������Ϊ��Ч��Ե��
			if (fAngDiff < MIN_EDGE_ANGLE || fAngDiff > PI - MIN_EDGE_ANGLE)
				return false;

			pt = ln.m_ptStart;
			return true;
		}
		else if (ln.m_ptEnd.DistanceTo(pt) < 30)
		{	
			CAngle& ang = !ln.SlantAngle();

			// ȡ��ɨ������ֱ�����ɵļн�
			float fAngDiff = ang.GetDifference(angScanner);

			// ����˼н�̫С��������Ϊ��Ч��Ե��
			if (fAngDiff < MIN_EDGE_ANGLE || fAngDiff > PI - MIN_EDGE_ANGLE)
				return false;

			pt = ln.m_ptEnd;
			return true;
		}
	}

	return false;
}

//
//   �жϸ����ĵ��Ƿ���ĳ���ǵ���������
//
bool CScan::PointTooCloseToSomeCorner(CPnt& pt)
{
	bool bTooClose = false;
	for (vector<CPnt>::iterator iter = m_ptCorners.begin(); iter != m_ptCorners.end(); iter++)
	{
		CPnt& pti = iter->GetPntObject();
		if (pt.DistanceTo(pti) < 200)
		{
			bTooClose = true;
			break;
		}
	}

	return bTooClose;
}

//
//   �������е�������
//   ע�⣺�˺��������������ɹ�����ֱ������֮��(�����ù�CreateLineFeatures()֮��)���ã�
//
bool CScan::CreatePointFeatures(/*CPosture& pstScanner*/)
{
	CPnt ptOrigin(0, 0);

	vector<CLine> Lines;

	// ��������(������������ǵ��)ֱ�߶�
	Lines.clear();

	// �ȸ������е�ֱ�߶ε�Lines��
	for (int i = 0; i < m_pLineFeaturesMerged->m_nCount; i++)
	{
		bool bFoundColinear = false;

		// ȡһ��ֱ��
		CLine& ln1 = m_pLineFeaturesMerged->m_pLines[i].GetLineObject();

		// ���û�з��ֹ������Σ�����ֱ�߼���Lines��
		if (!bFoundColinear)
			Lines.push_back(ln1);
	}

	// �����������������ֱཻ�ߵĽ��㼯�ϣ���Ϊ���ǵ�������
	m_ptCorners.clear();

	for (int i = 0; i < (int)Lines.size() - 1; i++)
	{
		CLine& ln1 = Lines[i];
		float fDist1 = ln1.m_ptStart.DistanceTo(ptOrigin);
		float fDist2 = ln1.m_ptEnd.DistanceTo(ptOrigin);
		float fAveDist = (fDist1 + fDist2) / 2;

#if 0
		if (ln1.Length() < MIN_LINE_FEATURE_LEN && fAveDist > DIST_IGNORE_SHORT_LINE)
		{
			continue;
		}
#endif
		for (int j = i + 1; j < (int)Lines.size(); j++)
		{
			CLine& ln2 = Lines[j];
			float fDist1 = ln2.m_ptStart.DistanceTo(ptOrigin);
			float fDist2 = ln2.m_ptEnd.DistanceTo(ptOrigin);
			float fAveDist = (fDist1 + fDist2) / 2;

#if 0
			if (ln2.Length() < MIN_LINE_FEATURE_LEN && fAveDist > DIST_IGNORE_SHORT_LINE)
			{
				continue;
			}
#endif
			// ��������ֱ�ߵĲ��
			CAngle angDiff = ln1.SlantAngle() - ln2.SlantAngle();
			if ((angDiff > MIN_CORNER_ANGLE && angDiff < (PI - MIN_CORNER_ANGLE)) ||
				 (angDiff > (PI + MIN_CORNER_ANGLE) && angDiff < (2*PI - MIN_CORNER_ANGLE)))
			{
				float x, y;
				bool bOnLine1 = false;
				bool bOnLine2 = false;

				// ��������ֱ�߶εĽ��㣬���жϽ����Ƿ��������߶���
				if (ln1.Intersect(ln2, &x, &y, &bOnLine1, &bOnLine2))
				{
					CPnt pt(x, y);
					float fDist;
					
					// ������㲻��ֱ��1��
					if (!bOnLine1)
					{
						ln1.FindNearPoint(pt, &fDist);
						float fLen1 = ln1.Length();
						
						// ���ں̵ܶ�ֱ��������Ҫ���ӳ����Ȳ��ܳ���ֱ����������ĳ���
						if (fLen1 < MIN_LINE_FEATURE_LEN * 3)
						{
							if (fDist > fLen1)
								continue;
						}
						// ����һ�㳤�ȵ�ֱ���������ӳ����Ȳ��ܳ����涨�Ĺ̶�����(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// ������㲻��ֱ��2��
					if (!bOnLine2)
					{
						ln2.FindNearPoint(pt, &fDist);
						float fLen2 = ln2.Length();

						// ���ں̵ܶ�ֱ��������Ҫ���ӳ����Ȳ��ܳ���ֱ����������ĳ���
						if (fLen2 < MIN_LINE_FEATURE_LEN * 3)
						{
							if (fDist > fLen2)
								continue;
						}

						// ����һ�㳤�ȵ�ֱ���������ӳ����Ȳ��ܳ����涨�Ĺ̶�����(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// ����õ�����ǰ����ĵ�̫�����򲻽�������
					if (!PointTooCloseToSomeCorner(pt))
						m_ptCorners.push_back(pt);
#if 0						
					bool bTooClose = false;
					for (vector<CPoint2d>::iterator iter = m_ptCorners.begin(); iter != m_ptCorners.end(); iter++)
					{
						CPoint2d& pti = iter->GetPoint2dObject();
						if (pt.DistanceTo(pti) < 200)
						{
							bTooClose = true;
							break;
						}
					}

					if (!bTooClose)
						m_ptCorners.push_back(pt);
#endif

				}
			}
		}
	}
	//m_crit.Lock();
	// �������ɱ�Ե������
	m_ptEdges.clear();
	//m_crit.Unlock();
	// �ڴ˼���Ե��
	int nLastJump = 0;   // ��һ����Ծ��
	bool bDetectStartEdge = false;

	float r1 = m_pPoints[0].r;
	for (int i = 1; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];

		// �����������ͻ�䣬˵���¶ο�ʼ
		if (fabs(sp.r - r1) > RANGE_JUMP_GATE)
		{
			// ����¼�����С��˵�������Ƿ�����ĳ���߶ε���ʼ��Ե��
			if (sp.r < r1)
			{
				bDetectStartEdge = true;
			}

			// �������ͻȻ���
			else
			{
				bDetectStartEdge = false;

				// �����ǰ������������������5�������ҵ���һ��������Ե��
				if (i - nLastJump > 20)
				{
					// ��һ����(���i-1)Ϊ������Ե��
					CEdgePoint ept;

					ept.GetPntObject() = m_pPoints[i - 1].GetPntObject();
					ept.m_nEdgeType = 1;   // ������Ե��
					if (VerifyEdgePoint(ept.GetPntObject()))
					{
						if (!PointTooCloseToSomeCorner(ept))
							m_ptEdges.push_back(ept);
					}
				}
			}

			nLastJump = i;
		}

		// �������û��ͻ��
		else
		{
			// ����Ѿ�������ʼ��Ե�㣬�Ҹոս����Ķ�������������5����ȷ������Ǹ���Ե��
			if (bDetectStartEdge && (i - nLastJump > 20))
			{
				// �ڴ˼����µı�Ե��
				CEdgePoint ept;
				ept.GetPntObject() = m_pPoints[nLastJump].GetPntObject();
				ept.m_nEdgeType = 0;    // ��ʼ��Ե��

				if (VerifyEdgePoint(ept.GetPntObject()))
				{
					m_ptEdges.push_back(ept);
				}
				bDetectStartEdge = false;
			}
		}
		r1 = sp.r;
	}

	// �ڴ�������������ʵ�ǵ�֮��������ɵ��߶�
	m_CornerPairs.clear();

	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		// ȡ��ֱ�߶�
		CLine& ln = m_pLineFeatures->m_pLines[i].GetLineObject();

		for (int j = 0; j < (int)m_ptCorners.size() - 1; j++)
		{
			for (int k = j + 1; k < (int)m_ptCorners.size(); k++)
			{
				float fLambda = -1;
				float fDist = ln.DistanceToPoint(true, m_ptCorners[j], &fLambda);
				if (fLambda < -0.05f || fLambda > 1.05f || fDist > 100)
					continue;

				fDist = ln.DistanceToPoint(true, m_ptCorners[k], &fLambda);
				if (fLambda < -0.05f || fLambda > 1.05f || fDist > 100)
					continue;

				m_CornerPairs.push_back(CCornerPointsPair(j, k));
			}
		}
	}

	// �ڴ��ҳ�������һ���ǵ��һ����Ե����ɵ��߶�
	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		// ȡ��ֱ�߶�
		CLine& ln = m_pLineFeatures->m_pLines[i].GetLineObject();

		for (int j = 0; j < (int)m_ptCorners.size(); j++)
		{
			for (int k = 0; k < (int)m_ptEdges.size(); k++)
			{
				float fLambda = -1;
				float fDist = ln.DistanceToPoint(true, m_ptCorners[j], &fLambda);
				if (fLambda < -0.05f || fLambda > 1.05f || fDist > 100)
					continue;

				fDist = ln.DistanceToPoint(true, m_ptEdges[k], &fLambda);
				if (fLambda < -0.05f || fLambda > 1.05f || fDist > 100)
					continue;

				m_CornerPairs.push_back(CCornerPointsPair(j, m_ptCorners.size() + k));
			}
		}
	}

#if 1
	// �ڴ��ҳ�������������Ե����ɵ��߶�
	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		// ȡ��ֱ�߶�
		CLine& ln = m_pLineFeatures->m_pLines[i].GetLineObject();

		for (int j = 0; j < (int)m_ptEdges.size() - 1; j++)
		{
			for (int k = j + 1; k < (int)m_ptEdges.size(); k++)
			{
				float fLambda = -1;
				float fDist = ln.DistanceToPoint(true, m_ptEdges[j], &fLambda);
				if (fLambda < -0.05f || fLambda > 1.05f || fDist > 100)
					continue;

				fDist = ln.DistanceToPoint(true, m_ptEdges[k], &fLambda);
				if (fLambda < -0.05f || fLambda > 1.05f || fDist > 100)
					continue;

				if (m_ptEdges[j].DistanceTo(m_ptEdges[k]) > 300)
					m_CornerPairs.push_back(CCornerPointsPair(m_ptCorners.size() + j, m_ptCorners.size() + k));
			}
		}
	}
#endif

	return true;
}

void CScan::SortByAngle()
{
	CScanPointCloud* sp = GetScanPointCloudPointer();
	qsort(sp->m_pPoints, sp->m_nCount, sizeof(CScanPoint), spcmp);
}


///////////////////////////////////////////////////////////////////////////////

//
//   converts raw range readings to scan structure.
//
CScan* CScan::PolarRangesToScan(short num, const USHORT *r, const PoseGauss *robotPos, 
										  const CPosture *relPose, float startA, float endA, 
										  USHORT maxRange, const int* m_nIntensityData)
{
	return CreateFromPolarRanges(num, 1, FALSE, r, robotPos, 0.0, 0.0, relPose, 
		startA, endA, maxRange, 0.0);
}

//
// converts raw range readings to scan taking into account interlaced scan and
// robot velocities.  If deinterlace is TRUE then only every interlace-th
// scan point is generated.
//
CScan* CScan::CreateFromPolarRanges(short num, short interlace, bool deinterlace, 
											const USHORT *r, const PoseGauss *robotPos, 
											float tvel, float rvel, const CPosture *relPose, 
											float startA, float endA, USHORT maxRange, 
											float scanTime)
{
	float stepA = (endA - startA) / (float)(num);
	float stepT = scanTime * stepA * interlace / (2.0 * PI);

	if (interlace < 1)
		interlace = 1;

	if (robotPos == NULL)
		robotPos = &PoseGaussZero;

	if (relPose == NULL)
		relPose = &PoseZero;

	if (ScanMaxRange < maxRange) 
		maxRange = (USHORT)ScanMaxRange;

	m_poseScanner = *robotPos;

	sc_ScannerPos = *relPose;
	m_fStartAng = startA;
	m_fEndAng = endA;
	m_nCount = 0;

	if (num == 0)
		return this;

	// ȡ�ü���ͷԭ��������
	CPosture scanPose;
	GetScannerAbsPos(scanPose);

	CScanPoint *sp = m_pPoints;

	float d, a;
	long i;

	for (i = 0, a = startA; i < num; i++, a += stepA)
	{
		if (deinterlace && (i % interlace != 0))
			continue;

		d = (float)r[i];
		if (d < maxRange)
		{
			long div = i / interlace;
			long mod = i % interlace;

			// ����ʱ����
			float time = scanTime * (mod - interlace) + div * stepT;

			CPosture movePose, realScanPose;
			float dx, dy;

			// movePoseΪ��ʱ��time�ڼ���ͷ״̬�ı仯��
			movePose.x = tvel * time;
			movePose.y = 0.0;
			movePose.fThita = rvel * time;

			((PoseGauss*)robotPos)->m_pstMean.TransformToGlobal(&movePose, &m_poseScanner.m_pstMean);

			// ȡ�ü���ͷ��ȫ������ϵ�µ���̬
			GetScannerAbsPos(realScanPose);

			// �����ɨ����ƽ��ֱ������
			sp->x = realScanPose.x + d * cos(/*realScanPose.fThita +*/ a);
			sp->y = realScanPose.y + d * sin(/*realScanPose.fThita +*/ a);

			// 
			dx = sp->x - scanPose.x;
			dy = sp->y - scanPose.y;

			// �����ɨ���ļ�����
			sp->a = NormAngle((float)(atan2(dy, dx) - scanPose.fThita));
			sp->r = _hypot(dx, dy);

			sp->m_nWeight = 1;
			sp->m_nLineID = -1;
			sp->m_fScanAng = i * stepA;
			sp++;
		}
	}

	m_nCount = (sp - m_pPoints);
	m_poseScanner = *robotPos;

#if 0
	m_ptEdges.clear();

	// �ڴ˼���Ե��
	int nLastJump = 0;   // ��һ����Ծ��
	bool bDetectStartEdge = false;

	float r1 = m_pPoints[0].r;
	for (int i = 1; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];

		// �����������ͻ�䣬˵���¶ο�ʼ
		if (fabs(sp.r - r1) > 500)
		{
			// ����¼�����С��˵�������Ƿ�����ĳ���߶ε���ʼ��Ե��
			if (sp.r < r1)
			{
				bDetectStartEdge = true;
			}

			// �������ͻȻ���
			else if (sp.r > r1)
			{
				// �����ǰ������������������5�������ҵ���һ��������Ե��
				if (i - nLastJump > 5)
				{
					// ��һ����(���i-1)Ϊ������Ե��
					CPoint2d& pt = m_pPoints[i-1].GetPoint2dObject();
					m_ptEdges.push_back(pt);
				}
			}

			nLastJump = i;
		}

		// �������û��ͻ��
		else
		{
			// ����Ѿ�������ʼ��Ե�㣬�Ҹոս����Ķ�������������5����ȷ������Ǹ���Ե��
			if (bDetectStartEdge && (i - nLastJump > 5))
			{
				// �ڴ˼����µı�Ե��
				CPoint2d& pt = m_pPoints[nLastJump].GetPoint2dObject();
				m_ptEdges.push_back(pt);

				bDetectStartEdge = false;
			}
		}
		r1 = sp.r;
	}
#endif

	SortByAngle();              // �ڴ˰������������ԭ��ɨ��Ǵ��򱻴���
	SetProperties();

	// ����ֱ��ɨ�輯��
//	VERIFY(CreateLineScans());

	return this;
}

//
//   �жϸ�ɨ�輯�Ƿ����ָ����ɨ��㡣
//
BOOL CScan::ContainScanPoint(const CScanPoint& sp, float fThreshHoldDist)
{
	// �жϸõ��Ƿ���ɨ�輯�е�ĳ���������ǳ���(С��ָ��������)
	if (CScanPointCloud::ContainPoint(sp, fThreshHoldDist) >= 0)
		return TRUE;

	// �жϸõ��Ƿ���ɨ�輯�е�ĳ��ֱ�������ǳ���
	if (m_pLineFeatures->ContainScanPoint(sp))
		return TRUE;

	return FALSE;
}
#if 1
//
//   ���ļ��ж�ȡɨ�����ݡ�
//
BOOL CScan::LoadFromFile(FILE* fp)
{
	m_pLineFeatures = NULL;

	if (!CScanPointCloud::LoadFromFile(fp))
		return FALSE;

	return CreateLineFeatures();
}

//
//   ��ɨ�����ݱ��浽�ļ���
//
BOOL CScan::SaveToFile(FILE* fp)
{
	if (!CScanPointCloud::SaveToFile(fp))
		return FALSE;

	return TRUE;
}
#endif

void CScan::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, COLORREF crColorLine, COLORREF crSelectedLine, 
					  BOOL bShowPoints, BOOL bShowLineScan, BOOL bShowScanner, int nPointSize, bool bShowInWorldFrame)
{
	CScreenReference ScrnRef1 = ScrnRef;
	/*ScrnRef1.m_ptCenter.x -= 20000;
	ScrnRef1.m_ptCenter.y -= 10000;*/

	// ȡ��ɨ��ʱ����ͷ����̬
	CPosture& pstRef = m_poseScanner.m_pstMean;
	
	// �����ֲ�����ϵ
	CTransform trans(pstRef.GetPntObject(), pstRef.GetAngle());

	if (bShowPoints)
	{
		CPen pen(PS_SOLID, 1, crColorPoint);
		CPen* pOldPen = pDC->SelectObject(&pen);

		CBrush Brush(RGB(0, 0, 0));
		CBrush* pOldBrush = pDC->SelectObject(&Brush);


		for (int i = 0; i < m_nCount; i++)
		{
			CPnt pt;

			// �����Ҫ����������ϵ����ʾ��������Ҫ��������任
			if (bShowInWorldFrame)
				pt = trans.GetWorldPoint(m_pPoints[i]);
			else
				pt = m_pPoints[i];

			CPoint pnt = ScrnRef.GetWindowPoint(pt);

			CRect r(pnt.x - nPointSize, pnt.y - nPointSize, pnt.x + nPointSize, pnt.y + nPointSize);
			pDC->Ellipse(&r);
		}
		pDC->SelectObject(pOldPen);
		pDC->SelectObject(pOldBrush);
	}

	if (bShowScanner)
	{
		CPen pen(PS_SOLID, 1, RGB(0, 0, 0));
		CPen* pOldPen = pDC->SelectObject(&pen);

		CPnt ptRobot(m_poseScanner.m_pstMean.x, m_poseScanner.m_pstMean.y);
		CAngle ang(m_poseScanner.m_pstMean.fThita);
		CLine lnArrow(ptRobot, ang, ARROW_LEN);
		CPnt ptArrowTip = lnArrow.m_ptEnd;

		CPoint pntRobot = ScrnRef.GetWindowPoint(ptRobot);
		CPoint pntArrowTip = ScrnRef.GetWindowPoint(ptArrowTip);

		CRect r1(pntRobot.x - 5, pntRobot.y - 5, pntRobot.x + 5, pntRobot.y + 5);
		CPoint pntStart(pntRobot.x + 5, pntRobot.y);
		CPoint pntEnd(pntRobot.x + 5, pntRobot.y);
		pDC->Arc(&r1, pntStart, pntEnd);

		pDC->MoveTo(pntRobot);
		pDC->LineTo(pntArrowTip);
		pDC->SelectObject(&pOldPen);
	}



	if (bShowLineScan)
	{
		if (bShowInWorldFrame)
		{
			for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
			{
				CLine& ln = m_pLineFeatures->m_pLines[i].GetLineObject();

				CPnt pt1 = trans.GetWorldPoint(ln.m_ptStart);
				CPnt pt2 = trans.GetWorldPoint(ln.m_ptEnd);

				CLine ln2(pt1, pt2);
				ln2.Draw(ScrnRef, pDC, crColorLine, 2*nPointSize, nPointSize, TRUE);
			}
		}
		else
			m_pLineFeatures->Plot(ScrnRef1, pDC, crColorLine, crSelectedLine, nPointSize);
	}
#if 0
	//m_crit.Lock();
	for (int i = 0; i < (int)m_ptEdges.size(); i++)
	{
		CPnt pt = m_ptEdges[i];

		if (bShowInWorldFrame)
			pt = trans.GetWorldPoint(pt);

		if (m_ptEdges[i].m_nEdgeType == 0)
			pt.Draw(ScrnRef, pDC, RGB(255, 0, 0), 7);
		else
			pt.Draw(ScrnRef, pDC, RGB(255, 255, 0), 7);

	}
	//m_crit.Unlock();


	for (int i = 0; i < (int)m_ptCorners.size(); i++)
	{
		CPnt pt = m_ptCorners[i];

		if (bShowInWorldFrame)
			pt = trans.GetWorldPoint(pt);

		pt.Draw(ScrnRef, pDC, RGB(0, 0, 255), 5);
	}
#endif
}

///////////////////////////////////////////////////////////////////////////////

//CScan* NewSimulateScan(CPosture& poseScanner, float dViewAngle, CPosture* pAssumeScannerPos)
CScan* NewSimulateScan(CPosture& poseScanner, float fStartAngle, float fEndAngle, CPosture* pAssumeScannerPos)
{
	CPnt ptScanner((float)poseScanner.x, (float)poseScanner.y);

	// ����ɨ�����ʼ�Ǻ���ֹ��
	/*float fViewStartAngle = poseScanner.fThita;
	float fViewEndAngle = poseScanner.fThita + dViewAngle;*/

	float fViewStartAngle = poseScanner.fThita + fStartAngle;
	float fViewEndAngle = poseScanner.fThita + fEndAngle;

	// ����SimuScanner����һ��ɨ�裬�õ���ɨ���ļ�����
	int nCountPoints = SimuScanner.Scan(ptScanner, fViewStartAngle, fViewEndAngle);

	CScan* pScan = new CScan(nCountPoints);
	if (pScan == NULL)
		return NULL;

	PoseGauss robotPos;

#if 0
	robotPos.m_pstMean.x = poseScanner.x;
	robotPos.m_pstMean.y = poseScanner.y;
	robotPos.m_pstMean.fThita = poseScanner.fThita;
#endif
	float dViewAngle = fEndAngle - fStartAngle;
	pScan->PolarRangesToScan(nCountPoints, (const USHORT*)&(SimuScanner.m_nDist[0]), &robotPos, NULL, fStartAngle, fEndAngle, DEFAULT_SCAN_MAX_RANGE,0);

	if (pAssumeScannerPos != NULL)
	{
		float dx = pAssumeScannerPos->x - pScan->m_poseScanner.m_pstMean.x;
		float dy = pAssumeScannerPos->y - pScan->m_poseScanner.m_pstMean.y;
		float dThita = pAssumeScannerPos->fThita - pScan->m_poseScanner.m_pstMean.fThita;

		pScan->Move(dx, dy);
		pScan->Rotate(dThita);
	}

	// ����ֱ��ɨ�輯��
	//VERIFY(pScan->CreateLineFeatures());

	// �������е�����
	//pScan->CreatePointFeatures(/*poseScanner*/);
	
	pScan->m_poseScanner.m_pstMean = poseScanner;

	return pScan;
}
