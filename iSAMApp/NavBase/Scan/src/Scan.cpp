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

#define RANGE_JUMP_GATE            350      // 判断极径是否发生跳变的门限值
#define MIN_CORNER_ANGLE           (PI/6)   // 形成角点的两条直线的最小夹角
#define MIN_EDGE_ANGLE             (PI/6)   // 扫描线与边缘点所在线所成夹角的最小值
#define MIN_LINE_FEATURE_LEN       450      // 只有超过此长度的直线段才允许生成角点
#define MAX_HIDDEN_LINE_LEN        5000     // 如果两直线段的延长线超过此距离，生成的交点不采用
#define DIST_IGNORE_SHORT_LINE     10000    // 如果扫描距离短于此距离，将忽略发现的极短直线

float ScanMaxRange = DEFAULT_SCAN_MAX_RANGE;
static ulong scanId = 1;

CSimuScanner SimuScanner;

///////////////////////////////////////////////////////////////////////////////

//
//   构造具有nNum个点的CScan对象。
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
//   重载“=”操作符
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
//   清除所有数据。
//
void CScan::Clear()
{
	// 清除点云数据
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

	// 清除其它所有数据
	// ..
}

//
//   生成一个新的扫描。
//
BOOL CScan::Create(int nNum)
{
	// 在此应将所有数据成员初始化为0
	Clear();

	// 为点云分配空间
	if (!CScanPointCloud::Create(nNum))
		return FALSE;

	return TRUE;
}

//
//   计算两个扫描数据的扫描头之间的距离。
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
//   取得指定扫描头的绝对姿态。
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
//   根据点云数据计算“平均范围”和“最大范围”。
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
//   分配空间并复制当前扫描。
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
//   将当前扫描与另一个扫描集进行合并。
//
void CScan::Merge(CScan& scan2, BOOL bNewPointsOnly)
{
	CScanPointCloud* p1 = GetScanPointCloudPointer();
	CScanPointCloud* p2 = scan2.GetScanPointCloudPointer();

	// 先复制原来的点云。
	CScanPointCloud CopyThis(*p1);

	// 清除原点云数据
	p1->Clear();

	// 重新分配空间以便容纳两个点云的数据
	VERIFY(p1->Create(CopyThis.m_nCount + p2->m_nCount));

	int i;

	// 复制回原有数据
	for (i = 0; i < CopyThis.m_nCount; i++)
		m_pPoints[i] = CopyThis.m_pPoints[i];

	// 再增加入新点云数据
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
//   将全部点移动指定的距离。
//
void CScan::Move(float dx, float dy)
{
	if ((dx == 0) && (dy == 0))
		return;

	// 移动点云数据
	CScanPointCloud::Move(dx, dy);

	// 移动激光头的均值点
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
//   找到本扫描点集中的所有“直线扫描线”。
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
//   根据给定的激光头姿态，验证指定的点是否是一个有效的边缘点，
//   查看给定的点是否是直线集中某条直线的端点，并返回该边缘点的修正值。
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

			// 取得扫描线与直线所成的夹角
			float fAngDiff = ang.GetDifference(angScanner);

			// 如果此夹角太小，则不视其为有效边缘点
			if (fAngDiff < MIN_EDGE_ANGLE || fAngDiff > PI - MIN_EDGE_ANGLE)
				return false;

			pt = ln.m_ptStart;
			return true;
		}
		else if (ln.m_ptEnd.DistanceTo(pt) < 30)
		{	
			CAngle& ang = !ln.SlantAngle();

			// 取得扫描线与直线所成的夹角
			float fAngDiff = ang.GetDifference(angScanner);

			// 如果此夹角太小，则不视其为有效边缘点
			if (fAngDiff < MIN_EDGE_ANGLE || fAngDiff > PI - MIN_EDGE_ANGLE)
				return false;

			pt = ln.m_ptEnd;
			return true;
		}
	}

	return false;
}

//
//   判断给定的点是否与某个角点距离过近。
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
//   生成所有点特征。
//   注意：此函数必须在已生成过所有直线特征之后(即调用过CreateLineFeatures()之后)调用！
//
bool CScan::CreatePointFeatures(/*CPosture& pstScanner*/)
{
	CPnt ptOrigin(0, 0);

	vector<CLine> Lines;

	// 清零所有(用于生成虚拟角点的)直线段
	Lines.clear();

	// 先复制所有的直线段到Lines中
	for (int i = 0; i < m_pLineFeaturesMerged->m_nCount; i++)
	{
		bool bFoundColinear = false;

		// 取一条直线
		CLine& ln1 = m_pLineFeaturesMerged->m_pLines[i].GetLineObject();

		// 如果没有发现共线情形，将此直线加入Lines中
		if (!bFoundColinear)
			Lines.push_back(ln1);
	}

	// 下面生成所有两两相交直线的交点集合，作为“角点特征”
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
			// 计算两条直线的差角
			CAngle angDiff = ln1.SlantAngle() - ln2.SlantAngle();
			if ((angDiff > MIN_CORNER_ANGLE && angDiff < (PI - MIN_CORNER_ANGLE)) ||
				 (angDiff > (PI + MIN_CORNER_ANGLE) && angDiff < (2*PI - MIN_CORNER_ANGLE)))
			{
				float x, y;
				bool bOnLine1 = false;
				bool bOnLine2 = false;

				// 计算两条直线段的交点，并判断交点是否在两条线段上
				if (ln1.Intersect(ln2, &x, &y, &bOnLine1, &bOnLine2))
				{
					CPnt pt(x, y);
					float fDist;
					
					// 如果交点不在直线1上
					if (!bOnLine1)
					{
						ln1.FindNearPoint(pt, &fDist);
						float fLen1 = ln1.Length();
						
						// 对于很短的直线特征，要求延长长度不能超过直线特征自身的长度
						if (fLen1 < MIN_LINE_FEATURE_LEN * 3)
						{
							if (fDist > fLen1)
								continue;
						}
						// 对于一般长度的直线特征，延长长度不能超过规定的固定长度(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// 如果交点不在直线2上
					if (!bOnLine2)
					{
						ln2.FindNearPoint(pt, &fDist);
						float fLen2 = ln2.Length();

						// 对于很短的直线特征，要求延长长度不能超过直线特征自身的长度
						if (fLen2 < MIN_LINE_FEATURE_LEN * 3)
						{
							if (fDist > fLen2)
								continue;
						}

						// 对于一般长度的直线特征，延长长度不能超过规定的固定长度(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// 如果该点与先前加入的点太近，则不将它加入
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
	// 下面生成边缘点特征
	m_ptEdges.clear();
	//m_crit.Unlock();
	// 在此检测边缘点
	int nLastJump = 0;   // 上一个跳跃点
	bool bDetectStartEdge = false;

	float r1 = m_pPoints[0].r;
	for (int i = 1; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];

		// 如果极径发生突变，说明新段开始
		if (fabs(sp.r - r1) > RANGE_JUMP_GATE)
		{
			// 如果新极径变小，说明可能是发现了某个线段的起始边缘点
			if (sp.r < r1)
			{
				bDetectStartEdge = true;
			}

			// 如果极径突然变大
			else
			{
				bDetectStartEdge = false;

				// 如果当前连续段所含点数超过5个，则找到了一个结束边缘点
				if (i - nLastJump > 20)
				{
					// 上一个点(序号i-1)为结束边缘点
					CEdgePoint ept;

					ept.GetPntObject() = m_pPoints[i - 1].GetPntObject();
					ept.m_nEdgeType = 1;   // 结束边缘点
					if (VerifyEdgePoint(ept.GetPntObject()))
					{
						if (!PointTooCloseToSomeCorner(ept))
							m_ptEdges.push_back(ept);
					}
				}
			}

			nLastJump = i;
		}

		// 如果极径没有突变
		else
		{
			// 如果已经发现起始边缘点，且刚刚结束的段所含点数超过5，则确认起点是个边缘点
			if (bDetectStartEdge && (i - nLastJump > 20))
			{
				// 在此加入新的边缘点
				CEdgePoint ept;
				ept.GetPntObject() = m_pPoints[nLastJump].GetPntObject();
				ept.m_nEdgeType = 0;    // 超始边缘点

				if (VerifyEdgePoint(ept.GetPntObject()))
				{
					m_ptEdges.push_back(ept);
				}
				bDetectStartEdge = false;
			}
		}
		r1 = sp.r;
	}

	// 在此搜索出所有由实角点之间连线组成的线段
	m_CornerPairs.clear();

	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		// 取得直线段
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

	// 在此找出所有以一个角点和一个边缘点组成的线段
	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		// 取得直线段
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
	// 在此找出所有以两个边缘点组成的线段
	for (int i = 0; i < m_pLineFeatures->m_nCount; i++)
	{
		// 取得直线段
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

	// 取得激光头原来的坐标
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

			// 计算时间间隔
			float time = scanTime * (mod - interlace) + div * stepT;

			CPosture movePose, realScanPose;
			float dx, dy;

			// movePose为在时间time内激光头状态的变化量
			movePose.x = tvel * time;
			movePose.y = 0.0;
			movePose.fThita = rvel * time;

			((PoseGauss*)robotPos)->m_pstMean.TransformToGlobal(&movePose, &m_poseScanner.m_pstMean);

			// 取得激光头在全局坐标系下的姿态
			GetScannerAbsPos(realScanPose);

			// 计算该扫描点的平面直角坐标
			sp->x = realScanPose.x + d * cos(/*realScanPose.fThita +*/ a);
			sp->y = realScanPose.y + d * sin(/*realScanPose.fThita +*/ a);

			// 
			dx = sp->x - scanPose.x;
			dy = sp->y - scanPose.y;

			// 计算该扫描点的极坐标
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

	// 在此检测边缘点
	int nLastJump = 0;   // 上一个跳跃点
	bool bDetectStartEdge = false;

	float r1 = m_pPoints[0].r;
	for (int i = 1; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];

		// 如果极径发生突变，说明新段开始
		if (fabs(sp.r - r1) > 500)
		{
			// 如果新极径变小，说明可能是发现了某个线段的起始边缘点
			if (sp.r < r1)
			{
				bDetectStartEdge = true;
			}

			// 如果极径突然变大
			else if (sp.r > r1)
			{
				// 如果当前连续段所含点数超过5个，则找到了一个结束边缘点
				if (i - nLastJump > 5)
				{
					// 上一个点(序号i-1)为结束边缘点
					CPoint2d& pt = m_pPoints[i-1].GetPoint2dObject();
					m_ptEdges.push_back(pt);
				}
			}

			nLastJump = i;
		}

		// 如果极径没有突变
		else
		{
			// 如果已经发现起始边缘点，且刚刚结束的段所含点数超过5，则确认起点是个边缘点
			if (bDetectStartEdge && (i - nLastJump > 5))
			{
				// 在此加入新的边缘点
				CPoint2d& pt = m_pPoints[nLastJump].GetPoint2dObject();
				m_ptEdges.push_back(pt);

				bDetectStartEdge = false;
			}
		}
		r1 = sp.r;
	}
#endif

	SortByAngle();              // 在此按距离进行排序，原有扫描角次序被打乱
	SetProperties();

	// 生成直线扫描集合
//	VERIFY(CreateLineScans());

	return this;
}

//
//   判断该扫描集是否包含指定的扫描点。
//
BOOL CScan::ContainScanPoint(const CScanPoint& sp, float fThreshHoldDist)
{
	// 判断该点是否与扫描集中的某个点相距离非常近(小于指定的门限)
	if (CScanPointCloud::ContainPoint(sp, fThreshHoldDist) >= 0)
		return TRUE;

	// 判断该点是否与扫描集中的某条直线相距离非常近
	if (m_pLineFeatures->ContainScanPoint(sp))
		return TRUE;

	return FALSE;
}
#if 1
//
//   从文件中读取扫描数据。
//
BOOL CScan::LoadFromFile(FILE* fp)
{
	m_pLineFeatures = NULL;

	if (!CScanPointCloud::LoadFromFile(fp))
		return FALSE;

	return CreateLineFeatures();
}

//
//   将扫描数据保存到文件。
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

	// 取得扫描时激光头的姿态
	CPosture& pstRef = m_poseScanner.m_pstMean;
	
	// 建立局部坐标系
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

			// 如果需要在世界坐标系中显示，现在需要进行坐标变换
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

	// 计算扫描的起始角和终止角
	/*float fViewStartAngle = poseScanner.fThita;
	float fViewEndAngle = poseScanner.fThita + dViewAngle;*/

	float fViewStartAngle = poseScanner.fThita + fStartAngle;
	float fViewEndAngle = poseScanner.fThita + fEndAngle;

	// 利用SimuScanner进行一次扫描，得到各扫描点的极坐标
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

	// 生成直线扫描集合
	//VERIFY(pScan->CreateLineFeatures());

	// 生成所有点特征
	//pScan->CreatePointFeatures(/*poseScanner*/);
	
	pScan->m_poseScanner.m_pstMean = poseScanner;

	return pScan;
}
