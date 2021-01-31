#include "stdafx.h"
#include "misc.h"
#include "matrix.h"
#include "scan.h"
#include "filter.h"
#include "LineFeatureSet.h"
#include "sl.h"
#include "CoxMatcher.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


/* constants */
#define MAX_DISTANCE2_POINT_TO_LINE    (2000 * 2000) /*(2000 * 2000)          cambios de Antonio */
#define MIN_POINTS_MATCHED             0.5 //0.50 
#define EXCLUDE_POINTS_START           0.66 /*0.66*/
#define EXCLUDE_MIN_DIST               100.0 //100.0
#define EXCLUDE_MAX_DIST               5000.0 //500.0
#define MAX_SIGMA_A2                   (DEG2RAD(10.0) * DEG2RAD(10.0))
#define PERPENDICULAR_FAC              1e-2

#define REDUCE_RADIUS                  25.0                        /* for reduction filter */

#define BETA                           1.0
/* multiplication factor for the error covariance matrix */

#define EPS                            1e-8        /* small value near zero */
#define MAX_MATCHING_ERROR             150//100
#define DEFAULT_MAXITERATIONS 60       //20 
#define DEFAULT_EPSD 1.0 //2.0
#define DEFAULT_EPSA DEG2RAD(0.05) //DEG2RAD(0.1)

short slMaxIterations = DEFAULT_MAXITERATIONS;
float slEpsD = DEFAULT_EPSD, slEpsA = DEFAULT_EPSA;

int CoxCalcSigma(struct CoxEquation *ce, Matrix3 *sigma, Matrix3 *inv);
int cicmp(const void *x1, const void *x2);
bool svdsolve(float s11, float s12, float s22, float b1, float b2, float *dx, float *dy);

bool allDofFixed = FALSE;

///////////////////////////////////////////////////////////////////////////////
//   ��CCoxMatcher�����ʵ�֡�

CCoxMatcher::CCoxMatcher()
{
	m_pRefScan = NULL;                  // ָ�򡰲ο�ɨ�衱��ָ��
	m_pCurScan = NULL;                  // ָ�򡰵�ǰɨ�衱��ָ��
	m_pReducedScan = NULL;          		// ���������˲��ġ���ǰɨ�衱ָ��
	m_pWorkScan = NULL;             	   // ���ڽ�����ת��ƽ�Ʋ������Ĺ���ɨ��
	m_pCoxInfoArray = NULL;             // ����Ķ�Ӧ��ϵ
}

CCoxMatcher::~CCoxMatcher()
{
	Cleanup();
}

//
//   �ͷ����ж�̬������ڴ档
//
void CCoxMatcher::Cleanup()
{
	if (m_pCoxInfoArray!= NULL)
	{
		delete m_pCoxInfoArray;
		m_pCoxInfoArray = NULL;
	}

	if (m_pWorkScan != NULL)
	{
		delete m_pWorkScan;
		m_pWorkScan = NULL;
	}

	if (m_pReducedScan != NULL)
	{
		delete m_pReducedScan;
		m_pReducedScan = NULL;
	}
}

//
//   ����Coxƥ�����
//
BOOL CCoxMatcher::Setup(CScan *pRefScan, CScan *pCurScan)
{
	// �����ԭ������
	Cleanup();

	if (pRefScan == NULL || pCurScan == NULL)
		return FALSE;

	m_pRefScan = pRefScan;
	m_pCurScan = pCurScan;

	// ����һ�ݡ���ǰɨ�衱
	m_pReducedScan = m_pCurScan->Duplicate();
	if (m_pReducedScan == NULL)
		return FALSE;

	// ��������ǰɨ�衱�Ա�ӿ������ٶ�
	ScanLineFilterFast(m_pReducedScan);
	ScanReductionFilter(m_pReducedScan, REDUCE_RADIUS);

	int nNum = m_pReducedScan->m_nCount;

	// ����������ɨ����������ʣ�ĵ���̫�٣���Setup����ʧ��
	if (nNum < SL_MIN_POINTS_IN_SCAN)
		return FALSE;

	// ����workScan
	m_pWorkScan = new CScan(m_pCurScan->m_nCount);
	if (m_pWorkScan == NULL)
		return FALSE;

	// ΪCCoxInfoArray����ռ�
	m_pCoxInfoArray = new CCoxInfoArray;
	if (m_pCoxInfoArray == NULL)
		return FALSE;

	if (!m_pCoxInfoArray->Create(nNum))
		return FALSE;

	return TRUE;
}

//
//   ���õ����������С���-�ߡ�ƥ�䡣
//
int CCoxMatcher::ScanMatchHelp(PoseGauss *pPstMatch, float *pError)
{
	float dx = 0.0, dy = 0.0, da = 0.0;
	float ddx, ddy, dda, fError;
	int rc = OK, num;

	for (int i = 0; i < slMaxIterations; i++) 
	{
		// ����reducedScan�����Ƶ���workScan����
		*m_pWorkScan = *m_pReducedScan;

		// �ԡ�workScan��������ת��ƽ��
		m_pWorkScan->Rotate(da);
		m_pWorkScan->Move(dx, dy);
		m_pWorkScan->m_poseScanner = m_pReducedScan->m_poseScanner;

		// ���С���-�ߡ�ƥ��
		rc = MatchPointsToLines(i, &num, &ddx, &ddy, &dda, &fError, &allDofFixed);

		if (pError != NULL)
			*pError = fError;

		// ������Ҳ������㹻�ĳɹ�ƥ��㣬��������
		if (rc == SLERR_NOTENOUGHPOINTSMATCHED && i > 0) 
		{
			// �˻�һ����������һ�μ���Ľ��
			dx -= ddx;
			dy -= ddy;
			da -= dda;
			ddx = ddy = dda = 0.0;
			break;
		} 
		else if (rc != OK) 
			return rc;

		// ����dx, dy, da��׼����������
		dx += ddx;
		dy += ddy;
		da += dda;

		// �����������仯�Ѻ�С����������
		if ((ddx*ddx + ddy*ddy < slEpsD*slEpsD) && (fabs(dda) < slEpsA)) 
			break;
	}

	if (pPstMatch != NULL) 
	{
		Matrix3 inv;
		PoseGauss refPos;
		PoseGauss matched;

		matched.m_pstMean = m_pReducedScan->m_poseScanner.m_pstMean;
		matched.m_pstMean.x += dx;
		matched.m_pstMean.y += dy;
		matched.m_pstMean.fThita += da;

		rc = CoxCalcSigma(&m_ce, &matched.m_Sigma, &inv);
		if (rc != OK) 
			return rc;

		if (matched.m_Sigma.d[2][2] > MAX_SIGMA_A2) 
			return SLERR_UNRELIABLE;

		if (fabs(fError) > MAX_MATCHING_ERROR)
			return SLERR_UNRELIABLE;

		refPos.m_pstMean = m_pRefScan->m_pLineFeatures->ls_Pos.m_pstMean;
		refPos.m_Sigma = Matrix3Zero;

		refPos.TransformToLocal(&matched, pPstMatch);
	}

	return OK;
}

//
//   ����ƥ�����㡣
//
int CCoxMatcher::Match(CScan* pRefScan, CScan* pCurScan, PoseGauss* pPose, float* pError)
{
	if (!Setup(pRefScan, pCurScan))
		return -1;

	return ScanMatchHelp(pPose, pError);
}

//
//   ���С���-�ߡ�ƥ�䡣
//
int CCoxMatcher::MatchPointsToLines(int iteration, int *pnum, float *ddx, float *ddy, float *dda, 
													 float *error, bool *allDofFixed)
{
	PoseGauss *pos = &m_pWorkScan->m_poseScanner;
	float rx = pos->m_pstMean.x;
	float ry = pos->m_pstMean.y;
	float s11 = 0.0, s12 = 0.0, s13 = 0.0, s22 = 0.0, s23 = 0.0, s33 = 0.0;
	float q1 = 0.0, q2 = 0.0, q3 = 0.0, s2 = 0.0;
	float det, det3, da, d;

	// �㵽��Ӧֱ�ߵ������룬�����������Ӷ���С
	float maxdist = EXCLUDE_MIN_DIST + ((long)EXCLUDE_MAX_DIST >> iteration);

	// �����еĵ����θ�����֮�����ֱ��
	int numMatched = AssignNearestLines();

	// �����Ӧ�ɹ��ĵ���ռ����̫С����ʧ�ܲ�����
	if (((float)numMatched / (float)m_pWorkScan->m_nCount) < MIN_POINTS_MATCHED) 
		return SLERR_NOTENOUGHPOINTSMATCHED;

	int i;

	// ��m_pCoxInfoArray[]���鰴���������������
	qsort(m_pCoxInfoArray->m_pCoxBuf, numMatched, sizeof(CPointCoxInfo), cicmp);

	// �˳�����Щ��Ӧ���볬�޵ĵ�(�Ա�ӿ������ٶ�)
	for (i = numMatched * EXCLUDE_POINTS_START; i < numMatched; i++) 
		if (m_pCoxInfoArray->m_pCoxBuf[i].m_dDist > maxdist) 
			break;                             // ����ĵ������Զ������ֹ

	numMatched = i;

	// ���¼��㲢ȷ��*ddx, *ddy, *dda, *error, *allDofFixed
	for (i = 0; i < numMatched; i++) 
	{
		CLineFeature *ln = m_pCoxInfoArray->m_pCoxBuf[i].m_pLine;
		if (ln == NULL)
			continue;

		CScanPoint *sp = m_pCoxInfoArray->m_pCoxBuf[i].m_pScanPoint;
		float px = sp->x, py = sp->y;
		float x1 = ln->m_fNormalX;
		float x2 = ln->m_fNormalY;
		float x3 = (px - rx) * x2 - (py - ry) * x1;
		float r = -ln->m_fDistOrigin;
		float y = r - px * x1 - py * x2;

		s11 += x1*x1;
		s12 += x1*x2;
		s13 += x1*x3;
		s22 += x2*x2;
		s23 += x2*x3;
		s33 += x3*x3;
		q1 += y*x1;
		q2 += y*x2;
		q3 += y*x3;
		s2 += y*y;

		/*
		** Perpendicular constraints ensuring line boundaries
		*/
		x1 = ln->m_fNormalY * PERPENDICULAR_FAC;
		x2 = -ln->m_fNormalX * PERPENDICULAR_FAC;
		x3 = (px - rx) * x2 - (py - ry) * x1;
		r = x1 * m_pCoxInfoArray->m_pCoxBuf[i].m_ptTarget.x + x2 * m_pCoxInfoArray->m_pCoxBuf[i].m_ptTarget.y;
		y = r - px * x1 - py * x2;
		
		s11 += x1*x1;
		s12 += x1*x2;
		s13 += x1*x3;
		s22 += x2*x2;
		s23 += x2*x3;
		s33 += x3*x3;
		q1 += y*x1;
		q2 += y*x2;
		q3 += y*x3;
		s2 += y*y;
	}

	det = s11*s22*s33 + 2*s12*s13*s23 - s11*s23*s23 - s22*s13*s13 - s33*s12*s12;
	if (NEAR_ZERO(det))
		return SLERR_DETZERO;

	det3 = q1*s12*s23 + q2*s12*s13 + q3*s11*s22 - q1*s13*s22 - q2*s11*s23 - q3*s12*s12;

	// ��*dda���и�ֵ
	*dda = da = det3 / det;

	// ��*ddx, *ddy, *allDofFixed���и�ֵ
	*allDofFixed = svdsolve(s11, s12, s22, q1 - s13*da, q2 - s23*da, ddx, ddy);

	*pnum = numMatched;

	m_ce.s11 = s11;
	m_ce.s12 = s12;
	m_ce.s13 = s13;
	m_ce.s22 = s22;
	m_ce.s23 = s23;
	m_ce.s33 = s33;
	m_ce.s2 = s2 / (numMatched - 3);

	d = m_pCoxInfoArray->m_pCoxBuf[numMatched/2].m_dDist;

	// ��*error���и�ֵ
	*error = d * d;

	return OK;
}

//
//   ��CLineFeatureSet���ҵ�����ָ����ɨ������������ֱ�ߣ����ҷ��ظ�ֱ�ߵ�ָ�롣
//
//   ������
//     ls           - CLineFeatureSet����
//     sp           - ɨ���
//     spln         - 
//
//   ����ֵ��
//     NULL - ��Ӧʧ��
//     pDist        - ��̾���ֵ
//     pTargetPoint - ��̾����
//     ���� - ָ���Ӧֱ�ߵ�ָ��
//
CLineFeature *FindNearestLine(CLineFeatureSet *ls, CScanPoint *sp, CLineFeature *spln, float *pDist, CPoint2d* pTargetPoint)
{
	CLineFeature* pNearestLine = NULL;

	float x = sp->x;
	float y = sp->y;

	float snx = spln->m_fNormalX;
	float sny = spln->m_fNormalY;

	CLineFeature* line = ls->m_pLines;

	float fMinDist2 = MAX_DISTANCE2_POINT_TO_LINE;
	CPoint2d ptFoot;
	CPoint2d ptMinFoot;

	static const float mincosa = 0.5; // cos(DEG2RAD(60))

	for (int i = 0; i < ls->m_nCount; i++, line++) 
	{
		float nx = line->m_fNormalX;
		float ny = line->m_fNormalY;

		// ������ֱ�ߵ���Ƿֱ�Ϊa1, a2������֮��ļн�Ϊa
		// ���У�nx = -sin(a1), ny = cos(a1)
		//       snx = -sin(a2), sny = cos(a2)
		// ��ô��cos(a) = cos(a1-a2) = sin(a1)*sin(a2) + cos(a1)*cos(a2) = snx * nx + syn * ny
		float cosa = snx * nx + sny * ny;

		if (/*cosa > mincosa*/1) 
		{
			float cosa2 = cosa * cosa;
			float fDist2 = nx*x + ny*y + line->m_fDistOrigin;
			fDist2 = fDist2 * fDist2 / cosa2;

			if (fDist2 < fMinDist2) 
			{
				float fTemp = line->DistanceToPoint(TRUE, *sp, NULL, &ptFoot);
				fDist2 = fTemp * fTemp /*/ cosa2*/;

				if (fDist2 < fMinDist2) 
				{
					pNearestLine = line;
					fMinDist2 = fDist2;
					ptMinFoot = ptFoot;
				}
			}
		}
#if 0
		float fDist2 = line->DistanceToPoint(TRUE, *sp, NULL, &ptFoot);

		if (fDist2 < fMinDist2)
		{
			pNearestLine = line;
			fMinDist2 = fDist2;
			ptMinFoot = ptFoot;
		}
#endif
	}

	if (pDist != NULL)
		*pDist = sqrt(fMinDist2);
//		*pDist = fMinDist2;

	if (pTargetPoint != NULL)
	{
		*pTargetPoint = ptMinFoot;
//		*pTargetPoint = ptFoot;
	}

	return pNearestLine;
}

//
//   ��ͼ��m_pWorkScan�е�ÿ��ɨ����ҵ�һ����֮��Ӧ��ֱ�߶Ρ�
//
//   ����ֵ��
//     �ɹ���Ӧ��ֱ�ߵĵ��������
//
short CCoxMatcher::AssignNearestLines()
{
	CLineFeature *pCurLine = m_pCurScan->m_pLineFeatures->m_pLines;
	short numMatched = 0;

	CPointCoxInfo* pCoxInfo = m_pCoxInfoArray->m_pCoxBuf;

	for (int i = 0; i < m_pWorkScan->m_nCount; i++) 
	{
		// ȡһ��ɨ����ָ��
		CScanPoint *pPoint = &(m_pWorkScan->m_pPoints[i]);
		
		// �������ɨ��㣬�ڲο�LineScan��Ѱ����֮��Ӧ��ֱ�߶�

		float fDist;                    // ɨ��㵽ֱ�ߵľ���
		CPoint2d ptTarget;              // ��ֱ���ϵ������(����)

		CLineFeature* pLine = FindNearestLine(m_pRefScan->m_pLineFeatures, pPoint, &pCurLine[pPoint->m_nLineID], &fDist, &ptTarget);
		if (pLine != NULL)
		{
			pCoxInfo->m_pScanPoint = pPoint;
			pCoxInfo->m_pLine = pLine;
			pCoxInfo->m_dDist = fDist;
			pCoxInfo->m_ptTarget = ptTarget;

			pCoxInfo++;
			numMatched++;
		}
	}

	return numMatched;
}

///////////////////////////////////////////////////////////////////////////////

int CoxCalcSigma(struct CoxEquation *ce, Matrix3 *sigma, Matrix3 *inv)
{
	static const float maxsdd = 1e8;

	inv->Set(
		ce->s11, ce->s12, ce->s13,
		ce->s12, ce->s22, ce->s23,
		ce->s13, ce->s23, ce->s33);

	*inv *= 1.0 / (BETA * ce->s2);

	*sigma = *inv;
	if (!sigma->Inverse()) 
	{
		fprintf(stderr, "Cox: error inverse\n");
		return SLERR_MATRIXINVERSE;
	}

	if (sigma->d[0][0] < 0.0 || sigma->d[1][1] < 0.0 || sigma->d[2][2] < 0.0) 
		return SLERR_NEGATIVEMATRIX;

	if (sigma->d[0][0] > maxsdd) 
	{
		float fac = sqrt(maxsdd / sigma->d[0][0]);

		sigma->d[0][0] = maxsdd;
		sigma->d[0][1] *= fac;
		sigma->d[1][0] *= fac;
		sigma->d[0][2] *= fac;
		sigma->d[2][0] *= fac;
	}

	if (sigma->d[1][1] > maxsdd) 
	{
		float fac = sqrt(maxsdd / sigma->d[1][1]);

		sigma->d[1][1] = maxsdd;
		sigma->d[0][1] *= fac;
		sigma->d[1][0] *= fac;
		sigma->d[1][2] *= fac;
		sigma->d[2][1] *= fac;
	}

	return OK;
}

//
//   ��CPointCoxInfo�ṹ���бȽϵĺ���(���ڶ�CPointCoxInfo�����������)��
//
int cicmp(const void *x1, const void *x2)
{
	const CPointCoxInfo *ci1 = (CPointCoxInfo*)x1;
	const CPointCoxInfo *ci2 = (CPointCoxInfo*)x2;

	if (ci1->m_dDist < ci2->m_dDist) 
		return -1;
	else if (ci1->m_dDist > ci2->m_dDist) 
		return 1;

	return 0;
}

//
//   singular value decomposition.
//
bool svdsolve(float s11, float s12, float s22, float b1, float b2, float *dx, float *dy)
{
	float ds = s11 - s22, sdet = sqrt(ds * ds / 4.0 + s12 * s12);
	float add = (s11 + s22) / 2.0;
	float v[2][2];
	float lambda[2];
	float tx, ty;
	int i;
	bool allDofFixed = FALSE; 

	lambda[0] = add + sdet;
	lambda[1] = add - sdet;

	for (i=0; i < 2; i++) 
	{
		float l = lambda[i], d1, d2;

		if (l < 0.0) 
		{
			if (l < -EPS) 
			{
				fprintf(stderr, "cox-svd: lambda[%d] = %f\n", i, l);
			}
			l = lambda[i] = 0.0;
		}
		d1 = s11 - l; 
		d2 = s22 - l;
	
		if (fabs(d1) > fabs(d2)) 
		{
			if (fabs(d1) > fabs(s12)) 
			{
				float x = s12 / d1;
				float n = sqrt(x*x + 1.0);
				v[i][0] = x / n;
				v[i][1] = -1.0 / n;
			}
			else 
			{
				float x = d1 / s12;
				float n = sqrt(x*x + 1.0);
				v[i][0] = -1.0 / n;
				v[i][1] = x / n;
			}
		}
		else 
		{
			if (fabs(d2) > fabs(s12)) 
			{
				float x = s12 / d2;
				float n = sqrt(x*x + 1.0);
				v[i][0] = -1.0 / n;
				v[i][1] = x / n;
			} 
			else 
			{
				float x = d2 / s12;
				float n = sqrt(x*x + 1.0);
				v[i][0] = x / n;
				v[i][1] = -1.0 / n;
			}
		}
	}

	lambda[0] = 1.0 / lambda[0];
	if (lambda[1] > EPS) 
	{
		if (lambda[0] * lambda[1] < 5e-2) 
		{
			lambda[1] = 0.0;
		}
		else 
		{
			lambda[1] = 1.0 / lambda[1];
			allDofFixed = TRUE;
		}
	}
	else 
		lambda[1] = 0.0;

	tx = lambda[0] * (v[0][0] * b1 + v[0][1] * b2);
	ty = lambda[1] * (v[1][0] * b1 + v[1][1] * b2);
	*dx = v[0][0] * tx + v[1][0] * ty;
	*dy = v[0][1] * tx + v[1][1] * ty;

	return allDofFixed;
}
