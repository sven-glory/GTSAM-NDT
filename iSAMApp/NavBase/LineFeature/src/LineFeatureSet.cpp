#include "stdafx.h"
#include "misc.h"
#include "scan.h"
#include "LineFeatureSet.h"
#include "CMatrix.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define MAX_DIST_POINT_TO_POINT         500     // max distance for a cloud of points (grouping)
#define MAX_DIST_POINT_TO_LINE          150     // max distance from a point to the corresponding line
#define MAX_SIGMA                       30      // max sigma value for a line
#define MIN_POINTS_ON_LINE              6       // minimum points on a line, otherwise the line is discarded
#define MIN_LINE_LEN                    400     // minimum length of a line(ԭΪ250)


void split(CLineFeature *lines, long *index, CScanPoint *sp, long lStart, long end);

///////////////////////////////////////////////////////////////////////////////
//   ��CLineFeatureSet�����ʵ�֡�

CLineFeatureSet::CLineFeatureSet(int nNum)
{
	m_pLines = new CLineFeature[nNum];
	ASSERT(m_pLines != NULL);

	m_nCount = nNum;
}

CLineFeatureSet::CLineFeatureSet(const CScan& scan)
{
	CreateFromScan(scan);
}

//
//   ȱʡ�Ĺ��캯����
//
CLineFeatureSet::CLineFeatureSet()
{
	m_nCount = 0;
	m_pLines = NULL;
}

//
//   �������������ͷ������ѷ�����ڴ档
//
CLineFeatureSet::~CLineFeatureSet()
{
	Clear();
}

//
//   ��һ��ɨ�輯�г�ȡ��������ֱ�߶Ρ�
//
void CLineFeatureSet::CreateFromScan(const CScan& scan)
{
	CScanPoint *sp;
	float last_dist = 0.0; 
	const float fac = 10.0;
	const float div = 1.0 / fac;

	// Ϊ���е�ֱ�߷���ռ�
	m_pLines = new CLineFeature[scan.m_nCount];
	ASSERT(m_pLines != NULL);

	m_nCount = scan.m_nCount;

	ls_Pos = scan.m_poseScanner;
	m_nCount = 0;

	if (scan.m_nCount == 0)
		return;

	sp = scan.m_pPoints;
	sp[0].m_nLineID = -1;

	long i;
	long start = 0;

	for (i = 1; i < scan.m_nCount; i++)
	{
		// ���㵱ǰ��sp(i)����һ��sp(i-1)֮��ľ���
		float d = sp[i-1].DistanceTo(sp[i]);

		// ��������ȡ10mm
		float dist = MAX(d, 10.0);
		float rel = last_dist / dist;

		// �ȼٶ��õ�����Ӧ��ֱ�߲�����
		sp[i].m_nLineID = -1;

		// ��� (1) �õ㵽��һ��ľ������ ����
		//      (2) �õ㵽��һ��ľ����������֮��ľ����������
		// ���Խ��з���
		if (dist > MAX_DIST_POINT_TO_POINT || 
			(i > start + 1 && (rel > fac || rel < div))) 
		{
			// �����һ��[start..i]�е���������γ�һ���߶Σ����Է��ѳ�һ���߶�
			if (i - start >= MIN_POINTS_ON_LINE)
				split(m_pLines, &m_nCount, sp, start, i-1);

			// ������һ�ε���ʼ��λ��
			start = i;
		}

		// last_distȡ����������ľ�ֵ
		if (i <= start + 1)
			last_dist = dist;
		else
			last_dist = last_dist * 0.5 + dist * 0.5;
	}

	// �����start��i�������ĵ��������һ����С�߶���������������Է��ѳ�һ��ֱ��
	if (i - start >= MIN_POINTS_ON_LINE)
		split(m_pLines, &m_nCount, sp, start, i-1);

	LineScanMergeLines(this, &((CScan&)scan), NULL);
}

//
//   ���ء�=����������
//
void CLineFeatureSet::operator = (const CLineFeatureSet& LineScan)
{
	// ��������
	m_nCount = LineScan.m_nCount;

	// ����ռ�
	m_pLines = new CLineFeature[m_nCount];
	ASSERT(m_pLines != NULL);

	// ���Ƹ����߶�
	for (int i = 0;i < m_nCount; i++)
		m_pLines[i] = LineScan.m_pLines[i];

	// ���Ƶ�ǰɨ����λ��
	ls_Pos = LineScan.ls_Pos;
}

//
//   ��������ڵ����е�ֱ�ߡ�
//
void CLineFeatureSet::Clear()
{
	if (m_pLines != NULL)
		delete []m_pLines;

	m_nCount = 0;
}

//
//   �����ڴ沢���Ƶ�ǰ�������ݣ������¸��ƵĶ���ָ�롣
//
CLineFeatureSet* CLineFeatureSet::Duplicate()
{
	// Ϊ�¸����ĸ�ֱ�߶η���ռ�
	CLineFeatureSet *copy = new CLineFeatureSet(m_nCount);
	if (copy == NULL)
		return NULL;

	// ���Ƹ�ֱ�ߵ���������
	for (int i = 0; i < m_nCount; i++)
		copy->m_pLines[i] = m_pLines[i];

	// �����������ݶ�
	copy->ls_Pos = ls_Pos;

	return copy;
}

//
//   ������һ��ֱ�߶μ������ֱ�߶μ��С�
//
BOOL CLineFeatureSet::Merge(const CLineFeatureSet& LineScan)
{
	int i;

	// ȷ���¼�����ֱ�߶ε�����
	int nNewCount = m_nCount + LineScan.m_nCount;
	CLineFeature* pNewLines = new CLineFeature[nNewCount];
	if (pNewLines == NULL)
		return FALSE;

	// ����ԭ���ĸ����߶�
	for (i = 0;i < m_nCount; i++)
		pNewLines[i] = m_pLines[i];

	// ����¼����е�ֱ�߶�
	for (i = m_nCount; i < nNewCount; i++)
		pNewLines[i] = LineScan.m_pLines[i - m_nCount];

	// �ͷ�ԭ���ĸ�ֱ�߶���ռ�ռ�
	delete []m_pLines;

	m_pLines = pNewLines;
	m_nCount = nNewCount;

	return TRUE;
}
//
//   ͨ���ϲ����ߵ��߶����򻯴�ֱ�߶μ��ϡ�
//
BOOL CLineFeatureSet::Simplify()
{
	BOOL bChange;
	int i, j;

	// ������һ���߶μ��ĸ�������Ϊÿ���߶α����Ƿ����á�
	CLineFeatureSet* pScan1 = Duplicate();
	BOOL* pUse = new BOOL[m_nCount];

	do {
		// �ȱ��������Ӧ����
		for (i = 0; i < pScan1->m_nCount; i++)
			pUse[i] = TRUE;

		// �ٽ�pScan1����һ��(�õ�pScan2)����Ϊ�Ƚ�֮��
		CLineFeatureSet* pScan2 = pScan1->Duplicate();
		bChange = FALSE;

		// ���Զ����е��߶ν�������ϲ�
		for (i = 0; i < pScan1->m_nCount; i++)
		{
			// ������Щ�ѱ����������á����߶�
			if (!pUse[i])
				continue;

			for (j = i+1; j < pScan2->m_nCount; j++)
			{
//				if (!pUse[j])
//					continue;

				// ����ϲ��ɹ���������ڶ����߶�Ϊ�������á�
				if (pScan1->m_pLines[i].Merge(pScan2->m_pLines[j], CAngle::m_fReso*3, 50.0f))
				{
					pUse[j] = FALSE;
					bChange = TRUE;
				}
			}
		}

		// �����С��ն�����������С���ѹ����ʹ֮��Ϊ�������е�����(�����г�Ա�������õ�)
		i = 0;
		for (j = 0; j < pScan1->m_nCount; j++)
		{
			if (pUse[j])
				pScan1->m_pLines[i++] = pScan1->m_pLines[j];
			else
				continue;
		}

		pScan1->m_nCount = i;          // �����߶�����
		delete pScan2;                 // �ͷ�pScan2
	} while (bChange);                // һֱ���������ٺϲ�

	Clear();
	*this = *pScan1;

	delete pScan1;
	delete []pUse;

	return TRUE;
}

//
//   ɾ��ָ�����߶Ρ�
//
BOOL CLineFeatureSet::Remove(int nIdx)
{
	if (nIdx >= m_nCount -1 || nIdx < 0)
		return FALSE;

	for (int i = nIdx; i < m_nCount-1; i++)
		m_pLines[i] = m_pLines[i+1];

	m_nCount--;

	return TRUE;
}

//
//   ��������ֱ�߶ε��ܳ��ȡ�
//
float CLineFeatureSet::TotalLength()
{
	float fSum = 0;
	for (long i = 0; i < m_nCount; i++)
		fSum += m_pLines[i].m_fTotalLen;

	return fSum;
}

//
//   ��������и������ָ��ֱ��(n1*x + n2*y + c = 0)������Զ�ľ��롣
//
//   ˵������(x0, y0)��ֱ��(n1*x + n2*y + c = 0)���빫ʽΪ��
//     d = abs(n1*x0 + n2*y0 + c)/sqrt(n1*n1 + n2*n2)
//
//   ����ֵ��
//     ����: ������Զ������
//     pMaxDist: ָ����Զ����ֵ��ָ��
//
long findMaxDist(const CScanPoint *sp, long num, float n1, float n2, float c, float *pMaxDist)
{
	long lCurMaxIdx = 0;
	float fMaxDist = 0.0;

	for (long i = 0; i < num; i++)
	{
		// ����㵽ֱ�ߵľ���,�������û���Ƿ�ĸ(����)
		float d = fabs(sp[i].x * n1 + sp[i].y * n2 + c);
		if (d > fMaxDist)
		{
			fMaxDist = d;
			lCurMaxIdx = i;
		}
	}

	if (pMaxDist != NULL)
		*pMaxDist = fMaxDist;

	return lCurMaxIdx;     // ��Զ���������Ӧ�����
}

//
//   ����һ���Ƿ�����ҵ�һ�������߶εĶ˵�����ġ��á��ϵ㣬�����������ƽ�е�ǽ�����ֱ����ȡЧ����
//   ע�⣺
//      ��(x0, y0)��ֱ��  n1*x +n2*y + c = 0 �ľ��빫ʽΪ��
//	     d = abs(n1 * x0 + n2 * y0 + c)/sqrt(n1*n1 + n2*n2)
//
static long refineBreakPoint(const CScanPoint *sp, long num, long brk, float n1, float n2, float c)
{
	// ȡ�õ�ǰ�ϵ�����
	float x = sp[brk].x;
	float y = sp[brk].y;

	// ����öϵ㵽ֱ�ߵľ���
	float fMaxDist = fabs(x * n1 + y * n2 + c);

	// ���㡰��̾���1��- ������ľ����15����
	float fMinD1 = fMaxDist - MAX_SIGMA/2.0;

	// ���㡰��̾���2��- Ϊ�ϵ㴦�����80%
	float fMinD2 = fMaxDist * 0.8;

	// ȡ������������Ľϴ�ֵ
	float fMinD = MAX(fMinD1, fMinD2);

	long end = num - 1;
//	long stop = brk;
	long i;

	// ����öϵ㵽���ľ���
	float fStartD = _hypot(x - sp[0].x, y - sp[0].y);

	// ����öϵ㵽�յ�ľ���
	float fEndD = _hypot(x - sp[end].x, y - sp[end].y);

	// ��������Ͻ�
	if (fStartD < fEndD)
	{
		// ��������㵽�öϵ㴦��û�о���ֱ��ƫ�����Ҫ��ġ���С���롱�ĵ�
		for (i = 1; i < brk; i++)
			if (fabs(sp[i].x * n1 + sp[i].y * n2 + c) > fMinD)
				return i;
	}
	// ������յ�Ͻ�
	else
	{
		// �������յ㵽�öϵ㴦��û�о���ֱ��ƫ�����ĵ�
		for (i = end - 1; i > brk; i--)
			if (fabs(sp[i].x * n1 + sp[i].y * n2 + c) > fMinD)
				return i;
	}

	return brk;
}

//
//  �ж�ɨ���������Ƿ���������������ƽֱ��
//
static bool isZigZag(const CScanPoint *sp, long num, long brk, float n1, float n2, float c)
{
	static const float eps = 0.01;

	long i;
	long lNegCount = 0;     // �����������
	long lPosCount = 0;     // �����������
	
	// �ȷ�����㵽�ϵ�֮������е�
	for (i = 0; i < brk; i++)
	{
		// ����㵽ֱ�ߵľ���
		float val = sp[i].x * n1 + sp[i].y * n2 + c;

		// ������Щ����ֱ��ƫ��ϴ�ĵ������(������ƫ��͸���ƫ���������)
		if (val < -eps) 
			lNegCount++;
		else if (val > eps) 
			lPosCount++;
	}

	// ������ƫ�롢����ƫ�����ֵ�е�һ������һ����2�����󣬿���Ϊ�ǡ�����������
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return TRUE;

	// ���¼���
	lNegCount = lPosCount = 0;

	// �ٷ����ϵ㵽�յ�֮������е㣬ԭ��ͬ��
	for (i = brk + 1; i < num; i++)
	{
		// ����㵽ֱ�ߵľ���
		float val = sp[i].x * n1 + sp[i].y * n2 + c;

		// ������Щ����ֱ��ƫ��ϴ�ĵ������(������ƫ��͸���ƫ���������)
		if (val < -eps) 
			lNegCount++;
		else if (val > eps) 
			lPosCount++;
	}

	// ������ƫ�롢����ƫ�����ֵ�е�һ������һ����2�����󣬿���Ϊ�ǡ�����������
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return TRUE;

	return FALSE;
}

/*
** split scan points[start .. end] into two parts.
** if we are at the end of the recursion,
** write the line information to lines[*index]
** and increase *index.
*/
//
//    ��ɨ���[start .. end]���ѳ��������֡�����ѵ��ݹ�Ľ������֣���ֱ����Ϣ
//    д�뵽lines[*index]�У�Ȼ�����*index��
//
void split(CLineFeature *lines, long *index, CScanPoint *sp, long lStart, long lEnd)
{
	const long min_points = MIN_POINTS_ON_LINE;
	long num_points = lEnd + 1 - lStart;
	float fStartX = sp[lStart].x;
	float fStartY = sp[lStart].y;

	float fEndX = sp[lEnd].x;
	float fEndY = sp[lEnd].y;

	float fMaxDist;
	long brk, i;
	float dx = fStartX - fEndX;
	float dy = fStartY - fEndY;
	float d = _hypot(dx, dy);
	float n1 = -dy / d;
	float n2 = dx / d;
	float c = -fStartX*n1 - fStartY*n2;
	bool refine_break_point = TRUE;

	if (d < MIN_LINE_LEN)
		return;		 // ֱ�߶�̫��

	// ��lStart�㿪ʼ���������ֱ��(n1*x + n2*y + c = 0)��Զ�����ţ����������Զ����(����fMaxDist��)
	brk = lStart + findMaxDist(&sp[lStart], num_points, n1, n2, c, &fMaxDist);

	// ���������õ���Զ����������������룬��ֱ����Ҫ����
	BOOL bSplit = (fMaxDist > MAX_DIST_POINT_TO_LINE);

	// �����ֱ�߲��ط���
	if (!bSplit)
	{
		const float max_sigma2 = MAX_SIGMA * MAX_SIGMA;
		const float thresh1 = max_sigma2 / 2.0;               // ����ֵ1
		const float thresh2 = max_sigma2 * 16.0;              // ����ֵ2

		// ������Ҫȷ��������Щ������ֱ�ߵı�׼����(n1, n2, c)
		float n1, n2, c;
		if (!RegressionLine(sp + lStart, num_points, &n1, &n2, &c))
		{
			ASSERT(FALSE);
			return;		       // fatal error
		}

		long lNewStart, lNewEnd;
		float sigma2 = 0.0;

		// ȷ����ֱ�ߵ���ʼ��(��������˳�����)
		for (i = lStart; i < lEnd; i++)
		{
			float val = n1*sp[i].x + n2*sp[i].y + c;
			val = val*val;
			sigma2 += val;

			if (val < thresh1)
				break;		  // �õ��Ѻܿ���ֱ�ߣ��ҵ���ʼ��
		}

		lNewStart = i;       // ��ʼ�����

		// ȷ����ֱ�ߵ���ֹ��(�ط������������)
		for (i = lEnd; i > lNewStart; i--)
		{
			float val = n1*sp[i].x + n2*sp[i].y + c;
			val = val*val;
			sigma2 += val;

			if (val < thresh1)
				break;		  // �õ��Ѻܿ���ֱ�ߣ��ҵ���ֹ��
		}

		lNewEnd = i;         // ��ֹ�����

		// ����������ĵ�̫�٣����޷�����߶Σ���Ҫ���׷���
		if (lNewEnd + 1 - lNewStart < min_points - 2)
			bSplit = TRUE;
		else
		{
			// ��ͼ��ʣ�µĶ����ҵ�һ���µľ����Զ�Ķϵ�
			long newbrk = -1;

			for (i = lNewStart + 1; i < lNewEnd; i++)
			{
				float val = n1*sp[i].x + n2*sp[i].y + c;
				val = val*val;
				sigma2 += val;

				if (val > thresh2 && newbrk < 0)
					newbrk = i;
			}
		
			sigma2 /= num_points;
			if (sigma2 > max_sigma2)
				bSplit = TRUE;

			// ��������ɷ�
			else if (newbrk >= 0)
			{
				brk = newbrk;
				refine_break_point = FALSE;
				bSplit = TRUE;
			}

			// ���򣬲������ٷ���
			else
			{
				// ����һ���߶�
				float val, sx, sy, tx, ty;
				long new_num = lNewEnd + 1 - lNewStart;
		
				if (num_points - new_num > 3)
				{
					float m1, m2, d;
					if (RegressionLine(&sp[lNewStart], new_num, &m1, &m2, &d))
					{
						n1 = m1;
						n2 = m2;
						c = d;
					}
				}

				// ���λ��lNewStart֮ǰ�ĵ������Ҳ��һ���߶Σ�Ӧ����Щ��Ҳ���з���
				if (lNewStart + 1 - lStart >= min_points)
					split(lines, index, sp, lStart, lNewStart);

				fStartX = sp[lNewStart].x;
				fStartY = sp[lNewStart].y;
				val = c + n1*fStartX + n2*fStartY;
				sx = fStartX - val*n1;
				sy = fStartY - val*n2;

				fEndX = sp[lNewEnd].x;
				fEndY = sp[lNewEnd].y;
				val = c + n1*fEndX + n2*fEndY;
				tx = fEndX - val*n1;
				ty = fEndY - val*n2;

				// �����߶γ���
				float fDist = _hypot(tx - sx, ty - sy);

				// ����߶εĳ��ȳ�������С���ȡ�����ȷ���ҵ�һ����Ч�߶�
				if (fDist >= MIN_LINE_LEN)
				{
					lines[*index].m_lStart = lNewStart;
					lines[*index].m_lEnd = lNewEnd;
					lines[*index].m_ptStart.x = sx;
					lines[*index].m_ptStart.y = sy;
					lines[*index].m_ptEnd.x = tx;
					lines[*index].m_ptEnd.y = ty;
					lines[*index].m_fTotalLen = fDist;
					lines[*index].m_fNormalX = n1;
					lines[*index].m_fNormalY = n2;
					lines[*index].m_fDistOrigin = c;
					lines[*index].m_fSigma2 = sigma2;

					float fTemp = (float)atan2(ty - sy, tx - sx);
					lines[*index].m_angSlant = CAngle(fTemp);
					
					// ����Щ���ڸ�ֱ���ϵĵ�����ֱ�ߵ����
					for (i = lNewStart; i <= lNewEnd; i++)
						sp[i].m_nLineID = *index;

					++*index;
				}

				// ���λ��lNewEnd֮��ĵ������Ҳ��һ���߶Σ�Ӧ����Щ��Ҳ���з���
				if (lEnd + 1 - lNewEnd >= min_points)
					split(lines, index, sp, lNewEnd, lEnd);
			}
		}
	}

	// �����Ҫ��һ������
	if (bSplit)
	{
		if (refine_break_point)
			brk = lStart + refineBreakPoint(&sp[lStart], lEnd + 1 - lStart, brk - lStart, n1, n2, c);

		// �������ʼ�㵽�ϵ㴦���н϶�㣬����һ����Ҫ��������
		if (brk + 1 - lStart >= min_points)
			split(lines, index, sp, lStart, brk);

		// ����Ӷϵ㵽��ʼ�㴦���н϶�㣬����һ����Ҫ��������
		if (lEnd + 1 - brk >= min_points)
			split(lines, index, sp, brk, lEnd);
	}
}

///////////////////////////////////////////////////////////////////////////////

#define LINE_MERGE_MAX_SIGMA2 (25.0 * 25.0)
#define LINE_MERGE_MAX_SIGMA2_FAC 2.0

#define MIN_COS_ALPHA       cos(DEG2RAD(30.0))

//
//   ����Щ�������������߶κϲ���
//
void LineScanMergeLines(CLineFeatureSet *ls, CScan *scan, long *lineNum)
{
	CScanPoint *tmp, *sp;
	static const float fac = 1.0;

	bool change = TRUE;
	long totalLines;
	long i, j;

	if (ls == NULL || scan == NULL)
		return;

	tmp = (CScanPoint *)SSmalloc(scan->m_nCount * sizeof(scan->m_pPoints[0]));
	if (tmp == NULL)
	{
		fprintf(stderr, "Unable to allocate buffer for %ld scan points.\n"
		"LineScanMergeLines failed.\n", scan->m_nCount);
		return;
	}

	sp = scan->m_pPoints;
	totalLines = ls->m_nCount;

	while (change)
	{
		change = FALSE;

		// �����е�ֱ�߶������Աȣ����Ƿ���������
		for (i = 0; i < ls->m_nCount; i++)
			for (j = i+1; j < ls->m_nCount; j++)
			{
				// ����ȡ�������߶�
				CLineFeature *ln1 = &ls->m_pLines[i];
				CLineFeature *ln2 = &ls->m_pLines[j];

				// ��������ֱ�ߵļнǣ�
				float cosa = ln1->m_fNormalX * ln2->m_fNormalX + ln1->m_fNormalY * ln2->m_fNormalY;

				float d1, d2, lambda1, lambda2;
				long num1, num2, k, l, l1, l2;
				float n1, n2, c;
				float x1, y1, x2, y2;
				float val, dist, appdist, sigma2, maxSigma2;

				if (cosa < MIN_COS_ALPHA)
					continue;

				// ����߶�1�ĳ���С��ֱ��2���������߶ζԻ�(����ǣ��߶�1�������߶�2)
				if (ln1->m_fTotalLen < ln2->m_fTotalLen)
				{
					CLineFeature *h = ln1;
					ln1 = ln2;
					ln2 = h;
				}

				d1 = ln1->DistanceToPoint(FALSE, ln2->m_ptStart, &lambda1);
				d2 = ln1->DistanceToPoint(FALSE, ln2->m_ptEnd, &lambda2);

				if (d1 > fac * MAX_DIST_POINT_TO_LINE)
					continue;

				if (d2 > fac * MAX_DIST_POINT_TO_LINE)
					continue;

				if ((lambda1 < 0.0 || lambda1 > 1.0) && (lambda2 < 0.0 || lambda2 > 1.0))
					continue;

				k = 0;
				l1 = ln1->m_lStart;
				l2 = ln2->m_lStart;
				n1 = ln1->m_fNormalX;
				n2 = ln1->m_fNormalY;

				while (l1 <= ln1->m_lEnd && l2 <= ln2->m_lEnd)
				{
					float x1 = sp[l1].x * n2 - sp[l1].y * n1;
					float x2 = sp[l2].x * n2 - sp[l2].y * n1;

					if (x1 > x2)
						tmp[k++] = sp[l1++];
					else
						tmp[k++] = sp[l2++];
				}

				while (l1 <= ln1->m_lEnd)
					tmp[k++] = sp[l1++];

				while (l2 <= ln2->m_lEnd)
					tmp[k++] = sp[l2++];

				if (!RegressionLine(tmp, k, &n1, &n2, &c))
					continue;

				maxSigma2 = LINE_MERGE_MAX_SIGMA2_FAC * (ln1->m_fSigma2 + ln2->m_fSigma2);
				maxSigma2 = MIN(maxSigma2, LINE_MERGE_MAX_SIGMA2);

				if ((lambda1 >= 0.0 && 1.0 - lambda1 > lambda2 - 1.0) ||	(lambda2 <= 1.0 && lambda2 > -lambda1))
					maxSigma2 *= 20.0;
				else
				{
					long brk = findMaxDist(tmp, k, n1, n2, c, NULL);
					brk = refineBreakPoint(tmp, k, brk, n1, n2, c);
					if (isZigZag(tmp, k, brk, n1, n2, c))
						continue;
				}

				num1 = ln1->m_lEnd + 1 - ln1->m_lStart;
				sigma2 = LineDeviation(&sp[ln1->m_lStart], num1, n1, n2, c);

				if (sigma2 > maxSigma2)
					continue;

				num2 = ln2->m_lEnd + 1 - ln2->m_lStart;
				sigma2 = LineDeviation(&sp[ln2->m_lStart], num2, n1, n2, c);

				if (sigma2 > maxSigma2)
					continue;

				sigma2 = LineDeviation(tmp, k, n1, n2, c);
				if (sigma2 > maxSigma2)
					continue;

				// �������߶κϲ�
				if (ln1->m_lStart > ln2->m_lStart)
				{
					CLineFeature *h = ln1;
					ln1 = ln2;
					ln2 = h;
					l = 0;
				}

				if (ln1->m_lEnd + 1 < ln2->m_lStart)
				{
					long num2 = ln2->m_lEnd + 1 - ln2->m_lStart;
					long src = ln1->m_lEnd + 1;
					long dst = src + num2;
					long size = ln2->m_lStart - src;

					memmove(&sp[dst], &sp[src], size * sizeof(*sp));

					for (l = 0; l < ls->m_nCount; l++)
					{
						CLineFeature *ln3 = &ls->m_pLines[l];
						if (ln3->m_lStart >= ln1->m_lEnd && ln3->m_lEnd <= ln2->m_lStart)
						{
							if (ln3->m_lStart == ln1->m_lEnd)
								ln3->m_lStart++;

							if (ln3->m_lEnd == ln2->m_lStart)
								ln3->m_lEnd--;

							ln3->m_lStart += num2;
							ln3->m_lEnd += num2;
						}
					}
					ln2->m_lStart = src;
					ln2->m_lEnd = src + num2 - 1;
				}

				x1 = tmp[0].x;
				y1 = tmp[0].y;
				x2 = tmp[k-1].x;
				y2 = tmp[k-1].y;

				val = c + n1*x1 + n2*y1;
				x1 -= val*n1;
				y1 -= val*n2;

				val = c + n1*x2 + n2*y2;
				x2 -= val*n1;
				y2 -= val*n2;

				dist = _hypot(x2 - x1, y2 - y1);
				k = ln2->m_lEnd + 1 - ln1->m_lStart;
				appdist = dist / (float)k;

				ln1->m_lEnd = ln2->m_lEnd;
				ln1->m_ptStart.x = x1;
				ln1->m_ptStart.y = y1;
				ln1->m_ptEnd.x = x2;
				ln1->m_ptEnd.y = y2;
				ln1->m_fTotalLen = dist;
				ln1->m_fNormalX = n1;
				ln1->m_fNormalY = n2;
				ln1->m_fDistOrigin = c;
				ln1->m_fSigma2 = sigma2;

				if (ln1 != &ls->m_pLines[i])
					ls->m_pLines[i] = *ln1;

				memcpy(&sp[ln1->m_lStart], tmp,  k * sizeof(*sp));

				memmove(&ls->m_pLines[j], &ls->m_pLines[j+1], (ls->m_nCount - j - 1) * sizeof(ls->m_pLines[0]));
				ls->m_nCount--;

				change = TRUE;
				j--;
			}
	}


	// ���¸����Ӧ��ֱ�߶α��
	for (i = 0; i < ls->m_nCount; i++)
	{
		CLineFeature *ln = &ls->m_pLines[i];
		for (j = ln->m_lStart; j <= ln->m_lEnd; j++)
		{
			long old = sp[j].m_nLineID;

			if (lineNum != NULL && old >=0 && old < totalLines)
				lineNum[old] = i;

			sp[j].m_nLineID = i;
		}
	}

	SSfree(tmp);
}





///////////////////////////////////////////////////////////////////////////////

/* new version, 15.03.2003 */

#define LA_MAX_DIST 200.0
#define MIN_ANGLEDIST DEG2RAD(20.0)

//
//   returns field of view of linescan in rad.
//
float CLineFeatureSet::FieldOfView(CScan *pScan)
{
	CScanPoint *sp;
	long i, start, end;
	float fVisibleAng = 2*PI;     // ���ӽ�(����)
	float fAngDist;

	if (pScan == NULL || m_nCount < 1)
		return 0.0;

	sp = pScan->m_pPoints;

	for (i = 1; i < m_nCount; i++)
	{
		end = m_pLines[i-1].m_lEnd;
		start = m_pLines[i].m_lStart;
		fAngDist = sp[end].AngleDistanceTo(sp[start]);

		if (fAngDist > MIN_ANGLEDIST)
			fVisibleAng -= fAngDist;
	}

	end = m_pLines[i-1].m_lEnd;
	start = m_pLines[0].m_lStart;
	fAngDist = sp[end].AngleDistanceTo(sp[start]);

	if (fAngDist > MIN_ANGLEDIST)
		fVisibleAng -= fAngDist;

	if (fVisibleAng < 0.0)
		fVisibleAng = 0.0;

	return fVisibleAng;
}

//
//   ȥ�����г��ȶ���minLineLength��ֱ�ߡ�
//
void CLineFeatureSet::LengthFilter(float minLineLength)
{
	int num = 0;
	
	for (int i = 0; i < m_nCount; i++)
		if (m_pLines[i].m_fTotalLen > minLineLength)
			m_pLines[num++] = m_pLines[i];

	m_nCount = num;
}

void CLineFeatureSet::Dump()
{
	TRACE("Dumping Line Scan (%d lines):\n", m_nCount);
	for (int i = 0; i < m_nCount; i++)
	{
		TRACE("Line #%d: %.2f, %.2f, %.2f, %.2f, %0.2f, %0.2f, %0.2f,\tLength = %.2f\n", i,
			m_pLines[i].m_ptStart.x, m_pLines[i].m_ptStart.y, 
			m_pLines[i].m_ptEnd.x, m_pLines[i].m_ptEnd.y, 

			m_pLines[i].m_fNormalX, m_pLines[i].m_fNormalY, m_pLines[i].m_fDistOrigin,

			m_pLines[i].m_fTotalLen);
	}

	TRACE("\n");
}

void CLineFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize)
{
	for (int i = 0; i < m_nCount; i++)
	{
		if (!m_pLines[i].m_bSelected)
			m_pLines[i].Draw(ScrnRef, pDC, crColor, 2*nPointSize, nPointSize, TRUE);
		else
			m_pLines[i].Draw(ScrnRef, pDC, crSelected, 2*nPointSize, nPointSize, TRUE);
	}
}


//
//   �ж�ֱ��ɨ�輯�Ƿ����ָ���ĵ㡣
//
BOOL CLineFeatureSet::ContainScanPoint(const CScanPoint& sp)
{
	for (int i = 0; i < m_nCount; i++)
	{
		if (m_pLines[i].DistanceToPoint(TRUE, sp) < 50)
			return TRUE;
	}

	return FALSE;
}

//
//   �Ƴ�λ��ָ�������ڵ��߶Ρ�
//
void CLineFeatureSet::RemoveWithin(CRegion& rgn)
{
	int i = 0;
	for (int j = 0; j < m_nCount; j++)
	{
		if (rgn.Contain(m_pLines[j].m_ptStart) && rgn.Contain(m_pLines[j].m_ptEnd))
			continue;
		else
			m_pLines[i++] = m_pLines[j];
	}
	m_nCount = i;
}

//
//   ѡ��/ȡ��ѡ��ָ�����߶Ρ�
//
void CLineFeatureSet::Select(int nIdx, bool bOn)
{
	if (nIdx < 0)
	{
		for (int i = 0; i < m_nCount; i++)
			m_pLines[i].Select(bOn);
	}
	else
		m_pLines[nIdx].Select(bOn);
}
