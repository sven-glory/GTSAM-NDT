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
#define MIN_LINE_LEN                    400     // minimum length of a line(原为250)


void split(CLineFeature *lines, long *index, CScanPoint *sp, long lStart, long end);

///////////////////////////////////////////////////////////////////////////////
//   “CLineFeatureSet”类的实现。

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
//   缺省的构造函数。
//
CLineFeatureSet::CLineFeatureSet()
{
	m_nCount = 0;
	m_pLines = NULL;
}

//
//   在析构函数中释放所有已分配的内存。
//
CLineFeatureSet::~CLineFeatureSet()
{
	Clear();
}

//
//   从一个扫描集中抽取其有所有直线段。
//
void CLineFeatureSet::CreateFromScan(const CScan& scan)
{
	CScanPoint *sp;
	float last_dist = 0.0; 
	const float fac = 10.0;
	const float div = 1.0 / fac;

	// 为所有的直线分配空间
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
		// 计算当前点sp(i)到上一点sp(i-1)之间的距离
		float d = sp[i-1].DistanceTo(sp[i]);

		// 距离至少取10mm
		float dist = MAX(d, 10.0);
		float rel = last_dist / dist;

		// 先假定该点所对应的直线不存在
		sp[i].m_nLineID = -1;

		// 如果 (1) 该点到上一点的距离过大， 或者
		//      (2) 该点到上一点的距离比上两点之间的距离差距离过大
		// 则尝试进行分裂
		if (dist > MAX_DIST_POINT_TO_POINT || 
			(i > start + 1 && (rel > fac || rel < div))) 
		{
			// 如果这一段[start..i]中点的数量够形成一个线段，尝试分裂出一条线段
			if (i - start >= MIN_POINTS_ON_LINE)
				split(m_pLines, &m_nCount, sp, start, i-1);

			// 调整下一段的起始点位置
			start = i;
		}

		// last_dist取近两个距离的均值
		if (i <= start + 1)
			last_dist = dist;
		else
			last_dist = last_dist * 0.5 + dist * 0.5;
	}

	// 如果从start到i点所含的点的数量够一个最小线段所需的数量，尝试分裂出一条直线
	if (i - start >= MIN_POINTS_ON_LINE)
		split(m_pLines, &m_nCount, sp, start, i-1);

	LineScanMergeLines(this, &((CScan&)scan), NULL);
}

//
//   重载“=”操作符。
//
void CLineFeatureSet::operator = (const CLineFeatureSet& LineScan)
{
	// 复制数量
	m_nCount = LineScan.m_nCount;

	// 分配空间
	m_pLines = new CLineFeature[m_nCount];
	ASSERT(m_pLines != NULL);

	// 复制各条线段
	for (int i = 0;i < m_nCount; i++)
		m_pLines[i] = LineScan.m_pLines[i];

	// 复制当前扫描器位置
	ls_Pos = LineScan.ls_Pos;
}

//
//   清除对象内的所有的直线。
//
void CLineFeatureSet::Clear()
{
	if (m_pLines != NULL)
		delete []m_pLines;

	m_nCount = 0;
}

//
//   分配内存并复制当前对象内容，返回新复制的对象指针。
//
CLineFeatureSet* CLineFeatureSet::Duplicate()
{
	// 为新副本的各直线段分配空间
	CLineFeatureSet *copy = new CLineFeatureSet(m_nCount);
	if (copy == NULL)
		return NULL;

	// 复制各直线的数据内容
	for (int i = 0; i < m_nCount; i++)
		copy->m_pLines[i] = m_pLines[i];

	// 复制其它数据段
	copy->ls_Pos = ls_Pos;

	return copy;
}

//
//   将此另一个直线段集并入此直线段集中。
//
BOOL CLineFeatureSet::Merge(const CLineFeatureSet& LineScan)
{
	int i;

	// 确定新集合中直线段的数量
	int nNewCount = m_nCount + LineScan.m_nCount;
	CLineFeature* pNewLines = new CLineFeature[nNewCount];
	if (pNewLines == NULL)
		return FALSE;

	// 复制原来的各条线段
	for (i = 0;i < m_nCount; i++)
		pNewLines[i] = m_pLines[i];

	// 添加新集合中的直线段
	for (i = m_nCount; i < nNewCount; i++)
		pNewLines[i] = LineScan.m_pLines[i - m_nCount];

	// 释放原来的各直线段所占空间
	delete []m_pLines;

	m_pLines = pNewLines;
	m_nCount = nNewCount;

	return TRUE;
}
//
//   通过合并共线的线段来简化此直线段集合。
//
BOOL CLineFeatureSet::Simplify()
{
	BOOL bChange;
	int i, j;

	// 先生成一个线段集的附本，并为每个线段标明是否“启用”
	CLineFeatureSet* pScan1 = Duplicate();
	BOOL* pUse = new BOOL[m_nCount];

	do {
		// 先标明所有项都应启用
		for (i = 0; i < pScan1->m_nCount; i++)
			pUse[i] = TRUE;

		// 再将pScan1复制一份(得到pScan2)，作为比较之用
		CLineFeatureSet* pScan2 = pScan1->Duplicate();
		bChange = FALSE;

		// 尝试对所有的线段进行逐项合并
		for (i = 0; i < pScan1->m_nCount; i++)
		{
			// 跳过那些已标明“不启用”的线段
			if (!pUse[i])
				continue;

			for (j = i+1; j < pScan2->m_nCount; j++)
			{
//				if (!pUse[j])
//					continue;

				// 如果合并成功，则标明第二个线段为“不启用”
				if (pScan1->m_pLines[i].Merge(pScan2->m_pLines[j], CAngle::m_fReso*3, 50.0f))
				{
					pUse[j] = FALSE;
					bChange = TRUE;
				}
			}
		}

		// 将带有“空洞”的数组进行“挤压”，使之成为紧凑排列的数组(即所有成员都是启用的)
		i = 0;
		for (j = 0; j < pScan1->m_nCount; j++)
		{
			if (pUse[j])
				pScan1->m_pLines[i++] = pScan1->m_pLines[j];
			else
				continue;
		}

		pScan1->m_nCount = i;          // 更新线段数量
		delete pScan2;                 // 释放pScan2
	} while (bChange);                // 一直处理到不能再合并

	Clear();
	*this = *pScan1;

	delete pScan1;
	delete []pUse;

	return TRUE;
}

//
//   删除指定的线段。
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
//   计算所有直线段的总长度。
//
float CLineFeatureSet::TotalLength()
{
	float fSum = 0;
	for (long i = 0; i < m_nCount; i++)
		fSum += m_pLines[i].m_fTotalLen;

	return fSum;
}

//
//   计算点云中各点距离指定直线(n1*x + n2*y + c = 0)距离最远的距离。
//
//   说明：点(x0, y0)到直线(n1*x + n2*y + c = 0)距离公式为：
//     d = abs(n1*x0 + n2*y0 + c)/sqrt(n1*n1 + n2*n2)
//
//   返回值：
//     返回: 距离最远点的序号
//     pMaxDist: 指向最远距离值的指针
//
long findMaxDist(const CScanPoint *sp, long num, float n1, float n2, float c, float *pMaxDist)
{
	long lCurMaxIdx = 0;
	float fMaxDist = 0.0;

	for (long i = 0; i < num; i++)
	{
		// 计算点到直线的距离,距离计算没考虑分母(常数)
		float d = fabs(sp[i].x * n1 + sp[i].y * n2 + c);
		if (d > fMaxDist)
		{
			fMaxDist = d;
			lCurMaxIdx = i;
		}
	}

	if (pMaxDist != NULL)
		*pMaxDist = fMaxDist;

	return lCurMaxIdx;     // 最远距离点所对应的序号
}

//
//   分析一下是否可以找到一个距离线段的端点更近的“好”断点，这样可以针对平行的墙面提高直线提取效果。
//   注意：
//      点(x0, y0)到直线  n1*x +n2*y + c = 0 的距离公式为：
//	     d = abs(n1 * x0 + n2 * y0 + c)/sqrt(n1*n1 + n2*n2)
//
static long refineBreakPoint(const CScanPoint *sp, long num, long brk, float n1, float n2, float c)
{
	// 取得当前断点坐标
	float x = sp[brk].x;
	float y = sp[brk].y;

	// 计算该断点到直线的距离
	float fMaxDist = fabs(x * n1 + y * n2 + c);

	// 计算“最短距离1”- 比上面的距离近15个点
	float fMinD1 = fMaxDist - MAX_SIGMA/2.0;

	// 计算“最短距离2”- 为断点处距离的80%
	float fMinD2 = fMaxDist * 0.8;

	// 取上面两个距离的较大值
	float fMinD = MAX(fMinD1, fMinD2);

	long end = num - 1;
//	long stop = brk;
	long i;

	// 计算该断点到起点的距离
	float fStartD = _hypot(x - sp[0].x, y - sp[0].y);

	// 计算该断点到终点的距离
	float fEndD = _hypot(x - sp[end].x, y - sp[end].y);

	// 如果离起点较近
	if (fStartD < fEndD)
	{
		// 看看从起点到该断点处有没有距离直线偏离大于要求的“最小距离”的点
		for (i = 1; i < brk; i++)
			if (fabs(sp[i].x * n1 + sp[i].y * n2 + c) > fMinD)
				return i;
	}
	// 如果离终点较近
	else
	{
		// 看看从终点到该断点处有没有距离直线偏离更大的点
		for (i = end - 1; i > brk; i--)
			if (fabs(sp[i].x * n1 + sp[i].y * n2 + c) > fMinD)
				return i;
	}

	return brk;
}

//
//  判断扫描点的连线是否来回弯曲，不够平直。
//
static bool isZigZag(const CScanPoint *sp, long num, long brk, float n1, float n2, float c)
{
	static const float eps = 0.01;

	long i;
	long lNegCount = 0;     // 负向距离点计数
	long lPosCount = 0;     // 正向距离点计数
	
	// 先分析起点到断点之间的所有点
	for (i = 0; i < brk; i++)
	{
		// 计算点到直线的距离
		float val = sp[i].x * n1 + sp[i].y * n2 + c;

		// 计算那些距离直线偏离较大的点的数量(分正向偏离和负向偏离两种情况)
		if (val < -eps) 
			lNegCount++;
		else if (val > eps) 
			lPosCount++;
	}

	// 如正向偏离、负向偏离读数值中的一方比另一方的2倍还大，可认为是“弯弯曲曲”
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return TRUE;

	// 重新计数
	lNegCount = lPosCount = 0;

	// 再分析断点到终点之间的所有点，原理同上
	for (i = brk + 1; i < num; i++)
	{
		// 计算点到直线的距离
		float val = sp[i].x * n1 + sp[i].y * n2 + c;

		// 计算那些距离直线偏离较大的点的数量(分正向偏离和负向偏离两种情况)
		if (val < -eps) 
			lNegCount++;
		else if (val > eps) 
			lPosCount++;
	}

	// 如正向偏离、负向偏离读数值中的一方比另一方的2倍还大，可认为是“弯弯曲曲”
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
//    将扫描点[start .. end]分裂成两个部分。如果已到递归的结束部分，将直线信息
//    写入到lines[*index]中，然后递增*index。
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
		return;		 // 直线段太短

	// 从lStart点开始，计算出到直线(n1*x + n2*y + c = 0)最远点的序号，并求出该最远距离(存于fMaxDist中)
	brk = lStart + findMaxDist(&sp[lStart], num_points, n1, n2, c, &fMaxDist);

	// 如果上面求得的最远距离大于最大允许距离，则直线需要分裂
	BOOL bSplit = (fMaxDist > MAX_DIST_POINT_TO_LINE);

	// 如果该直线不必分裂
	if (!bSplit)
	{
		const float max_sigma2 = MAX_SIGMA * MAX_SIGMA;
		const float thresh1 = max_sigma2 / 2.0;               // 门限值1
		const float thresh2 = max_sigma2 * 16.0;              // 门限值2

		// 现在需要确定所有这些点所处直线的标准参数(n1, n2, c)
		float n1, n2, c;
		if (!RegressionLine(sp + lStart, num_points, &n1, &n2, &c))
		{
			ASSERT(FALSE);
			return;		       // fatal error
		}

		long lNewStart, lNewEnd;
		float sigma2 = 0.0;

		// 确定该直线的起始点(沿正方向顺序查找)
		for (i = lStart; i < lEnd; i++)
		{
			float val = n1*sp[i].x + n2*sp[i].y + c;
			val = val*val;
			sigma2 += val;

			if (val < thresh1)
				break;		  // 该点已很靠近直线，找到起始点
		}

		lNewStart = i;       // 起始点序号

		// 确定该直线的终止点(沿反方向逆序查找)
		for (i = lEnd; i > lNewStart; i--)
		{
			float val = n1*sp[i].x + n2*sp[i].y + c;
			val = val*val;
			sigma2 += val;

			if (val < thresh1)
				break;		  // 该点已很靠近直线，找到终止点
		}

		lNewEnd = i;         // 终止点序号

		// 如果所包含的点太少，则无法组成线段，需要彻底分裂
		if (lNewEnd + 1 - lNewStart < min_points - 2)
			bSplit = TRUE;
		else
		{
			// 试图在剩下的段中找到一个新的距离较远的断点
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

			// 如果继续可分
			else if (newbrk >= 0)
			{
				brk = newbrk;
				refine_break_point = FALSE;
				bSplit = TRUE;
			}

			// 否则，不可能再分了
			else
			{
				// 增加一条线段
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

				// 如果位于lNewStart之前的点的数量也够一条线段，应对那些点也进行分裂
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

				// 计算线段长度
				float fDist = _hypot(tx - sx, ty - sy);

				// 如果线段的长度超过“最小长度”，可确认找到一个有效线段
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
					
					// 将这些处于该直线上的点标记上直线的序号
					for (i = lNewStart; i <= lNewEnd; i++)
						sp[i].m_nLineID = *index;

					++*index;
				}

				// 如果位于lNewEnd之后的点的数量也够一条线段，应对那些点也进行分裂
				if (lEnd + 1 - lNewEnd >= min_points)
					split(lines, index, sp, lNewEnd, lEnd);
			}
		}
	}

	// 如果需要进一步分裂
	if (bSplit)
	{
		if (refine_break_point)
			brk = lStart + refineBreakPoint(&sp[lStart], lEnd + 1 - lStart, brk - lStart, n1, n2, c);

		// 如果从起始点到断点处还有较多点，则这一段需要继续分裂
		if (brk + 1 - lStart >= min_points)
			split(lines, index, sp, lStart, brk);

		// 如果从断点到起始点处还有较多点，则这一段需要继续分裂
		if (lEnd + 1 - brk >= min_points)
			split(lines, index, sp, brk, lEnd);
	}
}

///////////////////////////////////////////////////////////////////////////////

#define LINE_MERGE_MAX_SIGMA2 (25.0 * 25.0)
#define LINE_MERGE_MAX_SIGMA2_FAC 2.0

#define MIN_COS_ALPHA       cos(DEG2RAD(30.0))

//
//   将那些共线且相连的线段合并。
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

		// 将所有的直线段两两对比，看是否共线且相连
		for (i = 0; i < ls->m_nCount; i++)
			for (j = i+1; j < ls->m_nCount; j++)
			{
				// 依次取得两条线段
				CLineFeature *ln1 = &ls->m_pLines[i];
				CLineFeature *ln2 = &ls->m_pLines[j];

				// 计算两条直线的夹角？
				float cosa = ln1->m_fNormalX * ln2->m_fNormalX + ln1->m_fNormalY * ln2->m_fNormalY;

				float d1, d2, lambda1, lambda2;
				long num1, num2, k, l, l1, l2;
				float n1, n2, c;
				float x1, y1, x2, y2;
				float val, dist, appdist, sigma2, maxSigma2;

				if (cosa < MIN_COS_ALPHA)
					continue;

				// 如果线段1的长度小于直线2，则将两条线段对换(结果是：线段1不短于线段2)
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

				// 将两条线段合并
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


	// 更新各点对应的直线段编号
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
	float fVisibleAng = 2*PI;     // 可视角(弧度)
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
//   去掉所有长度短于minLineLength的直线。
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
//   判断直线扫描集是否包含指定的点。
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
//   移除位于指定区域内的线段。
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
//   选中/取消选中指定的线段。
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
