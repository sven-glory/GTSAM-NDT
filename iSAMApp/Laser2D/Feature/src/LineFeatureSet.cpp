#include "stdafx.h"
#include "misc.h"
#include "scan.h"
#include "LineFeatureSet.h"
#include "FeatureCreationParam.h"
#include "DebugTrace.h"
#include "assert.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CFeatureCreationParam FeatureCreationParam;

///////////////////////////////////////////////////////////////////////////////

/*
** Calculates a line in the form (n1, n2) * (x, y) + c = 0,
** so that the leastsquare sum of the corresponding scan points
** is minimized.
** sp is an array of scan points, num specifies the number of scan points,
** n1, n2 and c are the result values.
*/
bool RegressionLine(const CScanPoint *sp, long num, CLineBase& ln);

/*
** Calculates variance of distance from a set of scan points
** given in (sp, num) to a line given in (n1, n2, c).
*/
float LineDeviation(CScanPoint *sp, long num, CLineBase& lb);

int MinPointsOnLine(float r, float fMinPointsOnLineRatio)
{
	int n = (int)(fMinPointsOnLineRatio / r);
	if (n < 20)
		n = 20;
	return n;
}

///////////////////////////////////////////////////////////////////////////////
//   “CLineFeatureSet”类的实现。

//
//   构造函数。
//
CLineFeatureSet::CLineFeatureSet(int nNum)
{
	Clear();
	m_rect.Clear();
}

//
//   “拷贝”构造函数。
//
CLineFeatureSet::CLineFeatureSet(const CLineFeatureSet& Obj, bool bFilterDisabled)
{
	Clear();

	for (int i = 0; i < (int)Obj.size(); i++)
	{
		// 根据需要，滤除那些被禁止的项
		if (bFilterDisabled && !Obj.at(i)->IsEnabled())
			continue;

		CLineFeature* p = Obj.at(i)->Duplicate();
		if (p == NULL)
			assert(false);
		else
			push_back(p);
	}

	m_Param = Obj.m_Param;    // 直线生成参数
	m_pstScanner = Obj.m_pstScanner;           // 激光头参考姿态
	
	UpdateCoveringRect();
}

//
//   在析构函数中释放所有已分配的内存。
//
CLineFeatureSet::~CLineFeatureSet()
{
	Clear();
}

//
//   重载“=”操作符。
//
void CLineFeatureSet::operator = (const CLineFeatureSet& Obj)
{
	Clear();

	m_rect = Obj.GetCoveringRect();
	m_Param = Obj.m_Param;    // 直线生成参数
	m_pstScanner = Obj.m_pstScanner;           // 激光头参考姿态

	for (int i = 0; i < (int)Obj.size(); i++)
	{
		CLineFeature* p = Obj.at(i)->Duplicate();
		if (p == NULL)
			assert(false);
		else
			push_back(p);
	}
}

//
//   根据直线特征类型分配空间。
//
CLineFeature* CLineFeatureSet::NewLineFeature(int nSubType)
{
	switch (nSubType)
	{
	case GENERIC_LINE_FEATURE:
		return new CLineFeature;

	case SINGLE_SIDED_LINE_FEATURE:
//		return new CSingleSidedLineFeature;

	default:
		return NULL;
	}
}

//
//   设置直线性特征生成参数。
//
void CLineFeatureSet::SetCreationParam(CLineFeatureCreationParam* pParam) 
{
	if (pParam != NULL)
		m_Param = *pParam; 
}

//
//   根据所提供的扫描到的直线数组生成直线特征集合。
//
//   注意：所有直线数组均必须为本地测量到的直线数据(即观测姿态为(0, 0, 0))
//
bool CLineFeatureSet::CreateFromLocalLines(int nNum, CLineFeature* pLineData)
{
	Clear();

	// 直线数据必须提供
	if (pLineData == NULL)
		return false;

	for (int i = 0; i < nNum; i++)
	{
		CLineFeature* p = pLineData[i].Duplicate();
		if (p == NULL)
			assert(false);
		else
			push_back(p);
	}

	// 设置直线特征的观测方向
	SetDetectPosture(CPosture(0, 0, 0));

	// 计算边界值
	UpdateCoveringRect();
	return true;
}

//
//   设置扫检测到这些直线特征时的激光头姿态，以便计算各条直线特征的观测方向。
//
void CLineFeatureSet::SetDetectPosture(const CPosture& pstDetect)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->SetDetectPosture(pstDetect);
}

//
//   从一个扫描集中抽取其有所有直线段。
//
bool CLineFeatureSet::CreateFromScan(const CScan& scan)
{
	Clear();

	const float fac = 5.0;
	const float div = 1.0 / fac;

	// 点云不能为空
	if (scan.m_nCount == 0)
		return false;

	m_pstScanner = scan.m_poseScanner;

	CScanPoint* sp = scan.m_pPoints;
	sp[0].m_nLineID = -1;

	long i, start = 0;
	float last_dist = 0.0; 

	for (i = 1; i < scan.m_nCount; i++)
	{
		// 计算当前点到上一点之间的距离
		float d = sp[i-1].DistanceTo(sp[i]);

		// 距离至少取10mm
		float dist = MAX(d, 0.01f);
		float rel = last_dist / dist;

		// 先假定该点所对应的直线不存在
		sp[i].m_nLineID = -1;

		// 当前点的极径
		float r = sp[i].r;

		// 直线段所含点的最小数量
		int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

		// 判断两点间距离是否过大
		bool bPointsDistTooBig = (dist > m_Param.fMaxDistPointToPoint * r);

		// 判断扫描点是否太不均匀(距离差距过大)
		bool bPointsDistChangeTooMuch = (i > start + 1 && (rel > fac || rel < div));

		// 如果点间距离太大，或者点间距离变化过块(不均匀)
		if (bPointsDistTooBig || bPointsDistChangeTooMuch)
		{
			// 如果这一段里面点的数量够一个线段，尝试分裂出一条新线段
			if (i - start >= m_Param.nMinPointsOnLine)
				SplitPoints(sp, start, i - 1);

			// 调整下一段的起始点位置
			start = i;
		}

		// 计算近两个距离的均值
		if (i <= start + 1)
			last_dist = dist;
		else
			last_dist = (last_dist + dist) / 2;
	}

	// 直线段所含点的最小数量
	float r = sp[i].r;
	int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

	// 如果从start到i点所含的点的数量够一个最小线段所需的数量，尝试分裂出一条直线
	if (i - start >= nMinPointsOnLine)
		SplitPoints(sp, start, i - 1);

	// 合并共线的直线特征
	LineScanMergeLines(&((CScan&)scan), NULL);

	// 删除那些扫描角太小的直线特征
	RemoveBadLines(m_pstScanner, scan);

	for (i = 0; i < (int)size(); i++)
	{
		at(i)->m_nId = i+1;
		at(i)->ComputeParam();

		// 附加上范围
		CRange range(0, at(i)->Length());
		at(i)->m_Ranges.push_back(range);
	}

	// 设置直线特征的观测方向
	SetDetectPosture(m_pstScanner);

	// 计算边界值
	UpdateCoveringRect();

	return true;
}

//
//   删除那些扫描角不佳的直线特征。
//
void CLineFeatureSet::RemoveBadLines(const CPosture& pstScanner, const CScan& scan)
{
	CPnt ptScanner = m_pstScanner;

	for (int i = 0; i < (int)size(); i++)
	{
		CLineFeature* pLineFeature = at(i);

		// 取直线特征的中点
		CPnt ptMid = pLineFeature->GetMidpoint();

		// 构造扫描线
		CLine ln1(ptScanner, ptMid);
		
		// 计算直线特征与扫描线之间的夹角
		CAngle angDiff = ln1.AngleToUndirectionalLine(*pLineFeature);
		float fAngDiff = angDiff.m_fRad;
		if (angDiff > PI / 2)
			fAngDiff = PI - fAngDiff;

		// 如果这个夹角小于指定的门限值，则认为此直线特征不合格
		bool bBad = fAngDiff < m_Param.fMinScanToLineAngle;

		// 如果夹角合格，下面核对点间距离
		if (!bBad)
		{
			float fDistLimit = (float)(m_Param.fMaxDistPointToPoint * ln1.Length() / sin(fAngDiff));

			CScanPoint* sp = scan.m_pPoints;
			for (long j = pLineFeature->m_lStart; j < pLineFeature->m_lEnd - 1; j++)
			{
				// 取得相邻的两个扫描点
				CPnt pt1 = sp[j].GetPntObject();
				CPnt pt2 = sp[j + 1].GetPntObject();

				// 计算这两个点在直线特征上的投影点
				CPnt ptFoot1, ptFoot2;
				pLineFeature->DistanceToPoint(false, pt1, NULL, &ptFoot1);
				pLineFeature->DistanceToPoint(false, pt2, NULL, &ptFoot2);

				// 计算两个投影点之间的距离
				float d = ptFoot1.DistanceTo(ptFoot2);

				// 如果两个相邻扫描点之间的距离大于极限值，说明此直线特征不合格
				if (d > fDistLimit)
				{
					bBad = true;
					break;
				}
			}
		}

		// 删除不合格的直线特征
		if (bBad)
		{
			delete at(i);
			erase(begin() + i);
			i--;
		}
	}
}

//
//   根据当前姿态、最大扫描半径和直线模型来生成直线特征集合。
//
bool CLineFeatureSet::CreateFromWorldLines(CPosture& pst, float fMaxRange, int nNumOfLines, 
												  CLine* pLines)
{
	Clear();

	// 构造一个“范围圆”
	CCircle RangeCircle(pst.GetPntObject(), fMaxRange);

	// 用这个圆截取各条直线，得到的直线将被加入到直线特征集合中
	int nResultCount = 0;
	CLineFeature* pNewLine = new CLineFeature;

	for (int i = 0; i < nNumOfLines; i++)
	{
		// 如果有剪切到长度大于300mm的线段
		if (RangeCircle.CutLine(pLines[i], *pNewLine) && (pNewLine->Length() > 300))       // 0.3米，单位有误!!
		{
			push_back(pNewLine);
		}
	}

	// 通过设置观测姿态来计算各直线特征的有效朝向
	SetDetectPosture(pst);

	// 更改特征数量(此举将导致存储空间中有多余未用的部分)
//	m_nCount = nResultCount;

	// 计算边界值
	UpdateCoveringRect();

	return true;
}

//
//   清除对象内的所有的直线。
//
void CLineFeatureSet::Clear()
{
	for (int i = 0; i < (int)size(); i++)
		delete at(i);

	clear();
	m_rect.Clear();
}

//
//   分配内存并复制当前对象内容，返回新复制的对象指针。
//
CLineFeatureSet* CLineFeatureSet::Duplicate()
{
	// 为新副本的各直线段分配空间
	CLineFeatureSet *copy = new CLineFeatureSet;

	if (copy == NULL)
		return NULL;

	*copy = *this;
	return copy;
}

//
//   将此另一个直线段集并入此直线段集中。
//
bool CLineFeatureSet::Merge(const CLineFeatureSet& LineScan)
{
	for (int i = 0; i < (int)LineScan.size(); i++)
	{
		CLineFeature* p = LineScan.at(i)->Duplicate();
		if (p == NULL)
			return false;
		else
		{
			push_back(p);
			m_rect += *p;
		}
	}


	return true;
}

//
//   增加一个直线特征。
//   (目前此函数效率很低，需要频繁释放/分配内存，将来改进)
//
bool CLineFeatureSet::Add(const CLineFeature& LineFeature)
{
	CLineFeature* p = LineFeature.Duplicate();
	if (p == NULL)
		return false;
	else
	{
		push_back(p);
		m_rect += *p;
	}

	return true;
}

//
//   通过合并共线的线段来简化此直线段集合。
//
//   说明：当两条无重叠区域但共线的线段之间的最小距离小于fMaxGapBetweenLines时，
//   可以将这两条线段进行合并(即认为断续区很小，可以忽略)。
//
bool CLineFeatureSet::Simplify(float fMaxGapBetweenLines)
{
	bool bChange;
	int i, j;

	// 先生成一个线段集的附本，并为每个线段标明是否“启用”
	CLineFeatureSet* pScan1 = Duplicate();
	bool* pUse = new bool[size()];

	do {
		// 先标明所有项都应启用
		for (i = 0; i < (int)pScan1->size(); i++)
			pUse[i] = true;

		// 再将pScan1复制一份(得到pScan2)，作为比较之用
		CLineFeatureSet* pScan2 = pScan1->Duplicate();
		bChange = false;

		// 尝试对所有的线段进行逐项合并
		for (i = 0; i < (int)pScan1->size(); i++)
		{
			// 跳过那些已标明“不启用”的线段
			if (!pUse[i])
				continue;

			for (j = i + 1; j < (int)pScan1->size(); j++)
			{
				//	if (!pUse[j])
				//	  continue;

				// 如果合并成功，则标明第二个线段为“不启用”
				if (pScan1->at(i)->ColinearMerge(*pScan1->at(j), m_Param.fMaxLineMergeAngle, 
					m_Param.fMaxLineMergeDist, fMaxGapBetweenLines))
				{
					pUse[j] = false;
					bChange = true;
				}
				else if (pScan1->at(j)->ColinearMerge(*pScan1->at(i), m_Param.fMaxLineMergeAngle, 
					m_Param.fMaxLineMergeDist, fMaxGapBetweenLines))
				{
					pUse[i] = false;
					bChange = true;
				}
			}
		}

		// 将带有“空洞”的数组进行“挤压”，使之成为紧凑排列的数组(即所有成员都是启用的)
		for (int j = pScan1->size() - 1; j >= 0; j--)
		{
			if (!pUse[j])
			{
				delete pScan1->at(j);
				pScan1->erase(pScan1->begin() + j);
			}
		}

		delete pScan2;                 // 释放pScan2
	} while (bChange);                // 一直处理到不能再合并

	Clear();
	*this = *pScan1;

	delete pScan1;
	delete []pUse;

	return true;
}

//
//   在集合中找到所有共线的特征，并进行分组记录。
//   返回值：最大的组号。
//
int CLineFeatureSet::FindColinearGroups(float fMaxAngDiff, float fMaxDistDiff)
{
	// 先将所有分组标志置为-1
	for (int i = 0; i < (int)size(); i++)
	{
		CLineFeature* pLine = at(i);
		pLine->m_nId = -1;
	}

	// 下面对所有直线段按照共线情况进行分组
	int nNextGroupId = 0;
	for (int i = 0; i < (int)size(); i++)
	{
		// 取第一个直线特征
		CLineFeature* pLine1 = at(i); 

		// 跳过已分组的特征
//		if (Line1.m_nGroupId >= 0)
		if (pLine1->m_nId >= 0)
			continue;
		else
			pLine1->m_nId = nNextGroupId++;
		//Line1.m_nGroupId = nNextGroupId++;

		for (int j = i + 1; j < (int)size(); j++)
		{
			// 取第二个直线特征
			CLineFeature* pLine2 = at(j);

			// 跳过已分组的特征
//			if (Line2.m_nGroupId >= 0)
			if (pLine2->m_nId >= 0)
				continue;

			// 如果两个特征共线，标明为同一组
			if (pLine1->IsColinearWith(*pLine2, fMaxAngDiff, fMaxDistDiff))
				pLine2->m_nId = pLine1->m_nId;
//			Line2.m_nGroupId = Line1.m_nGroupId;
		}
	}

	return nNextGroupId;
}

//
//   针对共线的特征，通过多段线的描述方式进行合并。
//
bool CLineFeatureSet::ColinearSimplify(float fMaxAngDiff, float fMaxDistDiff)
{
	// 先按照是否共线进行分组
	int nMaxGroupId = FindColinearGroups(fMaxAngDiff, fMaxDistDiff);

	// 以多段线的方式重新描述所有特征
	CLineFeatureSet setNew;

	for (int i = 0; i < nMaxGroupId; i++)
	{
		// 建立一个临时集合
		CLineFeatureSet temp;

		// 查找所有分组标识为i的特征，并将它们加入到临时集合中
		for (int j = 0; j < (int)size(); j++)
		{
			CLineFeature* pLine = at(j);

			// 找到所有属于第i组的特征
//			if (Line.m_nGroupId == i)
			if (pLine->m_nId == i)
				temp.Add(*pLine);
		}

		// 对此临时集合进行优化处理
		temp.ColinearRectify();

		// 结果加入到新集合中
		setNew.Merge(temp);
	}

	*this = setNew;
	return true;
}

//
//   优化共线特征组。
//
CLineFeatureSet* CLineFeatureSet::Optimize()
{
	CLineFeatureSet* pNew = Duplicate();
	pNew->ColinearSimplify(CAngle::ToRadian(5), 0.08f);
	return pNew;
}

//
//   对所有特征进行共线调理。
//
bool CLineFeatureSet::ColinearRectify()
{
	if (size() < 2)
		return true;

	// 分配直线段数组空间
	CLine* pLines = new CLine[size()];
	for (int i = 0; i < (int)size(); i++)
		pLines[i] = *at(i);

	// 生成等价多段线
	CMultiSegLine MultiLine;
	MultiLine.Create(pLines, size());

	// 生成唯一一个特征
	CLineFeature Feature;
	Feature.Create(MultiLine);
//	Feature.m_nGroupId = at(0).m_nGroupId;
	Feature.m_nId = at(0)->m_nId;

	// 清除集合内容，再把这唯一一个特征加入集合中
	Clear();
	Add(Feature);

	delete[]pLines;

	return true;
}

//
//   删除指定的线段。
//
bool CLineFeatureSet::DeleteAt(int nIdx)
{
	if (nIdx >= (int)size() - 1 || nIdx < 0)
		return false;

	delete at(nIdx);
	erase(begin() + nIdx);

	UpdateCoveringRect();
	return true;
}

//
//   计算所有直线段的总长度。
//
float CLineFeatureSet::TotalLength()
{
	float fSum = 0;
	for (long i = 0; i < (int)size(); i++)
		fSum += at(i)->m_fTotalLen;

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
float FindMaxDist(const CScanPoint *sp, long num, CLineBase& ln, long* pMaxDistIdx)
{
	if (pMaxDistIdx != NULL)
		*pMaxDistIdx = 0;

	float fMaxDist = 0;
	for (long i = 0; i < num; i++)
	{
		// 计算点到直线的距离
		float d = ln.DistanceToPoint(sp[i]);

		// 始终保持最大距离值
		if (d > fMaxDist)
		{
			fMaxDist = d;            // 更新最远距离值

			if (pMaxDistIdx != NULL)
				*pMaxDistIdx = i;     // 最远距离点所对应的序号
		}
	}

	return fMaxDist;
}

//
//   分析一下是否可以找到一个距离线段的端点更近的“好”断点，这样可以针对平行的墙面提高直线提取效果。
//   注意：
//      点(x0, y0)到直线  n1*x +n2*y + c = 0 的距离公式为：
//	     d = abs(n1 * x0 + n2 * y0 + c)/sqrt(n1*n1 + n2*n2)
//
static long RefineBreakPoint(const CScanPoint *sp, long num, long brk, CLineBase& lb)
{
	// 取得当前断点坐标
	CPnt pt(sp[brk]);

	// 计算该断点到直线的距离
	float fMaxDist = lb.DistanceToPoint(pt);

	// 计算“最短距离1”- 比上面的距离近15个点
	float fMinD1 = fMaxDist - MAX_SIGMA / 2.0;

	// 计算“最短距离2”- 为断点处距离的80%
	float fMinD2 = fMaxDist * 0.8;

	// 取上面两个距离的较大值
	float fMinD = MAX(fMinD1, fMinD2);

	long end = num - 1;

	// 计算该断点到起点的距离
	float fStartD = pt.DistanceTo(sp[0]);

	// 计算该断点到终点的距离
	float fEndD = pt.DistanceTo(sp[end]);

	// 如果离起点较近
	if (fStartD < fEndD)
	{
		// 看看从起点到该断点处有没有距离直线偏离大于要求的“最小距离”的点
		for (long i = 1; i < brk; i++)
			if (lb.DistanceToPoint(sp[i]) > fMinD)
				return i;
	}
	// 如果离终点较近
	else
	{
		// 看看从终点到该断点处有没有距离直线偏离更大的点
		for (long i = end - 1; i > brk; i--)
			if (lb.DistanceToPoint(sp[0]) > fMinD)
				return i;
	}

	return brk;
}

//
//  判断扫描点的连线是否来回弯曲，不够平直。
//
static bool IsZigZag(const CScanPoint *sp, long num, long brk, CLineBase& lb)
{
	static const float eps = 0.01;

	long i;
	long lNegCount = 0;     // 负向距离点计数
	long lPosCount = 0;     // 正向距离点计数

	// 先分析起点到断点之间的所有点
	for (i = 0; i < brk; i++)
	{
		// 计算点到直线的距离
		float val = lb.DistanceToPoint(sp[i]); //sp[i].x * n1 + sp[i].y * n2 + c;

		// 计算那些距离直线偏离较大的点的数量(分正向偏离和负向偏离两种情况)
		if (val < -eps)
			lNegCount++;
		else if (val > eps)
			lPosCount++;
	}

	// 如正向偏离、负向偏离读数值中的一方比另一方的2倍还大，可认为是“弯弯曲曲”
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return true;

	// 重新计数
	lNegCount = lPosCount = 0;

	// 再分析断点到终点之间的所有点，原理同上
	for (i = brk + 1; i < num; i++)
	{
		// 计算点到直线的距离
		float val = lb.DistanceToPoint(sp[i]); //sp[i].x * n1 + sp[i].y * n2 + c;

		// 计算那些距离直线偏离较大的点的数量(分正向偏离和负向偏离两种情况)
		if (val < -eps)
			lNegCount++;
		else if (val > eps)
			lPosCount++;
	}

	// 如正向偏离、负向偏离读数值中的一方比另一方的2倍还大，可认为是“弯弯曲曲”
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return true;

	return false;
}

//
//   对位于指定序号的扫描点中的直线进行优化，找到准确的起点和终点。
//   注意：如此找到的直线段其实有可能还需要进行再次分裂，进而提取出多个直线段。
//
//   返回值：
//      true - 优化成功，得到了新的lStart, lEnd值
//      false - 这个区间的扫描点提取不出有效的直线
//
bool RefineLineBetween(CScanPoint* sp, long& lStart, long& lEnd, float& sigma2, CLineFeatureCreationParam* pParam)
{
	const float max_sigma2 = MAX_SIGMA * MAX_SIGMA;
	const float thresh1 = max_sigma2 / 2.0;               // 门限值1

	long num_points = lEnd + 1 - lStart;

	// 构造成当前直线
	CPnt ptStart(sp[lStart]), ptEnd(sp[lEnd]);
	CLine ln(ptStart, ptEnd);

	// 现在需要确定所有这些点所处直线的标准参数(n1, n2, c)
	if (!RegressionLine(sp + lStart, num_points, ln))
		return false;

	// 确定该直线的起始点(沿正方向顺序查找)
	long i;
	for (i = lStart; i < lEnd; i++)
	{
		float val = ln.DistanceToPoint(sp[i]);
		val = val * val;
		sigma2 += val;

		if (val < thresh1)
			break;		  // 该点已很靠近直线，找到起始点
	}

	long lNewStart = i;       // 起始点序号

	// 当前起始点的极径
	float r = sp[lNewStart].r;
	int nMinPointsOnLine = MinPointsOnLine(r, pParam->nMinPointsOnLine);

	// 确定该直线的终止点(沿反方向逆序查找)
	for (i = lEnd; i > lNewStart; i--)
	{
		float val = ln.DistanceToPoint(sp[i]);
		val = val * val;
		sigma2 += val;

		if (val < thresh1)
			break;		  // 该点已很靠近直线，找到终止点
	}

	long lNewEnd = i;         // 终止点序号

	// 如果所包含的点太少，则无法组成线段
	if (lNewEnd + 1 - lNewStart < nMinPointsOnLine - 2)
		return false;

	return true;
}

//
//   将指定的扫描点分裂并抽取出直线段。
//
void CLineFeatureSet::SplitPoints(CScanPoint *sp, long lStart, long lEnd)
{
	bool refine_break_point = true;
	long num_points = lEnd + 1 - lStart;

	// 构造成当前直线
	CPnt ptStart(sp[lStart]), ptEnd(sp[lEnd]);
	CLine ln(ptStart, ptEnd);

	// 判断直线长度是否太短
	if (ln.Length() < m_Param.fMinLineLength)
		return;		 // 太短

	// 从lStart点开始，计算出到直线(n1*x + n2*y + c = 0)最远点的序号，并求出该最远距离(存于fMaxDist中)
	long lMaxDistIdx;
	float fMaxDist = FindMaxDist(&sp[lStart], num_points, ln, &lMaxDistIdx);
	long brk = lStart + lMaxDistIdx;

	// 如果上面求得的最远距离大于最大允许距离，则直线需要分裂
	bool bSplit = (fMaxDist > m_Param.fMaxDistPointToLine);

	// 如果该直线不必分裂
	if (!bSplit)
	{
		const float max_sigma2 = MAX_SIGMA * MAX_SIGMA;
		const float thresh2 = max_sigma2 * 16.0;              // 门限值2

		long lNewStart = lStart, lNewEnd = lEnd;
		float sigma2 = 0;

		// 如果所包含的点太少，则无法组成线段，需要彻底分裂
		if (!RefineLineBetween(sp, lNewStart, lNewEnd, sigma2, &m_Param))
			bSplit = true;
		else
		{
			// 试图在剩下的段中找到一个新的距离较远的断点
			long newbrk = -1;

			for (long i = lNewStart + 1; i < lNewEnd; i++)
			{
				float val = ln.DistanceToPoint(sp[i]);
				val = val * val;
				sigma2 += val;

				if (val > thresh2 && newbrk < 0)
					newbrk = i;
			}

			sigma2 /= num_points;
			if (sigma2 > max_sigma2)
				bSplit = true;

			// 如果继续可分
			else if (newbrk >= 0)
			{
				brk = newbrk;
				refine_break_point = false;
				bSplit = true;
			}

			// 否则，不可能再分了
			else
			{
				// 增加一条线段
				long new_num = lNewEnd + 1 - lNewStart;

				if (num_points - new_num > 3)
					RegressionLine(&sp[lNewStart], new_num, ln);

				// 当前起始点的极径
				float r = sp[lNewStart].r;
				int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

				// 如果位于lNewStart之前的点的数量也够一条线段，应对那些点也进行分裂
				if (lNewStart + 1 - lStart >= nMinPointsOnLine)
					SplitPoints(sp, lStart, lNewStart);

				CPnt ptStart1(sp[lNewStart]), ptEnd1(sp[lNewEnd]);
				CLine ln1(ptStart1, ptEnd1);

				// 计算线段长度
				float fDist = ln1.Length();
				float fSigmaRatio = sqrt(sigma2) / fDist;

				// 如果线段的长度超过“最小长度”，且平均距离误差小于指定门限，可确认找到一个有效线段
				if (fDist >= m_Param.fMinLineLength && fSigmaRatio < MAX_SIGMA_RATIO)
				{
					// 将这些处于该直线上的点标记上直线的序号
					for (long i = lNewStart; i <= lNewEnd; i++)
						sp[i].m_nLineID = (int)size();

					CLineFeature* pLine = new CLineFeature(ptStart1, ptEnd1);
					pLine->m_lStart = lNewStart;
					pLine->m_lEnd = lNewEnd;
					pLine->m_fSigma2 = sigma2;

					push_back(pLine);
				}

				// 如果位于lNewEnd之后的点的数量也够一条线段，应对那些点也进行分裂
				if (lEnd + 1 - lNewEnd >= nMinPointsOnLine)
					SplitPoints(sp, lNewEnd, lEnd);
			}
		}
	}

	// 如果需要进一步分裂
	if (bSplit)
	{
		if (refine_break_point)
		{
			brk = lStart + RefineBreakPoint(&sp[lStart], lEnd + 1 - lStart, brk - lStart, ln);
		}

		// 当前极径
		float r = sp[lStart].r;
		int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

		// 如果从起始点到断点处还有较多点，则这一段需要继续分裂
		if (brk + 1 - lStart >= nMinPointsOnLine)
			SplitPoints(sp, lStart, brk);

		// 如果从断点到起始点处还有较多点，则这一段需要继续分裂
		if (lEnd + 1 - brk >= nMinPointsOnLine)
			SplitPoints(sp, brk, lEnd);
	}
}

///////////////////////////////////////////////////////////////////////////////

#define LINE_MERGE_MAX_SIGMA2 (25.0 * 25.0)
#define LINE_MERGE_MAX_SIGMA2_FAC 2.0

#define MIN_COS_ALPHA       cos(DEG2RAD(30.0))

//
//   对指定的点云段进行线性回归直线拟合。
//   说明：
//     sp  - 点云数组缓冲区指针
//     num - 点的数量
//
bool RegressionLine(const CScanPoint *sp, long num, CLineBase& lb)
{
	float xm = 0.0, ym = 0.0, xxm = 0.0, yym = 0.0, xym = 0.0;
	float a, dx, dy;
	long i;

	if (num <= 1)
		return false;

	for (i = 0; i < num; i++)
	{
		float x = sp[i].x;
		float y = sp[i].y;

		xm += x;
		ym += y;
		xxm += x*x;
		yym += y*y;
		xym += x*y;
	}

	a = 0.5f * atan2((float)(-2.0*(xym - xm*ym / num)), (float)(yym - ym*ym / num - xxm + xm*xm / num));
	float n1 = cos(a);
	float n2 = sin(a);
	dx = sp[num - 1].x - sp[0].x;
	dy = sp[num - 1].y - sp[0].y;

	if (dx * n2 - dy * n1 > 0.0)
	{
		n1 = -n1;
		n2 = -n2;
	}
	float c = -(n1 * xm + n2 * ym) / num;
	
	lb.DirectCreate(n1, n2, c);
	return true;
}

float LineDeviation(CScanPoint *sp, long num, CLineBase& lb)
{
	float sum = 0.0, sigma2;
	long i;

	for (i = 0; i < num; i++)
	{
		float val = lb.DistanceToPoint(sp[i]);
		sum += val * val;
	}

	sigma2 = sum / (float)num;
	return sigma2;
}

//
//   将那些共线且相连的线段合并。
//
void CLineFeatureSet::LineScanMergeLines(CScan *scan, long *lineNum)
{
	static const float fac = 1.0;

	long i, j;

	if (scan == NULL)
		return;

	CScanPoint* tmp = (CScanPoint *)SSmalloc(scan->m_nCount * sizeof(scan->m_pPoints[0]));
	if (tmp == NULL)
		return;

	CScanPoint* sp = scan->m_pPoints;
	long totalLines = size(); //m_nCount;

	bool change = true;
	while (change)
	{
		change = false;

		// 将所有的直线段两两对比，看是否共线且相连
		for (i = 0; i < (int)size() - 1; i++)
			for (j = i + 1; j < (int)size(); j++)
			{
				// 依次取得两条线段
				CLineFeature *ln1 = at(i);
				CLineFeature *ln2 = at(j);

				// 计算两条直线的夹角？
				float cosa = ln1->a * ln2->a + ln1->b * ln2->b;

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

				d1 = ln1->DistanceToPoint(false, ln2->m_ptStart, &lambda1);
				d2 = ln1->DistanceToPoint(false, ln2->m_ptEnd, &lambda2);

				if (d1 > fac * m_Param.fMaxDistPointToLine)
					continue;

				if (d2 > fac * m_Param.fMaxDistPointToLine)
					continue;

				if ((lambda1 < 0.0 || lambda1 > 1.0) && (lambda2 < 0.0 || lambda2 > 1.0))
					continue;

				k = 0;
				l1 = ln1->m_lStart;
				l2 = ln2->m_lStart;
				n1 = ln1->a;
				n2 = ln1->b;

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

				CLineBase lb;
				if (!RegressionLine(tmp, k, lb))
					continue;

				n1 = lb.a;
				n2 = lb.b;
				c = lb.c;

				maxSigma2 = LINE_MERGE_MAX_SIGMA2_FAC * (ln1->m_fSigma2 + ln2->m_fSigma2);
				maxSigma2 = MIN(maxSigma2, LINE_MERGE_MAX_SIGMA2);


				if ((lambda1 >= 0.0 && 1.0 - lambda1 > lambda2 - 1.0) || (lambda2 <= 1.0 && lambda2 > -lambda1))
					maxSigma2 *= 20.0;
				else
				{
					// 计算扫描点序列tmp到直线lb最远的一点(在序号brk处)
					long brk;
					FindMaxDist(tmp, k, lb, &brk);
					brk = RefineBreakPoint(tmp, k, brk, lb);

					// 如果直线段是“歪歪扭扭”的
					if (IsZigZag(tmp, k, brk, lb))
						continue;
				}

				num1 = ln1->m_lEnd + 1 - ln1->m_lStart;

				sigma2 = LineDeviation(&sp[ln1->m_lStart], num1, lb);

				if (sigma2 > maxSigma2)
					continue;

				num2 = ln2->m_lEnd + 1 - ln2->m_lStart;
				sigma2 = LineDeviation(&sp[ln2->m_lStart], num2, lb);

				if (sigma2 > maxSigma2)
					continue;

				sigma2 = LineDeviation(tmp, k, lb);
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
					long size0 = ln2->m_lStart - src;

					memmove(&sp[dst], &sp[src], size0 * sizeof(*sp));

					for (l = 0; l < (long)size(); l++)
					{
						CLineFeature *ln3 = at(l);
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
				x2 = tmp[k - 1].x;
				y2 = tmp[k - 1].y;

				val = c + n1 * x1 + n2 * y1;
				x1 -= val * n1;
				y1 -= val * n2;

				val = c + n1 * x2 + n2 * y2;
				x2 -= val * n1;
				y2 -= val * n2;

				dist = _hypot(x2 - x1, y2 - y1);

				if (dist < 0.01f)
					continue;

				k = ln2->m_lEnd + 1 - ln1->m_lStart;
				appdist = dist / (float)k;

				ln1->m_lEnd = ln2->m_lEnd;
				ln1->m_ptStart.x = x1;
				ln1->m_ptStart.y = y1;
				ln1->m_ptEnd.x = x2;
				ln1->m_ptEnd.y = y2;
				ln1->m_fTotalLen = dist;
				ln1->a = n1;
				ln1->b = n2;
				ln1->c = c;
				ln1->m_fSigma2 = sigma2;

				if (ln1 != at(i))
					*at(i) = *ln1;

				memcpy(&sp[ln1->m_lStart], tmp, k * sizeof(*sp));  // 此处改变了原来的扫描点????

				delete at(j);
				erase(begin() + j);
				change = true;
				j--;
			}
	}


	// 更新各点对应的直线段编号
	for (long i = 0; i < (long)size(); i++)
	{
		CLineFeature *ln = at(i);
		for (long j = ln->m_lStart; j <= ln->m_lEnd; j++)
		{
			long old = sp[j].m_nLineID;

			if (lineNum != NULL && old >= 0 && old < totalLines)
				lineNum[old] = i;

			sp[j].m_nLineID = i;
		}
	}

	SSfree(tmp);
}

///////////////////////////////////////////////////////////////////////////////

//
//   去掉所有长度短于minLineLength的直线。
//
void CLineFeatureSet::LengthFilter(float minLineLength)
{
	for (int i = size() - 1; i >= 0; i--)
		if (at(i)->m_fTotalLen < minLineLength)
		{
			delete at(i);
			erase(begin() + i);
		}
}

//
//   判断直线扫描集是否包含指定的点。
//
bool CLineFeatureSet::ContainScanPoint(const CScanPoint& sp)
{
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i)->DistanceToPoint(true, sp) < 50)
			return true;
	}

	return false;
}

//
//   移除位于指定区域内的线段。
//
void CLineFeatureSet::RemoveWithin(const CRectangle& r)
{
	for (int j = size() - 1; j >= 0; j--)
	{
		if (r.Contain(*at(j)))
		{
			delete at(j);
			erase(begin() + j);
		}
	}
}

//
//   将特征进行平移。
//
void CLineFeatureSet::Move(float fX, float fY)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Move(fX, fY);
}

//
//   将特征进行旋转。
//
void CLineFeatureSet::Rotate(CAngle ang, CPnt ptCenter)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Rotate(ang, ptCenter);
}

//
//   选中/取消选中指定的线段。
//
void CLineFeatureSet::Select(int nIdx, bool bOn)
{
	if (nIdx < 0)
	{
		for (int i = 0; i < (int)size(); i++)
			at(i)->Select(bOn);
	}
	else
		at(nIdx)->Select(bOn);
}

//
//   从文本文件装入直线特征集合。
//   返回值：
//     < 0 : 读取失败
//     >= 0: 读取的特征数量
//
int CLineFeatureSet::LoadText(FILE* fp)
{
	Clear();

	// 先读入直线数量
	int nCount;
	if (fscanf(fp, "%d\n", &nCount) != 1 || nCount < 0)
		return -1;

	// 依次读入各直线特征
	for (int i = 0; i < nCount; i++)
	{
		// 读入直线类型
		int nSubType;
		if (fscanf(fp, "%d\t", &nSubType) != 1)
			return -1;

		// 根据类型分配空间
		CLineFeature* pFeature = NewLineFeature(nSubType);
		if (pFeature == NULL)
			return -1;

		// 根据类型读入直线特征数据
		if (pFeature->LoadText(fp) < 0)
			return -1;

		push_back(pFeature);
	}

	UpdateCoveringRect();

	return true;
}

//
//   将直线特征集合存到文本文件。
//
int CLineFeatureSet::SaveText(FILE* fp)
{
	// 先写直线数量
	int nCount = (int)size();
	fprintf(fp, "%d\n", nCount);

	// 依次写入各直线特征
	for (int i = 0; i < nCount; i++)
		at(i)->SaveText(fp);

	return nCount;
}

//
//   从二进制文件装入直线特征集合。
//
int CLineFeatureSet::LoadBinary(FILE* fp)
{
	Clear();

	// 先读入直线数量
	int nCount;
	if (fread(&nCount, sizeof(int), 1, fp) != 1 || nCount < 0)
		return -1;

	// 依次读入各直线特征
	for (int i = 0; i < nCount; i++)
	{
		// 读入直线类型
		int nSubType;
		if (fread(&nSubType, sizeof(int), 1, fp) != 1)
			return -1;

		// 根据类型分配空间
		CLineFeature* pFeature = NewLineFeature(nSubType);
		if (pFeature == NULL)
			return -1;

		// 根据类型读入直线特征数据
		if (!pFeature->LoadText(fp))
			return -1;

		push_back(pFeature);
	}

	UpdateCoveringRect();

	return nCount;
}

//
//   将直线特征集合存到二进制文件。
//
int CLineFeatureSet::SaveBinary(FILE* fp)
{
	// 先写直线数量
	int nCount = (int)size();
	if (!fwrite(&nCount, sizeof(int), 1, fp) != 1)
		return -1;

	// 依次写入各直线特征
	for (int i = 0; i < nCount; i++)
		if (at(i)->SaveBinary(fp) < 0)
			return -1;

	return nCount;
}

//
//   重新计算边界值。
//
void CLineFeatureSet::UpdateCoveringRect()
{
	m_rect.Clear();

	for (int i = 0; i < (int)size(); i++)
		m_rect += *at(i);
}

//
//   根据直线特征集合生成角点集合。
//
int CLineFeatureSet::CreateCornerPoints(vector<CPointFeature>& ptCorners)
{
	// 下面生成所有两两相交直线的交点集合，作为“角点特征”
	ptCorners.clear();

	for (int i = 0; i < (int)size() - 1; i++)
	{
		CLineFeature* pLine1 = at(i);
		if (pLine1->Length() < MIN_LINE_LEN)
			continue;

		for (int j = i + 1; j < (int)size(); j++)
		{
			CLineFeature* pLine2 = at(j);
			if (pLine2->Length() < MIN_LINE_LEN)
				continue;

			// 计算两条直线的差角
			CAngle angDiff = pLine1->SlantAngle() - pLine2->SlantAngle();
			if ((angDiff > MIN_CORNER_ANGLE && angDiff < (PI - MIN_CORNER_ANGLE)) ||
				(angDiff >(PI + MIN_CORNER_ANGLE) && angDiff < (2 * PI - MIN_CORNER_ANGLE)))
			{
				float x, y;
				bool bOnLine1 = false;
				bool bOnLine2 = false;

				// 计算两条直线段的交点，并判断交点是否在两条线段上
				if (pLine1->Intersect(*pLine2, &x, &y, &bOnLine1, &bOnLine2))
				{
					CPointFeature pt;
					pt.x = x;
					pt.y = y;
					pt.m_nType = 1;                  // 角点类型
					pt.m_nParam[0] = pLine1->m_nId;      // 直线1的ID号
					pt.m_nParam[1] = pLine2->m_nId;      // 直线2的ID号
#if 0
					pt.m_fParam[0] = pLine1->SlantAngle().m_fRad;  // 直线1的倾角
					pt.m_fParam[1] = pLine2->SlantAngle().m_fRad;  // 直线2的倾角
#endif
					float fDist;

					// 如果交点不在直线1上
					if (!bOnLine1)
					{
						pLine1->FindNearPoint(pt, &fDist);
						float fLen1 = pLine1->Length();

						// 对于很短的直线特征，要求延长长度不能超过直线特征自身的长度
						if (fLen1 < MIN_LINE_LEN * 3)
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
						pLine2->FindNearPoint(pt, &fDist);
						float fLen2 = pLine2->Length();

						// 对于很短的直线特征，要求延长长度不能超过直线特征自身的长度
						if (fLen2 < MIN_LINE_LEN * 3)
						{
							if (fDist > fLen2)
								continue;
						}

						// 对于一般长度的直线特征，延长长度不能超过规定的固定长度(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// 如果该点与先前加入的点太近，则不将它加入
					if (/*!PointTooCloseToSomeCorner(pt, ptCorners)*/1)
						ptCorners.push_back(pt);
				}
			}
		}
	}
	return (int)ptCorners.size();
}

//
//   判断给定的点是否与某个角点距离过近。
//
bool CLineFeatureSet::PointTooCloseToSomeCorner(CPnt& pt, vector<CPnt>& ptCorners)
{
	bool bTooClose = false;
	for (vector<CPnt>::iterator iter = ptCorners.begin(); iter != ptCorners.end(); iter++)
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
//   从二进制文件中装入用户编辑数据。
//
bool CLineFeatureSet::LoadUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		if (fread(&n, sizeof(int), 1, fp) != 1)
			return false;

		// 数据为0时特征使能
		at(i)->Enable(n == 0);
	}

	return true;
}

//
//   将用户编辑数据保存到二进制文件中。
//
bool CLineFeatureSet::SaveUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		// 特征使能时置0
		n = !(at(i)->IsEnabled());
		if (fwrite(&n, sizeof(int), 1, fp) != 1)
			return false;
	}

	return true;
}

//
//   从另外一个LineFeatureSet复制其用户使能设置。
//
bool CLineFeatureSet::CopyUserData(const CLineFeatureSet& another)
{
	if (another.size() != size())
		return false;

	for (int i = 0; i < (int)size(); i++)
	{
		bool bEnabled = another.at(i)->IsEnabled();
		at(i)->Enable(bEnabled);
	}

	return true;
}

//
//   将直线集合转换到局部坐标系中，转换后的原点姿态落到指定的姿态处。
//
void CLineFeatureSet::Transform(CPosture& pstLocal)
{
	// 建立坐标变换
	CTransform trans(pstLocal);

	// 进行直线的坐标变换
	for (int i = 0; i < (int)size(); i++)
	{
		// 先取得直线段的两个端点
		CPnt pt1 = at(i)->m_ptStart;
		CPnt pt2 = at(i)->m_ptEnd;

		// 将两个端点依次变换到局部坐标系下
		CPnt ptLocal1 = trans.GetLocalPoint(pt1);
		CPnt ptLocal2 = trans.GetLocalPoint(pt2);

		// 生成变换后的直线段
		at(i)->Create(ptLocal1, ptLocal2);
	}

	// 调整边界值
	UpdateCoveringRect();
}

//
//   将直线集合转换到世界坐标系中，转换后原来的原点姿态需要到达指定的姿态。
//
void CLineFeatureSet::InvTransform(CPosture& pstOrigin)
{
	// 建立坐标变换
	CTransform trans(pstOrigin);

	// 进行直线的坐标变换
	for (int i = 0; i < (int)size(); i++)
	{
		// 先取得直线段的两个端点
		CPnt pt1 = at(i)->m_ptStart;
		CPnt pt2 = at(i)->m_ptEnd;
		
		// 将两个端点依次变换到世界坐标系下
		CPnt ptWorld1 = trans.GetWorldPoint(pt1);
		CPnt ptWorld2 = trans.GetWorldPoint(pt2);

		vector<CRange> ranges = at(i)->m_Ranges;

		// 生成变换后的直线段
		at(i)->Create(ptWorld1, ptWorld2);
		at(i)->m_Ranges = ranges;
	}

	// 调整边界值
	UpdateCoveringRect();
}

//
//   以给定的点为中心，以指定的范围为半径，截取出一个特征子集。
//
bool CLineFeatureSet::GetSubset(CPnt& ptCenter, float fRange, CLineFeatureSet& Subset)
{
	Subset.Clear();

	// 依次判断所有直线特征是否应放入子集中
	for (int i = 0; i < (int)size(); i++)
	{
		bool bSelect = false;
		CLineFeature* pFeature = at(i);

		float fDist1 = pFeature->m_ptStart.DistanceTo(ptCenter);      // 直线起点到中心点的距离
		float fDist2 = pFeature->m_ptEnd.DistanceTo(ptCenter);        // 直线终点到中心点的距离

		// 如果直线的两个端点中有一个位于圆内，选择它进子集
		if (fDist1 < fRange || fDist2 < fRange)
			bSelect = true;
		else
		{
			float fLambda;
			CPnt ptFoot;

			// 计算直线到中心点的距离，并获得垂足点坐标
			pFeature->DistanceToPoint(false, ptCenter, &fLambda, &ptFoot);
			
			// 如果垂足点在线段以内，并且垂足点也在圆内，选择它进子集
			if (fLambda >= 0 && fLambda <= 1 && ptFoot.DistanceTo(ptCenter) < fRange)
				bSelect = true;
		}

		if (bSelect)
			Subset.Add(*pFeature);
	}

	return true;
}

#define Q_MIN_LINE_LEN          0.3f
#define Q_MIN_INC_ANGLE         TO_RADIAN(45)            // 最小夹角45度
#define Q_MAX_INC_ANGLE         TO_RADIAN(135)           // 最大夹角135度

//
//   针对当前的直线特征集合，分离出“较好的直线特征集合”和“角点特征集合”。
//
bool CLineFeatureSet::SeperateFeatures(CLineFeatureSet& GoodLineFeatureSet, CPointFeatureSet& CornerFeatureSet)
{
	GoodLineFeatureSet.Clear();
	CornerFeatureSet.Clear();

	for (int i = 0;i < (int)size(); i++)
	{
		CLineFeature* p = at(i);

		// 如果该直线段够长，直接将其加入“合格直线特征集合”中
		if (p->Length() > Q_MIN_LINE_LEN)
		{
			GoodLineFeatureSet.Add(*p);
			continue;
		}

		// 如果是短直线
		CPnt pt;
		
		// 跟前一条直线进行比较，看是否能构成角点
		if (i > 0)
		{
			CLineFeature* p1 = at(i - 1);

			// 如果能构成角点，则添加此角点
			if (p1->Length() > Q_MIN_LINE_LEN && p->MakeCorner(*p1, Q_MIN_INC_ANGLE, Q_MAX_INC_ANGLE, 0.03f, pt))
			{
				CPointFeature* pFeature = new CPointFeature;
				pFeature->SetSubType(CORNER_FEATURE);
				pFeature->SetCenterPoint(pt);
				CornerFeatureSet += pFeature;
			}
		}
		else if (i < (int)size() - 1)
		{
			CLineFeature* p1 = at(i + 1);

			// 如果能构成角点，则添加此角点
			if (p1->Length() > Q_MIN_LINE_LEN && p->MakeCorner(*p1, Q_MIN_INC_ANGLE, Q_MAX_INC_ANGLE, 0.03f, pt))
			{
				CPointFeature* pFeature = new CPointFeature;
				pFeature->SetSubType(CORNER_FEATURE);
				pFeature->SetCenterPoint(pt);
				CornerFeatureSet += pFeature;
			}
		}
	}

	return true;
}

//
//   进行坐标正变换。
//
void CLineFeatureSet::Transform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Transform(frame);

	m_pstScanner.Transform(frame);

	UpdateCoveringRect();
}

//
//   进行坐标逆变换。
//
void CLineFeatureSet::InvTransform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->InvTransform(frame);

	m_pstScanner.InvTransform(frame);

	UpdateCoveringRect();
}

#ifdef _MFC_VER

void CLineFeatureSet::Dump()
{
//	TRACE("Dumping Line Scan (%d lines):\n", size());

	for (int i = 0; i < (int)size(); i++)
	{
		DebugTrace(_T("Line #%d:\t%.2f\t%.2f\t\t%.2f\t%.2f\n"), i,
			at(i)->m_ptStart.x, at(i)->m_ptStart.y,
			at(i)->m_ptEnd.x, at(i)->m_ptEnd.y);

	}

	DebugTrace(_T("\n"));
}

//
//   绘制直线特征集合。
//
void CLineFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
	int nPointSize, bool bShowActiveSide, bool bShowId, bool bShowRefPoint, int nShowDisabled)
{
	for (int i = 0; i < (int)size(); i++)
	{
		at(i)->SetIntParam(0, (int)bShowActiveSide);
		at(i)->SetIntParam(1, (int)bShowId);
		at(i)->SetIntParam(2, (int)bShowRefPoint);

		at(i)->Plot(ScrnRef, pDC, crColor, crSelected, nPointSize, nShowDisabled);
	}
}

//
//   判断一个给定点是否触碰到某一条直线特征(用于屏幕上直线触碰判断)。
//   返回值：
//      -1：给定点没有触碰到任何直线特征。
//
int CLineFeatureSet::PointHit(const CPnt& pt, float fDistGate)
{
	// 逐个对所有直线特征进行判断
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i)->PointHit(pt, fDistGate))
			return i;
	}

	return -1;
}

#endif

