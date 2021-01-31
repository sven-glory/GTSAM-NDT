#include "stdafx.h"
#include <math.h>
#include "LineMatchList.h"
#include "LineFeature.h"
#include "DebugTrace.h"
#include "Combination.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define MAX_LINE_MATCH_ERROR              0.5f

// 初始化匹配打分表
float CLineMatchList::m_fDistErrTab[5] = { 0.1f, 0.3f, 0.5f, 0.8f, 1.2f };
int CLineMatchList::m_nScoreTab[5] = { 10, 8, 6, 4, 2 };

// 用于直线匹配表的变换方程
CCoorTransEquations CLineMatchList::m_LineEquations((MAX_LINE_MATCH_COUNT+1) * 3, 4);

///////////////////////////////////////////////////////////////////////////////
// for CLineMatchList member function

//
//   向表中增加一个线段匹配对。
//
bool CLineMatchList::Add(const CLineMatchPair& Pair)
{
	push_back(Pair);
	return true;
}

//
//   在一个匹配表中查找一个匹配对。
//
//   返回值：
//       如找到，则返回所在位置的索引号，否则返回-1。
//
int CLineMatchList::Search(const CLineMatchPair& Pair)
{
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i).m_lnLocal.m_nId == Pair.m_lnLocal.m_nId &&
			at(i).m_lnWorld.m_nId == Pair.m_lnWorld.m_nId)
			return i;
	}

	return -1;
}

//
//   根据所提供的局部直线段在匹配表中查找其对应的匹配对。
//
//   返回值：
//       如找到，则返回所在位置的索引号，否则返回-1。
//
int CLineMatchList::SearchByLocalLine(const CLine& ln)
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_lnLocal.m_nId == ln.m_nId)
			return i;

	return -1;
}

//
//   根据所提供的世界直线段在匹配表中查找其对应的匹配对。
//
//   返回值：
//       如找到，则返回所在位置的索引号，否则返回-1。
//
int CLineMatchList::SearchByWorldLine(const CLine& ln)
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_lnWorld.m_nId == ln.m_nId)
			return i;

	return -1;
}

//
//   对匹配表中那些非”一对一“匹配的项进行移除。
//
void CLineMatchList::Filter(bool bDeleteParallel)
{
	int nCount = size();
	bool* pRemove = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		pRemove[i] = false;

	for (int i = 0; i < nCount - 1; i++)
	{
		// 跳过那些已标记为“删除”的项
		if (pRemove[i])
			continue;

		CLineMatchPair& Pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			if (pRemove[j])
				continue;

			CLineMatchPair& Pair = at(j);

			// 标记多重匹配项
			if (Pair1.m_lnLocal.m_nId == Pair.m_lnLocal.m_nId)
			{
				// 如果这两个线段共线，则删除后者
				if (Pair.m_lnLocal.IsColinearWith(Pair1.m_lnLocal, 5*PI/180, 0.080f) || 
					 (bDeleteParallel && Pair.m_lnLocal.IsParallelTo(Pair1.m_lnLocal)))
				{
					pRemove[j] = true;
				}

#if 0
				// 否则，两个都删除
				else
				{
					pRemove[i] = true;
					pRemove[j] = true;
				}
#endif
			}
			else if (Pair1.m_lnWorld.m_nId == Pair.m_lnWorld.m_nId)
			{
				// 如果这两个线段共线，则删除其中之一
				if (Pair.m_lnWorld.IsColinearWith(Pair1.m_lnWorld, 5*PI/180, 0.080f) ||
					(bDeleteParallel && Pair.m_lnWorld.IsParallelTo(Pair1.m_lnWorld)))
				{
					pRemove[j] = true;
				}
#if 0
				// 否则，两个都删除
				else
				{
					pRemove[i] = true;
					pRemove[j] = true;
				}
#endif
			}
		}
	}

	for (int i = 0; i < nCount - 1; i++)
	{
		// 跳过那些已标记为“删除”的项
		if (pRemove[i])
			continue;

		CLineMatchPair& Pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			if (pRemove[j])
				continue;

			CLineMatchPair& Pair = at(j);

			if (Pair.m_lnWorld.IsColinearWith(Pair1.m_lnWorld, 5 * PI / 180, 0.080f) ||
				(bDeleteParallel && Pair.m_lnWorld.IsParallelTo(Pair1.m_lnWorld)))
			{
				pRemove[j] = true;
			}
		}
	}

	// 将那些标记为”移除“的项进行移除
	for (int i = nCount - 1; i >= 0; i--)
	{
		if (pRemove[i])
			erase(begin() + i);
	}

	delete[]pRemove;
}

//
//   根据当前的(可能含有“一对多”匹配的)直线匹配表，生成所有可能的“一对一”匹配方案集合。
//
void CLineMatchList::CreateAllOptions(vector<CLineMatchList>* pLineList)
{
	int nCount = size();
	int* pNext = new int[nCount];
	for (int i = 0; i < nCount; i++)
		pNext[i] = -1;

	// 将local线段相同的项加以标记，以便一会儿提取
	for (int i = 0; i < nCount - 1; i++)
	{
		CLineMatchPair& Pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			CLineMatchPair& Pair = at(j);

			// 标记多重匹配项
			if (Pair1.m_lnLocal.m_nId == Pair.m_lnLocal.m_nId)
			{
				pNext[i] = j;
			}
#if 0
			else if (Pair1.m_lnWorld.m_nId == Pair.m_lnWorld.m_nId)
			{
			}
#endif
		}
	}

	// 准备生成所有的“一对一”匹配方案的集合
	vector<vector<CLineMatchPair>> vvPairs;

	// 先找到所有具有多重匹配关系的“匹配对”，并将这些“匹配对”加入vvPair数组中
	int j = 0;
	for (int i = 0; i < nCount; i++)
	{
		// 如果此多重匹配项已被处理过，跳过它
		if (pNext[i] == -2)
			continue;

		// 针对尚未处理过的多重匹配对
		else if (pNext[i] >= 0)
		{
			vector<CLineMatchPair> vPairs;

			vPairs.push_back(at(i));
			int nNext = pNext[i];

			pNext[i] = -2;

			// 将与该Local直线相同的所有匹配对收集到vPairs向量中
			while (nNext >= 0)
			{
				vPairs.push_back(at(nNext));
				int nNext = pNext[nNext];
				pNext[nNext] = -2;
			}

			// 将所有具有多重匹配的匹配对集合都加入到vvPairs向量中
			vvPairs.push_back(vPairs);
		}
	}

	CLineMatchList MinList = *this;

	// 将那些标记为”移除“的项进行移除
	for (int i = MinList.size() - 1; i >= 0; i--)
	{
		if (pNext[i] == -2)
			MinList.erase(MinList.begin() + i);
	}

	delete[]pNext;


	// 下面准备生成组合数
	int nvvCount = vvPairs.size();
	if (nvvCount > 0)
	{
		int* pSize = new int[nvvCount];
		int* pOutput = new int[nvvCount];

		CCombination Combination;
		Combination.Create(nvvCount, pSize);

		// 根据组合数生成所有“直线匹配集合”的集合
		bool bDone = false;
		while (!bDone)
		{
			bDone = !Combination.Generate(pOutput);
			if (!bDone)
			{
				CLineMatchList newList = MinList;
				for (int i = 0; i < nvvCount; i++)
					newList.push_back(vvPairs[i][pOutput[i]]);

				pLineList->push_back(newList);
			}
		}
		delete[]pSize;
		delete[]pOutput;
	}
	else
	{
		pLineList->push_back(MinList);
	}
}

int CLineMatchList::ApplicablePairCount()
{
	// 判断是否有足够进行定位的直线匹配对
	int nCount = 0;
	for (int i = 0; i < GetCount() - 1; i++)
	{
		CLine& Line1 = at(i).m_lnLocal;
		for (int j = i + 1; j < GetCount(); j++)
		{
			CLine& Line2 = at(j).m_lnLocal;
			CAngle angDiff = Line1.AngleToUndirectionalLine(Line2);

			// 如果两直线夹角大于30度，则可用于定位
			if (angDiff > CAngle(30.0f, IN_DEGREE))
				nCount++;
		}
	}

	// 返回可用的匹配直线对数量
	return nCount;
}

int CLineMatchList::FindBestMatchList(vector<CLineMatchList>* pLineList)
{
	int nBest = -1;
	float fMaxScore = 0;

	CCoorTransEquations lsm((MAX_LINE_MATCH_COUNT + 1)* 3, 4);

	for (int i = 0; i < (int)pLineList->size(); i++)
	{
		lsm.Start();

		CLineMatchList& LineMatchList = pLineList->at(i);
		if (LineMatchList.ApplicablePairCount() < 2)
			continue;

		LineMatchList.CreateLeastSquareData(&lsm);

		CTransform trans;
		float f[4];
		if (lsm.LeastSquareSolve(f, 4))
		{
			float sin = f[0];
			float cos = f[1];
			float angle = (float)atan2(sin, cos);

			float x = f[2];
			float y = f[3];
			trans.Create(x, y, angle);

			// 在此需要对求解的结果进行判断，如果误差太大则认为求解失败
			float fScore2 = 0, fAveErr1 = 0, fAveErr2 = 0;
			int nCountLineMatched = LineMatchList.EvaluateTransform(trans, &fAveErr2, &fScore2);

			if (fScore2 > fMaxScore)
			{
				fMaxScore = fScore2;
				nBest = i;
			}
		}
	}

	return nBest;
}

//
//   判断是否匹配表中的所有匹配对都是平行线。
//
bool CLineMatchList::AllParallel()
{
	// 如果仅有一个直线匹配对，则返回true
	if (size() == 1)
		return true;

	// 取得第一条直线
	CLineMatchPair& Pair1 = at(0);
	CLine& lnWorld1 =at(0).m_lnWorld;

	// 将后续直线依次与第一条进行对比，看是否与之平行
	for (int i = 1; i < (int)size(); i++)
	{
		CLineMatchPair& Pair = at(i);

		// 如果不平行，直接返回false
		if (!Pair.m_lnWorld.IsParallelTo(lnWorld1))
			return false;
	}

	return true;
}

//
//   生成关于所有直线段对的最小二乘数据。
//
bool CLineMatchList::CreateLeastSquareData(CCoorTransEquations* pLsm)
{
	float a[3][4], b[3];

	int nMaxCount = size();
	if (nMaxCount > MAX_LINE_MATCH_COUNT)
		nMaxCount = MAX_LINE_MATCH_COUNT;

	for (int i = 0; i < nMaxCount; i++)
	{
		// 根据直线段对生成三行矩阵数据
		at(i).MakeLeastSquareData(a, b);
	
		// 将这三行数据加入最小二乘求解器中
		for (int j = 0; j < 3; j++)
			if (!pLsm->AddRow(a[j], b[j]))
				return false;
	}

	return true;
}

//
//   根据匹配表计算两个直线集合之间的坐标变换关系。
//
bool CLineMatchList::FindTransform()
{
	// 重新设置矩阵数据
	m_LineEquations.Start();

	if (!CreateLeastSquareData(&m_LineEquations))
		return false;
	
	m_LineEquations.Dump();

	float f[4];
	if (m_LineEquations.LeastSquareSolve(f, 4))
	{
		float sin = f[0];
		float cos = f[1];
		float angle = (float)atan2(sin, cos);

		float x = f[2];
		float y = f[3];
		m_Trans.Create(x, y, angle);

		// 在此需要对求解的结果进行判断，如果误差太大则认为求解失败
		float fScore2 = 0, fAveErr1 = 0, fAveErr2 = 0;
		int nCountLineMatched = EvaluateTransform(m_Trans, &fAveErr2, &fScore2);
#if 0
		if (nCountLineMatched >= 2)
			return true;
		else
			return false;
#endif

		return true;
	}
}

//
//   计算匹配程度参数。
//
int CLineMatchList::EvaluateTransform(CTransform& Transform, float* pAveErr, float* pScore)
{
	if (size() == 0)
		return 0;

	// 统计匹配线对之间的平均综合距离
	int nCountMatched = 0;
	float fTotalDist = 0;
	int nTotalScore = 0;

	for (int i = 0; i < (int)size(); i++)
	{
		CLineMatchPair& Pair = at(i);

		// 计算本地直线段变换到世界坐标系中的位置
		CLine ln1 = Transform.GetWorldLine(Pair.m_lnLocal);

		// 计算它到匹配线段的综合距离
		CLineFeature Feature1, Feature2;
		Feature1.Create(ln1);
		Feature2.Create(Pair.m_lnWorld);
		float fDist = Feature1.TransformCostTo(Feature2);

		if (fDist < MAX_LINE_MATCH_ERROR)
			nCountMatched++;

		fTotalDist += fDist;

		// 根据距离误差打分
		int nScore = 0;
		for (int j = 0; j < 5; j++)
		{
			// 查表定分
			if (fDist <= m_fDistErrTab[j])
			{
				nScore = m_nScoreTab[j];
				break;
			}
		}
		nTotalScore += nScore;
	}

	// 计算平均距离误差
	float fAveDist = fTotalDist / size();

	// 返回平均误差
	if (pAveErr != NULL)
		*pAveErr = fAveDist;

	// 返回总得分
	if (pScore != NULL)
		*pScore = nTotalScore;

	// 返回匹配数量
	return nCountMatched;
}

#ifdef _MSC_VER

void CLineMatchList::Dump()
{
	for (int i = 0; i < (int)size(); i++)
	{
		CLineMatchPair& Pair = at(i);
		
		CLine& lnLocal = Pair.m_lnLocal;
		CLine& lnWorld = Pair.m_lnWorld;

//		DebugTrace(_T("Local:[%d, %.3f, %.3f, %.3f] (%.2f, %.2f)-(%.2f, %.2f) --------  World: [%d, %.3f, %.3f, %.3f](%.2f, %.2f)-(%.2f, %.2f)\n"), 
		DebugTrace(_T("[\t%d\t%.3f\t%.3f\t%.3f\t]\t%.2f\t%.2f\t%.2f\t%.2f\t\t[\t%d\t%.3f\t%.3f\t%.3f\t]\t%.2f\t%.2f\t%.2f\t%.2f\n"), 
			lnLocal.m_nId, lnLocal.a, lnLocal.b, lnLocal.c,
			lnLocal.m_ptStart.x, lnLocal.m_ptStart.y, lnLocal.m_ptEnd.x, lnLocal.m_ptEnd.y,

			lnWorld.m_nId, lnWorld.a, lnWorld.b, lnWorld.c,
			lnWorld.m_ptStart.x, lnWorld.m_ptStart.y, lnWorld.m_ptEnd.x, lnWorld.m_ptEnd.y);
	}
}
#endif
