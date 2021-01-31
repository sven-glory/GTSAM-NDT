#include "stdafx.h"
#include <math.h>
#include "PointMatchList.h"
#include "CMatrix.h"
#include "CoorTransEquations.h"
#include "DebugTrace.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// 初始化匹配打分表
float CPointMatchList::m_fDistErrTab[5] = { 0.05f,  0.2f,   0.3f,  0.5f,  0.8f };
int CPointMatchList::m_nScoreTab[5]     = {    10,     8,      6,     4,     2 };

CCoorTransEquations CPointMatchList::m_PointEquations((MAX_POINT_MATCH_COUNT+1) * 2, 4);

//////////////////////////////////////////////////////////////////////////////

//
//   向表中增加一个点匹配对。
//
bool CPointMatchList::Add(const CPointMatchPair& Pair)
{
	push_back(Pair);
	return true;
}

//
//    将一个匹配对按局部点极径从小到大的顺序加入到匹配表中。
//
bool CPointMatchList::AddInRadiusOrder(const CPointMatchPair& Pair)
{
	// 先找到本项应在的插入点i
	bool bFound = false;
	int i;
	for (i = 0; i < (int)size(); i++)
	{
		if (Pair.m_ptLocal.r < at(i).m_ptLocal.r)
		{
			bFound = true;
			break;
		}
	}

	// 如果未找到插入点，说明此极径最大，应加入到队尾
	if (!bFound)
		push_back(Pair);
	else
	{
		// 在位置i处插入新项
		insert(begin() + i, Pair);
	}

	return true;
}

//
//   在一个匹配表中查找一个匹配对。
//
//   返回值：
//       如找到，则返回所在位置的索引号，否则返回-1。
//
int CPointMatchList::Search(const CPointMatchPair& Pair) const
{
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i).m_ptLocal.id == Pair.m_ptLocal.id && at(i).m_ptWorld.id == Pair.m_ptWorld.id)
			return i;
	}

	return -1;
}

//
//   根据所提供的局部点在匹配表中查找其对应的匹配对。
//
//   返回值：
//       如找到，则返回所在位置的索引号，否则返回-1。
//
int CPointMatchList::SearchByLocalPoint(const CPnt& ptLocal) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_ptLocal.id == ptLocal.id)
			return i;

	return -1;
}

//
//   根据所提供的世界点在匹配表中查找其对应的匹配对。
//
//   返回值：
//       如找到，则返回所在位置的索引号，否则返回-1。
//
int CPointMatchList::SearchByWorldPoint(const CPnt& ptWorld) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_ptWorld.id == ptWorld.id)
			return i;

	return -1;
}

//
//   限制实际启用的匹配对的数量。
//
void CPointMatchList::LimitMatchPairNum(int nLimit)
{
	if ((nLimit > 0) && (size() > nLimit))
		erase(begin() + nLimit, end());
}

//
//   对匹配表中那些非”一对一“匹配的项进行移除。
//
void CPointMatchList::Filter()
{
	int nCount = size();

	// 为“删除”标记分配临时空间
	bool* pRemove = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		pRemove[i] = false;

	for (int i = 0; i < nCount - 1; i++)
	{
		CPointMatchPair& pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			CPointMatchPair& pair2 = at(j);

			// 标记多重匹配项
			if (pair1.m_ptLocal.id == pair2.m_ptLocal.id ||
				 pair1.m_ptWorld.id == pair2.m_ptWorld.id)
			{
				pRemove[i] = true;
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

	// 释放“删除”标记
	delete []pRemove;
}

//
//   生成关于所有点对的最小二乘数据。
//
bool CPointMatchList::CreateLeastSquareData(CLinearEquations* pLsm)
{
	float a[2][4], b[2];

	int nMaxCount = size();
	if (nMaxCount > MAX_POINT_MATCH_COUNT)
		nMaxCount = MAX_POINT_MATCH_COUNT;

	for (int i = 0; i < nMaxCount; i++)
	{
		// 根据点对生成两行矩阵数据
		at(i).MakeLeastSquareData(a, b);
	
		// 将这两行数据加入最小二乘求解器中
		if (!pLsm->AddRow(a[0], b[0]))
			return false;

		if (!pLsm->AddRow(a[1], b[1]))
			return false;
	}

	return true;
}

//
//   根据匹配表计算两个点云之间的坐标变换关系。
//
bool CPointMatchList::FindTransform()
{
	// 重新设置矩阵数据
	m_PointEquations.Start();

	float a[2][4], b[2];
	for (int i = 0; i <(int)size(); i++)
	{
		// 根据点对生成两行矩阵数据
		at(i).MakeLeastSquareData(a, b);
	
		// 将这两行数据加入最小二乘求解器中
		if (!m_PointEquations.AddRow(a[0], b[0]))
			return false;

		if (!m_PointEquations.AddRow(a[1], b[1]))
			return false;
	}

	// 求解最小二乘法
	float f[4];
	if (m_PointEquations.LeastSquareSolve(f, 4))
	{
		float sin = f[0];
		float cos = f[1];
		float angle = (float)atan2(sin, cos);

		float x = f[2];
		float y = f[3];
		m_Trans.Create(x, y, angle);

		// 计算评估结果
		EvaluateTransform();
		return true;
	}
	else
		return false;
}

//
//   计算两个点云之间的匹配程度参数。
//
void CPointMatchList::EvaluateTransform()
{
	int nCount = size();
	if (nCount == 0)
		return;

	// 统计匹配点对之间的平均距离
	int nCountMatched = 0;
	int nTotalScore = 0;
	float fTotalDist = 0;

	for (int i = 0; i < nCount; i++)
	{
		CPnt& pnt1 = at(i).m_ptLocal;
		CPnt& pnt2 = at(i).m_ptWorld;
		at(i).m_ptLocalToWorld = m_Trans.GetWorldPoint(pnt1);

		// 计算该点的匹配距离误差
		float fDist = at(i).m_ptLocalToWorld.DistanceTo(pnt2);

		// 如果该点的误差小于规定的门限
		if (fDist < m_fErrGate)
		{
			nCountMatched++;
			fTotalDist += fDist;
		}

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

	// 统计评估结果
	m_Evaluation.fMeanErr = fTotalDist / nCount;
	m_Evaluation.nMatchCount = nCountMatched;
	m_Evaluation.fMatchRate = (float)nCountMatched / (float)nCount;
	m_Evaluation.fScore = (float)nTotalScore;
}

//
//   计算两个点集之间的匹配程度参数。
//
//   返回值：匹配成功的点的数量。
//
int CPointMatchList::EvaluateTransform(CTransform& Transform, float* pAveErr, float* pScore)
{
	int nCount = size();
	if (nCount == 0)
		return 0;

	// 统计匹配点对之间的平均距离
	int nCountMatched = 0;
	int nTotalScore = 0;
	float fTotalDist = 0;

	for (int i = 0; i < nCount; i++)
	{
		CPnt& pnt1 = at(i).m_ptLocal;
		CPnt& pnt2 = at(i).m_ptWorld;
		at(i).m_ptLocalToWorld = Transform.GetWorldPoint(pnt1);

		// 计算该点的匹配距离误差
		float fDist = at(i).m_ptLocalToWorld.DistanceTo(pnt2);

		// 如果该点的误差小于规定的门限
		if (fDist < m_fErrGate)
		{
			nCountMatched++;
			fTotalDist += fDist;
		}

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
	float fAveDist = fTotalDist / nCount;

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

//
//   查看点匹配表。
//
void CPointMatchList::Dump()
{
	for (int i = 0; i < (int)size(); i++)
	{
		CPointMatchPair& Pair = at(i);
		
		CPnt& ptLocal = Pair.m_ptLocal;
		CPnt& ptWorld = Pair.m_ptWorld;

		DebugTrace(_T("Local: (%.2f, %.2f) --------  World: (%.2f, %.2f)\n"), 
			ptLocal.x, ptLocal.y, ptWorld.x, ptWorld.y);
	}
}
#endif
