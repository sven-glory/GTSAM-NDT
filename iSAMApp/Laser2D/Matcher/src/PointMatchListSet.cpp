#include "stdafx.h"
#include "PointMatchListSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   向一个匹配表集合中加入一个新的匹配表。
//   返回值：
//      true  - 操作成功
//      false - 操作失败
//
bool CPointMatchListSet::Add(const CPointMatchList& lst)
{
	push_back(lst);
	return true;
}

//
//   在一个匹配表集合中查找一个匹配表，使它包含指定的两个匹配对。
//   返回值：
//       >= 0: 成功找到的匹配表的序号
//       < 0 : 未找到
//
int CPointMatchListSet::Search(const CPointMatchPair& Pair1, const CPointMatchPair& Pair2) const
{
	for (int i = 0; i < size(); i++)
		if (at(i).Search(Pair1) >= 0 && at(i).Search(Pair2) >= 0)
			return i;

	return -1;
}

//
//   寻找最优匹配表。
//
int CPointMatchListSet::FindBestMatch()
{
	int nBest = -1;      // 当前最优匹配表的序号
	float fMaxScore = 0;

	for (int i = 0; i < size(); i++)
	{
		// 取当前匹配表的得分
		float fScore = at(i).m_Evaluation.fScore;

		// 如当前当前匹配表的点对数大于记录的"最大数量"
		if (fScore > fMaxScore)
		{
			fMaxScore = fScore;        // 记录新的最数量
			nBest = i;                 // 记录当前最优表的序号
		}
	}

	// 如果没找到
	if (nBest < 0)
		return -1;

	// 最优表的点对数量小于3，视为不成功
	else if (at(nBest).m_Evaluation.nMatchCount < 3)
		return -1;
	else
		return nBest;
}
