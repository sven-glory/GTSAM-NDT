#ifndef __CPointMatchListSet
#define __CPointMatchListSet

#include <vector>
#include "PointMatchList.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   定义“点匹配表集合”类。
class CPointMatchListSet : private vector<CPointMatchList>
{
public:
	CPointMatchListSet() {}

	// 清零一个匹配表集合
	void Clear() { clear(); }

	// 取得一个匹配表集合中所含匹配表的数量
	int GetCount() { return (int)size(); }

	// 从一个匹配表集合中取得一个匹配表
	CPointMatchList& GetList(int i) { return at(i); }

	// 向一个匹配表集合中加入一个新的匹配表
	bool Add(const CPointMatchList& lst);

	// 在匹配表集合中查找一个匹配表，使它包含指定的两个匹配对
	int Search(const CPointMatchPair& Pair1, const CPointMatchPair& Pair2) const;

	// 寻找最优匹配表
	int FindBestMatch();
};
#endif
