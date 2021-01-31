#ifndef __CLineMatchList
#define __CLineMatchList

// 注意:可存储的线段对数量大于实际参与匹配运算的数量

#define MAX_LINE_MATCH_COUNT      10      // 最多可参与最小二乘匹配的直线段数量

#include <vector>
#include "LineMatchPair.h"
#include "CoorTransEquations.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   "CLineMatchList"类的定义。
class CLineMatchList : public vector<CLineMatchPair>
{
private:
	static float m_fDistErrTab[5];
	static int m_nScoreTab[5];

public:
	CTransform   m_Trans;
	static CCoorTransEquations m_LineEquations;

public:
	CLineMatchList() {}

	// 取得表中匹配对的数量
	int GetCount() {return (int)size();}

	// 为匹配表设置坐标变换关系
	void SetTransform(const CTransform& _trans) { m_Trans = _trans; }

	// 取得坐标变换关系
	CTransform& GetTransform() { return m_Trans; }

	// 将一个匹配对加入到匹配表中
	bool Add(const CLineMatchPair& Pair);

	// 在一个匹配表中查找一个匹配对
	int Search(const CLineMatchPair& Pair);

	// 根据所提供的局部直线段在匹配表中查找其对应的匹配对
	int SearchByLocalLine(const CLine& ln);

	// 根据所提供的世界直线段在匹配表中查找其对应的匹配对
	int SearchByWorldLine(const CLine& ln);

	// 对匹配表中那些非”一对一“匹配的项进行移除
	void Filter(bool bDeleteParallel = false);

	void CreateAllOptions(vector<CLineMatchList>* pLineList);
	int FindBestMatchList(vector<CLineMatchList>* pLineList);
	int ApplicablePairCount();

	// 判断是否匹配表中的所有匹配对都是平行线
	bool AllParallel();

	// 生成关于所有线段匹配对的最小二乘数据
	bool CreateLeastSquareData(CCoorTransEquations* pLsm);

	// 根据匹配表计算两个直线集合之间的坐标变换关系
	bool FindTransform();

	// 计算匹配程度参数
	int EvaluateTransform(CTransform& Transform, float* fAverageErr, float* pMatchingScore);

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
