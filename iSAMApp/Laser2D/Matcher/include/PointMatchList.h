#ifndef __CPointMatchList
#define __CPointMatchList

#define MAX_POINT_MATCH_COUNT     10      // 最多可参与最小二乘匹配的直线段数量

#include <vector>
#include "PointMatchPair.h"
#include "CoorTransEquations.h"

using namespace std;

#define DEFAULT_POINT_MATCH_ERR_GATE        0.3f      // 缺省的点匹配误差门限

//
//   匹配结果评估。
//
struct CMatchEvaluation
{
	int   nMatchCount;             // 匹配数量
	float fMatchRate;              // 匹配成功的点所占比例(0%~100%)
	float fMeanErr;                // 平均匹配误差(单位:m)
	float fScore;                  // 综合得分
};

///////////////////////////////////////////////////////////////////////////////
//   "CPointMatchList"类的定义。
class CPointMatchList : public vector<CPointMatchPair>
{
private:
	static float m_fDistErrTab[5];
	static int   m_nScoreTab[5];
	float        m_fErrGate;          // 点匹配误差门限：大于此门限则认为未匹配上

public:
	CTransform   m_Trans;
	CMatchEvaluation m_Evaluation;

	static CCoorTransEquations m_PointEquations;

public:
	CPointMatchList() { m_fErrGate = DEFAULT_POINT_MATCH_ERR_GATE; }

	// 设置点匹配误差门限
	void SetErrGate(float fErrGate) { m_fErrGate = fErrGate; }

	// 清除“点匹配表”
	void Clear() { clear(); }

	// 取得表中匹配对的数量
	int GetCount() {return (int)size();}

	// 取得指定的匹配对
	CPointMatchPair& GetPair(int nIdx) { return at(nIdx); }

	// 为匹配表设置坐标变换关系
	void SetTransform(const CTransform& _trans) {m_Trans = _trans;}

	// 取得坐标变换关系
	CTransform& GetTransform() { return m_Trans; }

	// 将一个匹配对加入到匹配表中
	bool Add(const CPointMatchPair& Pair);

	// 将一个匹配对按局部点极径从小到大的顺序加入到匹配表中
	bool AddInRadiusOrder(const CPointMatchPair& Pair);

	// 在一个匹配表中查找一个匹配对
	int Search(const CPointMatchPair& Pair) const;

	// 根据所提供的局部点在匹配表中查找其对应的匹配对
	int SearchByLocalPoint(const CPnt& ptLocal) const;

	// 根据所提供的世界点在匹配表中查找其对应的匹配对
	int SearchByWorldPoint(const CPnt& ptWorld) const;

	// 限制实际启用的匹配对的数量
	void LimitMatchPairNum(int nLimit);

	// 对匹配表中那些非”一对一“匹配的项进行移除
	void Filter();

	// 生成关于所有点对的最小二乘数据
	bool CreateLeastSquareData(CLinearEquations* pLsm);

	// 根据匹配表计算两个点云之间的坐标变换关系
	bool FindTransform();

	// 计算两个点云之间的匹配程度参数
	void EvaluateTransform();

	// 计算匹配程度参数
	int EvaluateTransform(CTransform& Transform, float* fAverageErr, float* pMatchingScore);

#ifdef _MFC_VER
	// 查看点匹配表
	void Dump();
#endif
};
#endif
