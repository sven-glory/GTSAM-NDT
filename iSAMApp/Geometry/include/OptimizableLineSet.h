#ifndef __COptimizableLine
#define __COptimizableLine

#define MAX_SIGMA                       30      // max sigma value for a line
#define MIN_LINE_LEN                    500     // 直线段的最小长度
#define MAX_LINE_MERGE_DIST             80      // 两条平行直线的间距小于此值时可认为是一条直线，可以合并
#define MAX_LINE_MERGE_ANGLE            (5*PI/180)    // 两条直线的夹角小于此值时可认为是相互平行
#define MIN_CORNER_ANGLE                (PI/6)   // 形成角点的两条直线的最小夹角
#define MAX_HIDDEN_LINE_LEN             5000     // 如果两直线段的延长线超过此距离，生成的交点不采用

#include "Geometry.h"
#include "vector"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   定义“可优化直线”类。
class COptimizableLine : public CLine
{
public:
	bool m_bUse;
	vector<CPnt> m_ptKeys;

public:
	COptimizableLine(CLine& ln)
	{
		Create(ln.m_ptStart, ln.m_ptEnd);
	}

	COptimizableLine() {}
};

///////////////////////////////////////////////////////////////////////////////
//   定义“可优化直线集合”类。
class COptimizableLineSet
{
public:
	vector<COptimizableLine> m_LineSet;

public:
	COptimizableLineSet() {}
	
	// 尝试向集合中加入一条直线
	bool AddLine(CLine& ln);

	// 根据直线特征集合生成角点集合
	int CreateCornerPoints(vector<CPnt>& ptCorners);

};

#endif
