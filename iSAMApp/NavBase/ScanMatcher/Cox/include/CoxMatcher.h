#ifndef SL_COX_H
#define SL_COX_H

#include "CoxInfo.h"
#include "matrix.h"

struct CoxEquation
{
	float s11, s12, s13, s22, s23, s33;
	float s2;
};

//
//   基于Cox算法的匹配器。
//
class CCoxMatcher
{
private:
	CScan*         m_pRefScan;              // 指向“参考扫描”的指针
	CScan*         m_pCurScan;              // 指向“当前扫描”的指针
	CScan*         m_pReducedScan;          // 经过缩减滤波的“当前扫描”指针
	CScan*         m_pWorkScan;             // 用于进行旋转、平移并迭代的工作扫描
	CCoxInfoArray* m_pCoxInfoArray;         // 各点的对应关系

	struct CoxEquation m_ce;           /* equation system matrix and vector */

private:
	// 释放所有动态分配的内存
	void Cleanup();

	// 进行“点-线”匹配
	int MatchPointsToLines(int iteration, int *pnum, float *ddx, float *ddy, float *dda,
		float *error,	bool *allDofFixed);

	// 试图对m_pWorkScan中的每个扫描点找到一条与之对应的直线段
	short AssignNearestLines();

	// 配置Cox匹配对象
	BOOL Setup(CScan* pRefScan, CScan* pCurScan);

	// 采用迭代方法进行“点-线”匹配
	int ScanMatchHelp(PoseGauss *match, float *error);

public:
	// 构造函数
	CCoxMatcher();

	// 析构函数
	~CCoxMatcher();

	// 进行匹配运算
	int Match(CScan* pRefScan, CScan* pCurScan, PoseGauss* pPose, float* pError);
};
#endif
