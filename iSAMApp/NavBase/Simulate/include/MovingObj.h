#ifndef __CMoveObj
#define __CMoveObj

#include "Geometry.h"
#include "ScrnRef.h"

#define CYCLE_TIME              0.05f
#define MAX_MOVING_OBJ_NUM      50

///////////////////////////////////////////////////////////////////////////////

//
//   定义“围栏区域”。
//
class CFence : public CRegion
{
public:
	// 在指定的区域内随机取得一点并返回
	CPnt GetRandomPoint();
};

///////////////////////////////////////////////////////////////////////////////
//
//   定义“移动物体”。
//
class CMovingObj
{
private:
	CFence   m_Fence;                    // 围栏区域
	CPnt m_ptStart;                  // 当前运动路段的起始点
	CPnt m_ptTarget;                 // 当前运动路段的目标点
	CLine    m_ln;                       // 当前运动直线段
	int      m_nStep;                    // 当前步数
	float    m_fVel;                     // 运动速度
	BOOL     m_bShown;                   // 是否图像已绘制过

public:
	CCircle  m_Circle;                   // 当前图像
	CCircle  m_Circle0;                  // 上一次绘制的图像

private:
	// 设置运动的目标点
	void SetTargetPoint();

public:
	CMovingObj();

	// 从文件中装入参数
	BOOL Load(FILE* fp);

	// 启动自动移动过程
	void Start();

	// 产生下一个位置点
	void Run();

	// 重新启动一次初始绘制过程
	void NewDraw();

	// 绘制该移动目标
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
};

///////////////////////////////////////////////////////////////////////////////
//
//   定义“移动物体集合”。
//
class CMovingObjSet
{
public:
	CMovingObj m_Obj[MAX_MOVING_OBJ_NUM];
	int        m_nCount;

public:
	CMovingObjSet() {}

	// 从文件中装入参数
	BOOL Load(FILE* fp);

	// 启动自动移动过程
	void Start();

	// 产生下一个位置点
	void Run();

	// 重新启动一次初始绘制过程
	void NewDraw();

	// 绘制该移动目标
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
};

#endif
