#include "stdafx.h"
#include <stdlib.h>
#include "MovingObj.h"

#define CYCLE_TIME              0.05f

///////////////////////////////////////////////////////////////////////////////

//
//   在指定的区域内随机取得一点并返回。
//
CPnt CFence::GetRandomPoint()
{
	CPnt pt;
	
	float fRatioX = (rand() % 100) / 100.0f;
	float fRatioY = (rand() % 100) / 100.0f;

	pt.x = fLeft + Width() * fRatioX;
	pt.y = fBottom + Height() * fRatioY;

	return pt;
}

///////////////////////////////////////////////////////////////////////////////

CMovingObj::CMovingObj()
{
	m_fVel = 4.5f*1000;
	m_Circle.m_fRadius = 0.2f*1000;
	m_Fence.Create(4000.0f, 4000.0f, 8000.0f, 10000.0f);
	m_bShown = FALSE;
}

//
//   从文件中装入参数。
//
BOOL CMovingObj::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\t%f\t%f\t%f\n", &m_Fence.fLeft, &m_Fence.fTop, &m_Fence.fRight, &m_Fence.fBottom,
		&m_Circle.m_fRadius, &m_fVel);

	return TRUE;
}

void CMovingObj::Start() 
{
	m_ptStart = m_Fence.GetRandomPoint();
	m_ptTarget = m_ptStart;
	m_nStep = -1;
	m_Circle.m_ptCenter = m_ptStart;
	m_Circle0 = m_Circle;
}

void CMovingObj::SetTargetPoint()
{
	m_ptTarget = m_Fence.GetRandomPoint();
	m_ln.Create(m_ptStart, m_ptTarget);
	m_nStep = 0;
}

// 产生下一个位置点
void CMovingObj::Run()
{
	if (m_nStep < 0)
	{
		m_ptStart = m_ptTarget;
		SetTargetPoint();
	}

	float fProgress = (m_nStep++ * m_fVel) * CYCLE_TIME;
	if (fProgress > m_ln.Length())
	{
		m_nStep = -1;
		fProgress = m_ln.Length();
	}

	m_Circle.m_ptCenter = m_ln.TrajFun(fProgress);
}

//
//   重新启动一次初始绘制过程。
//
void CMovingObj::NewDraw()
{
	m_bShown = FALSE;
}

//
//   在屏幕上绘制此圆。
//
void CMovingObj::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
{
	pDC->SetROP2(R2_NOT);
	
	// 如果是首次绘制，没有擦除原有图像这一操作
	if (!m_bShown)
		m_bShown = TRUE;	
	
	// 非首次绘制，应先擦掉原来的图像
	else	
		m_Circle0.Draw(ScrnRef, pDC, crColor, nWidth);


	// 再在新位置绘制新图像
	m_Circle.Draw(ScrnRef, pDC, crColor, nWidth);
	m_Circle0 = m_Circle;

	pDC->SetROP2(R2_COPYPEN);
}

///////////////////////////////////////////////////////////////////////////////


//
//   从文件中装入参数。
//
BOOL CMovingObjSet::Load(FILE* fp)
{
	if (fscanf(fp, "%d\n", &m_nCount) != 1)
		return FALSE;

	for (int i = 0; i < m_nCount; i++)
		if (!m_Obj[i].Load(fp))
			return FALSE;

	return TRUE;
}

//
//   启动自动移动过程。
//
void CMovingObjSet::Start()
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].Start();
}

//
//   产生下一个位置点。
//
void CMovingObjSet::Run()
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].Run();
}

//
//   重新启动一次初始绘制过程。
//
void CMovingObjSet::NewDraw()
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].NewDraw();
}

//
//   绘制该移动目标。
//
void CMovingObjSet::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].Draw(ScrnRef, pDC, crColor, nWidth);
}
