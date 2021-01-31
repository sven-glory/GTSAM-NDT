#include <stdafx.h>
#include "MultiSegLine.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   Constructor form #1
//
CMultiSegLine::CMultiSegLine(const CPnt& ptStart, const CPnt& ptEnd) : CLine(ptStart, ptEnd)
{
	CreateSingleLine();
}

//
//   Constructor form #2
//
CMultiSegLine::CMultiSegLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen) :
	CLine(ptStart, angSlant, fTotalLen)
{
	CreateSingleLine();
}

//
//   Constructor form #3
//
CMultiSegLine::CMultiSegLine(const CLine& Line2) :
	CLine(Line2)
{
	CreateSingleLine();
}

//
//   生成单个的直线段。
//
void CMultiSegLine::CreateSingleLine()
{
	CRange range(0, Length());

	m_Ranges.clear();
	m_Ranges.push_back(range);
}

//
//   取得指定的子段。
//
bool CMultiSegLine::GetSegment(int i, CLine& Line) const
{
	// 段号合法性检查
	if (i >= (int)m_Ranges.size())
		return false;

	float fFrom = m_Ranges[i].fFrom;
	float fTo = m_Ranges[i].fTo;

	CPnt pt1 = TrajFun(fFrom);
	CPnt pt2 = TrajFun(fTo);

	Line.Create(pt1, pt2);

	return true;
}

//
//   根据两点生成线段。
//
void CMultiSegLine::Create(const CPnt& ptStart, const CPnt& ptEnd)
{
	CLine::Create(ptStart, ptEnd);
	CreateSingleLine();
}

//
//   根据一点和直线的倾斜角生成线段。
//
void CMultiSegLine::Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen)
{
	CLine::Create(ptStart, angSlant, fTotalLen);
	CreateSingleLine();
}

//
//   从多条线段生成此多段线。
//   说明：
//       此函数假定给定的多条直线段是大体平行的。在此情况下，程序将对这些直线段
//   进行离散采样取点，并拟合出一条直线。最后，程序将取得所有
//
void CMultiSegLine::Create(CLine* pLines, int nNum)
{
	m_Ranges.clear();

	if (nNum == 1)
	{
		Create(pLines[0].m_ptStart, pLines[0].m_ptEnd);
		return;
	}

	// 先计算总长度
	float fTotalLen = 0;
	for (int i = 0; i < nNum; i++)
		fTotalLen += pLines[i].Length();

	// 估算采样点数量(5cm间隔)
	float fUnitLen = 0.05f;
	int nCountPoints = (int)(fTotalLen / fUnitLen);
	nCountPoints += nNum * 2;                  // 考虑到端头情况，多留出一些空间

	CPnt* pPoints = new CPnt[nCountPoints];

	// 通过离散采样，求出共线方程
	nCountPoints = 0;
	for (int i = 0; i < nNum; i++)
	{
		CLine& Line = pLines[i];
		float fLen = Line.Length();

		// 计算采样点坐标
		for (float fCurLen = 0; fCurLen < fLen; fCurLen += fUnitLen)
			pPoints[nCountPoints++].GetPntObject() = Line.TrajFun(fCurLen);
	}

	// 线性回归，求出直线方程
	CLineBase lineBase;
	bool bOk = lineBase.CreateFitLine(pPoints, nCountPoints);
	delete[]pPoints;

	if (!bOk)
		return;

	CLine* pProjectLine = new CLine[nNum];

	// 计算各直线段到此拟合线的投影
	for (int i = 0; i < nNum; i++)
	{
		pProjectLine[i] = lineBase.GetProjectLine(pLines[i]);

		// 将起点调整为位于左|下方向
		if (pProjectLine[i].m_ptStart > pProjectLine[i].m_ptEnd)
			pProjectLine[i].Reverse();

	}

	// 下面求最“小”的起点和最“大”的终点
	int nMinStartId = 0;
	int nMaxEndId = 0;

	for (int i = 1; i < nNum; i++)
	{
		if (pProjectLine[i].m_ptStart < pProjectLine[nMinStartId].m_ptStart)
			nMinStartId = i;

		if (pProjectLine[i].m_ptEnd > pProjectLine[nMaxEndId].m_ptEnd)
			nMaxEndId = i;
	}

	// 得到总体线段的起点、终点
	CPnt ptStart = pProjectLine[nMinStartId].m_ptStart;
	CPnt ptEnd = pProjectLine[nMaxEndId].m_ptEnd;

	// 现在生成总体线段
	Create(ptStart, ptEnd);
	m_Ranges.clear();

	// 下面生成各分段
	for (int i = 0; i < nNum; i++)
	{
		float fFrom = pProjectLine[i].m_ptStart.DistanceTo(ptStart);
		float fTo = pProjectLine[i].m_ptEnd.DistanceTo(ptStart);
		
		if (fTo < fFrom)
		{
			float fTemp = fFrom;
			fFrom = fTo;
			fTo = fTemp;
		}

		CRange range(fFrom, fTo);
		m_Ranges.push_back(range);
	}

	m_Ranges.Normalize();

	delete []pProjectLine;
}

//
//   从多段线生成此多段线。
//
void CMultiSegLine::Create(const CMultiSegLine& MultiSegLine)
{
	*this = MultiSegLine;
}

//
//   判断一个给定的点是否“触碰”到该多段直线。
//
bool CMultiSegLine::PointHit(const CPnt& pt, float fDistGate)
{
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line;
		GetSegment(i, Line);
		
		if (Line.PointHit(pt, fDistGate))
			return true;
	}

	return false;
}

#ifdef _MSC_VER

//
//   在屏幕上绘制此直线。
//
void CMultiSegLine::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bBigVertex)
{
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line;
		GetSegment(i, Line);

		Line.Draw(ScrnRef, pDC, crColor, nWidth, nPointSize, bBigVertex);
	}
}
#endif
