#pragma once

#include <vector>
#include <Eigen/Eigen>

using namespace std;

#ifdef _MSC_VER
class CDC;
class CScreenReference;
class CPnt;
class CPosture;
#endif

///////////////////////////////////////////////////////////////////////////////
//   历史姿态记录
class CHistoryPoses : public vector<Eigen::Affine3d>
{
private:
	bool m_bShowPoses;
	bool m_bShowSelected;
	int  m_nSelected;

public:
	CHistoryPoses() 
	{
		m_bShowPoses = false;
		m_bShowSelected = true;
		m_nSelected = -1;
	}

	// 选择指定的姿态
	void Select(int nIdx) { m_nSelected = nIdx; }

	// 设置显示选项
	void SetOption(bool bPosesOn, bool bSelectedOn)
	{
		m_bShowPoses = bPosesOn;
		m_bShowSelected = bSelectedOn;
	}

	// 取得指定序号的姿态
	CPosture GetPosture(int nIdx);

#ifdef _MSC_VER
	// 判断指定的点是否触碰到某个位姿
	int PointHit(const CPnt& pt, float fDistGate);

	//   绘制整个位姿曲线和姿态
	void Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrTraj, unsigned long clrPoses, unsigned long clrSelected);
#endif
};
