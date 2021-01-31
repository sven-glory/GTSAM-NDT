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
//   ��ʷ��̬��¼
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

	// ѡ��ָ������̬
	void Select(int nIdx) { m_nSelected = nIdx; }

	// ������ʾѡ��
	void SetOption(bool bPosesOn, bool bSelectedOn)
	{
		m_bShowPoses = bPosesOn;
		m_bShowSelected = bSelectedOn;
	}

	// ȡ��ָ����ŵ���̬
	CPosture GetPosture(int nIdx);

#ifdef _MSC_VER
	// �ж�ָ���ĵ��Ƿ�����ĳ��λ��
	int PointHit(const CPnt& pt, float fDistGate);

	//   ��������λ�����ߺ���̬
	void Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrTraj, unsigned long clrPoses, unsigned long clrSelected);
#endif
};
