#include <stdafx.h>
#include "ndt_fuser/HistoryPoses.h"
#include "ScrnRef.h"

#ifdef _MSC_VER

//
//   ȡ��ָ����ŵ���̬
//
CPosture CHistoryPoses::GetPosture(int nIdx)
{
	CPosture pst;
	pst.x = at(nIdx).translation().transpose()(0);
	pst.y = at(nIdx).translation().transpose()(1);
	pst.fThita = at(nIdx).rotation().eulerAngles(0, 1, 2)(2);

	return pst;
}

//
//   �ж�ָ���ĵ��Ƿ�����ĳ��λ�ˡ�
//   ����ֵ��
//     -1 : û�д������κ�λ��
//    ��������������λ�˵ı��
//
int CHistoryPoses::PointHit(const CPnt& pt, float fDistGate)
{
	CPnt ptPose;
	for (int i = 0; i < (int)size() - 1; i++)
	{
		ptPose.x = at(i).translation().transpose()(0);
		ptPose.y = at(i).translation().transpose()(1);
		if (ptPose.DistanceTo(pt) < fDistGate)
			return i;
	}

	return -1;
}

//
//   ��������λ�����ߺ���̬��
//
void CHistoryPoses::Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected)
{
	// ��ʾλ������
	double x1, y1, x2, y2, theta = 0;
	for (int i = 0; i < (int)size(); i++)
	{
		Eigen::Affine3d& pose1 = at(i);
		x1 = pose1.translation().transpose()(0);
		y1 = pose1.translation().transpose()(1);
		theta = pose1.rotation().eulerAngles(0, 1, 2)(2);
		CPnt pt1(x1, y1);

		// �������һ����֮�⣬��Ҫ�������һ����Ĺ켣������
		if (i != (int)size() - 1)
		{
			Eigen::Affine3d& pose2 = at(i + 1);
			x2 = pose2.translation().transpose()(0);
			y2 = pose2.translation().transpose()(1);
			theta = pose2.rotation().eulerAngles(0, 1, 2)(2);
			CPnt pt2(x2, y2);

			CLine ln(pt1, pt2);
			ln.Draw(ScrnRef, pDc, clrTraj, 2);
		}
	}

	// ��ʾ����λ��
	for (int i = 0; i <(int)size(); i++)
	{
		Eigen::Affine3d& pose1 = at(i);
		x1 = pose1.translation().transpose()(0);
		y1 = pose1.translation().transpose()(1);
		theta = pose1.rotation().eulerAngles(0, 1, 2)(2);
		CPosture pstRobot(x1, y1, theta);

		// �����Ҫ�Ļ�����ʾ����̬
		if (m_bShowPoses)
			pstRobot.Draw(ScrnRef, pDc, clrPoses, 40, 150, 1);

		// �����Ҫ�Ļ�����ʾѡ����̬
		if (m_bShowSelected && i == m_nSelected)
		{
			pstRobot.Draw(ScrnRef, pDc, clrSelected, 40, 150, 2);
		}
	}
}
#endif
