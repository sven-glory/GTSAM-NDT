//                           - SCRNREF.H -
//
//   ���塰�ӿ�Ӱ�䡱�ࡣ
//
//

#pragma once

#include "Geometry.h"
#include "misc.h"

//////////////////////////////////////////////////////////////////////////////
//   ���塰�ӿ�Ӱ�䡱�ࡣ
class DllExport CScreenReference
{
public:
	// �ӿڶ˱���
	USHORT   m_uWidth;               // ʵ���ӿڵĿ��
	USHORT   m_uHeight;              // ʵ���ӿڵĸ߶�
	int      m_nLeft;                // ʵ���ӿ�������Ӧ�������ӿڵ�X����
	int      m_nTop;                 // ʵ���ӿ����ϲ��Ӧ�������ӿڵ�Y����

	int      m_nVPortWidth;          // �����ӿڵĿ��
	int      m_nVPortHeight;         // �����ӿڵĿ��

	// ����˱���
	CPnt m_ptCenter;             // Ӱ�䵽�ӿ����Ĵ�������������
	CPnt m_ptWorldLeftTop;       // �����������Ͻǵ�����
	CPnt m_ptWorldRightBottom;   // �����������½ǵ�����
	float    m_fRatio;               // �����絽�ӿڶ˵���ʾ������

public:
	CScreenReference(USHORT uWidth, USHORT uHeight, float fRatio, const CPnt& ptCenter);
	CScreenReference();

	// �����ӿڵ����Ľǵ�����Ӧ�������λ��
	void SetCenterPoint(const CPnt& pt);

	// �����ӿڵ����Ͻǵ�����Ӧ�������λ��
	void SetLeftTopPoint(const CPnt& pt);

	// ���������������Ͻǵ�����
	void SetWorldLeftTop(const CPnt& ptWorldLeftTop) { m_ptWorldLeftTop = ptWorldLeftTop; }

	// ���������������½ǵ�����
	void SetWorldRightBottom(const CPnt& ptWorldRightBottom) { m_ptWorldRightBottom = ptWorldRightBottom; }

	// �����������ķ�Χ
	void SetWorldRange(const CPnt& ptWorldLeftTop, const CPnt& ptWorldRightBottom);

	// �����ӿ���һ�㵽��������ϵ��ĳһ��Ķ�Ӧ��ϵ
	void SetPointMapping(const CPoint& ptWindow, const CPnt& ptWorld);

	// ������ʾ������
	void SetRatio(float fRatio);

	// �����ӿڵĴ�С
	void SetViewPort(USHORT uWidth, USHORT uHeight);

	void SetViewPortDiff(USHORT uWidthDiff, USHORT uHeightDiff);
	// ȡ���ӿ����ϽǴ�����Ӧ������������
	CPnt GetLeftTopPoint() const;

	// ȡ���ӿ����Ĵ�����Ӧ������������
	CPnt GetCenterPoint() const { return m_ptCenter; }

	// ȡ���������Ŀ��
	float GetWorldWidth() const { return m_ptWorldRightBottom.x - m_ptWorldLeftTop.x; }

	// ȡ���������ĸ߶�
	float GetWorldHeight() const { return  m_ptWorldLeftTop.y - m_ptWorldRightBottom.y; }

#ifdef _MFC_VER
	// ȡ���ӿ���ָ��λ������Ӧ������������
	CPnt GetWorldPoint(const CPoint& pntWindow) const;
	void SetPointMappping(int x, int y, CPnt& pt);
	// ȡ��ָ�������������Ӧ���ӿڵ��λ��
	CPoint GetWindowPoint(const CPnt& ptWorld) const;
#endif
};
