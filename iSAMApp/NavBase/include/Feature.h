#ifndef __CFeature
#define __CFeature

#include "Geometry.h"
#include "ScrnRef.h"

#define FEATURE_DIR_FRONT_SIDE_ONLY           1        // ֻʹ�ö�ֱ������
#define FEATURE_DIR_BACK_SIDE_ONLY            2        // ֻʹ�ö�ֱ�߷���
#define FEATURE_DIR_BOTH_SIDES                3        // ʹ�ö�ֱ����������

#define SIASUN_MATCHER_ANGLE_WINDOW           (15*PI/180)   // +/-15�ȿ��Ž�
#define SIASUN_MATCHER_DIST_WINDOW            (400)         // +/-400mm����

///////////////////////////////////////////////////////////////////////////////
//   ���塰���������ࡣ

class CFeature
{
public:
	int      m_nType;               // ���������ͱ��
	int      m_nID;                 // ��������ID��
	bool     m_bDelete;             // �Ƿ񽫱�ɾ��

public:
	CFeature() 
	{
		m_nType = 0;
		m_nID = 0;
		m_bDelete = false;
	}

	virtual bool Load(FILE* fp) = 0;
	virtual bool Save(FILE* fp) = 0;

#ifdef _MSC_VER
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize) = 0;
#endif
};

#endif
