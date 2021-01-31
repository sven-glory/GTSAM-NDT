#include "stdafx.h"
#include "scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//
//   ��ȫ������תָ������̬��
//
void CScan::RotatePos(float angle, float mx, float my)
{
	if (angle != 0.0)
	{
		float sina = sin(angle);
		float cosa = cos(angle);

		CScanPoint *sp = m_pPoints;

		m_poseScanner.m_pstMean.fThita += angle;

		float x, y;
		for (int i = 0; i < m_nCount; i++, sp++)
		{
			x = sp->x - mx;
			y = sp->y - my;
			sp->x = mx + cosa * x - sina * y;
			sp->y = my + sina * x + cosa * y;
		}

		x = m_poseScanner.m_pstMean.x - mx;
		y = m_poseScanner.m_pstMean.y - my;
		m_poseScanner.m_pstMean.x = mx + cosa * x - sina * y;
		m_poseScanner.m_pstMean.y = my + sina * x + cosa * y;
	}
}

//
//   ��ȫ������תָ������̬��
//
void CScan::Rotate(float angle)
{
	RotatePos(angle, m_poseScanner.m_pstMean.x, m_poseScanner.m_pstMean.y);
}
