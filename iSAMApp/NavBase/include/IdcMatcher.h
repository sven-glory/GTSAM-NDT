#ifndef __CIdcMatcher
#define __CIdcMatcher

#include "matrix.h"

//
//   ����IDC�㷨��ƥ������
//
class CIdcMatcher
{
private:
	CScan*         m_pRefScan;              // ָ�򡰲ο�ɨ�衱��ָ��
	CScan*         m_pCurScan;              // ָ�򡰵�ǰɨ�衱��ָ��

public:
	// ���캯��
	CIdcMatcher();

	// ��������
	~CIdcMatcher();

	// ����ƥ������
	int Match(CScan* pRefScan, CScan* pCurScan, PoseGauss* pPose, float* pError);
};
#endif
