#ifndef SL_COX_H
#define SL_COX_H

#include "CoxInfo.h"
#include "matrix.h"

struct CoxEquation
{
	float s11, s12, s13, s22, s23, s33;
	float s2;
};

//
//   ����Cox�㷨��ƥ������
//
class CCoxMatcher
{
private:
	CScan*         m_pRefScan;              // ָ�򡰲ο�ɨ�衱��ָ��
	CScan*         m_pCurScan;              // ָ�򡰵�ǰɨ�衱��ָ��
	CScan*         m_pReducedScan;          // ���������˲��ġ���ǰɨ�衱ָ��
	CScan*         m_pWorkScan;             // ���ڽ�����ת��ƽ�Ʋ������Ĺ���ɨ��
	CCoxInfoArray* m_pCoxInfoArray;         // ����Ķ�Ӧ��ϵ

	struct CoxEquation m_ce;           /* equation system matrix and vector */

private:
	// �ͷ����ж�̬������ڴ�
	void Cleanup();

	// ���С���-�ߡ�ƥ��
	int MatchPointsToLines(int iteration, int *pnum, float *ddx, float *ddy, float *dda,
		float *error,	bool *allDofFixed);

	// ��ͼ��m_pWorkScan�е�ÿ��ɨ����ҵ�һ����֮��Ӧ��ֱ�߶�
	short AssignNearestLines();

	// ����Coxƥ�����
	BOOL Setup(CScan* pRefScan, CScan* pCurScan);

	// ���õ����������С���-�ߡ�ƥ��
	int ScanMatchHelp(PoseGauss *match, float *error);

public:
	// ���캯��
	CCoxMatcher();

	// ��������
	~CCoxMatcher();

	// ����ƥ������
	int Match(CScan* pRefScan, CScan* pCurScan, PoseGauss* pPose, float* pError);
};
#endif
