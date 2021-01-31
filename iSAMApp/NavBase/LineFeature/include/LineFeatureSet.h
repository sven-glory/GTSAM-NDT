#ifndef __CLineScan
#define __CLineScan

#include "LineFeature.h"
#include "misc.h"
#include <afxwin.h>

class CScan;

//
//   CLineFeatureSet����ɨ���е�ֱ�߶Ρ�
//
class CLineFeatureSet
{
public:
    CLineFeature* m_pLines;               // ֱ�߻�������ָ��
    long          m_nCount;               // ֱ�ߵ�����
    PoseGauss     ls_Pos;                 // ����ο���̬

public:
	CLineFeatureSet(const CScan& scan);

	CLineFeatureSet(int nNum);

	CLineFeatureSet();

	~CLineFeatureSet();

	void CreateFromScan(const CScan& scan);

	// ���ء�=��������
	void operator = (const CLineFeatureSet& LineScan);

	// ������е�ֱ��ɨ��(CLineFeatureSet)
	void Clear();

	// �����ڴ沢���Ƶ�ǰ��������
	CLineFeatureSet *Duplicate();

	// ������һ��ֱ�߶μ������ֱ�߶μ���
	BOOL Merge(const CLineFeatureSet& LineScan);

	// ͨ���ϲ����ߵ��߶����򻯴�ֱ�߶μ���
	BOOL Simplify();

	// ɾ��ָ�����߶�
	BOOL Remove(int nIdx);

	// returns field of view of linescan in rad.
	float FieldOfView(CScan *scan);

	// returns total length of all line segments
	float TotalLength();

	// ȥ�����г��ȶ���minLineLength��ֱ��
	void LengthFilter(float minLineLength);

	void Dump();

	// �ж�ֱ��ɨ�輯�Ƿ����ָ���ĵ�
	BOOL ContainScanPoint(const CScanPoint& sp);

	// �Ƴ�λ��ָ�������ڵ��߶�
	void RemoveWithin(CRegion& rgn);

	void Select(int nIdx, bool bOn);

	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize = 1);
};

/*
** Merges lines that are co-linear and connected.
*/
void LineScanMergeLines(CLineFeatureSet *ls, CScan *scan, long *newLineNumbers);

#endif
