#ifndef __CLineScan
#define __CLineScan

#include "LineFeature.h"
#include "misc.h"
#include <afxwin.h>

class CScan;

//
//   CLineFeatureSet描述扫描中的直线段。
//
class CLineFeatureSet
{
public:
    CLineFeature* m_pLines;               // 直线缓冲区首指针
    long          m_nCount;               // 直线的数量
    PoseGauss     ls_Pos;                 // 车体参考姿态

public:
	CLineFeatureSet(const CScan& scan);

	CLineFeatureSet(int nNum);

	CLineFeatureSet();

	~CLineFeatureSet();

	void CreateFromScan(const CScan& scan);

	// 重载“=”操作符
	void operator = (const CLineFeatureSet& LineScan);

	// 清除所有的直线扫描(CLineFeatureSet)
	void Clear();

	// 分配内存并复制当前对象内容
	CLineFeatureSet *Duplicate();

	// 将此另一个直线段集并入此直线段集中
	BOOL Merge(const CLineFeatureSet& LineScan);

	// 通过合并共线的线段来简化此直线段集合
	BOOL Simplify();

	// 删除指定的线段
	BOOL Remove(int nIdx);

	// returns field of view of linescan in rad.
	float FieldOfView(CScan *scan);

	// returns total length of all line segments
	float TotalLength();

	// 去掉所有长度短于minLineLength的直线
	void LengthFilter(float minLineLength);

	void Dump();

	// 判断直线扫描集是否包含指定的点
	BOOL ContainScanPoint(const CScanPoint& sp);

	// 移除位于指定区域内的线段
	void RemoveWithin(CRegion& rgn);

	void Select(int nIdx, bool bOn);

	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize = 1);
};

/*
** Merges lines that are co-linear and connected.
*/
void LineScanMergeLines(CLineFeatureSet *ls, CScan *scan, long *newLineNumbers);

#endif
