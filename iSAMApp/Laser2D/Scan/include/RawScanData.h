#ifndef __CRawScanData
#define __CRawScanData

#include <list>
#include <vector>
#include "Geometry.h"

using namespace std;

typedef vector<double> RangeReadings;

class CRawScanData
{
public:
	int           nCount;                     // 每周扫描线数
	float         fStartAngle;                // 起始扫描角
	float         fEndAngle;                  // 终止扫描角
	RangeReadings Readings;                   // 扫描线数据
	CPosture      pstOdometry;                // 扫描时所处的姿态
};

typedef CRawScanData* CScanDataPtr;
typedef list<CScanDataPtr> ScanDataSet;

#endif
