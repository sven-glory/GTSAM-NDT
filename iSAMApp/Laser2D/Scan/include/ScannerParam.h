#ifndef __CLaserScannerParam
#define __CLaserScannerParam

#include <vector>
#include "Geometry.h"
#include "Range.h"

using namespace std;

// 激光扫描器参数
class CLaserScannerParam
{
public:
	float m_fStartAngle;               // 起始扫描角(单位：弧度)
	float m_fEndAngle;                 // 终止扫描角(单位：弧度)
	int   m_nLineCount;                // 扫描线数
	float m_fReso;                     // 分辩率(单位：弧度/线)
	CPosture m_pst;                    // 相对于机器人的安装姿态
	CRangeSet m_AppAngleRange;         // 实际可用角度范围(由传感器安装位置决定)
	bool  m_bUseCreateModel;           // 是否利用此扫描器建模(仅用于RoboMapping软件中)
	bool  m_bUseLocalize;              // 是否利用此扫描器定位(仅用于RoboMapping软件中)

public:
	CLaserScannerParam();

	// 设置起始角、终止线和扫描线数
	void Set(float fStartAngle, float fEndAngle, int nLineCount);

	// 从二进制文件中读取参数
	bool LoadBinary(FILE* fp, int nFileVersion = 200);

	// 向二进制文件中写入参数
	bool SaveBinary(FILE* fp, int nFileVersion = 200);
};

// “激光扫描器组”参数。
class CScannerGroupParam : public  vector<CLaserScannerParam>
{
public:
	void SetRelativePosture(int nRefScanner, int nCurScanner, const CPosture& pst);

	// 从二进制文件中读取参数
	bool LoadBinary(FILE* fp, int nFileVersion = 200);

	// 向二进制文件中写入参数
	bool SaveBinary(FILE* fp, int nFileVersion = 200);

	// 设置各扫描器是否应用于建模
	void UseWhenCreateModel(int nIdx, bool bYesOrNo);

	// 设置各扫描器是否应用于定位
	void UseWhenLocalize(int nIdx, bool bYesOrNo);
};
#endif
