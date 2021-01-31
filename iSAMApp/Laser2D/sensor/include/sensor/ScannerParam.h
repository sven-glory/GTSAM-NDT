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
    bool state = false;
    int LaserId = 0;
    int LaserProductor = 0; //0 - pf, 1 - hokuyo, 2 - sick
	float m_fStartAngle;               // 起始扫描角(单位：弧度)
	float m_fEndAngle;                 // 终止扫描角(单位：弧度)
    float m_fRefViewAngle;
	int   m_nLineCount;                // 扫描线数
	float m_fReso;                     // 分辩率(单位：弧度/线)
	CPosture m_pst;                    // 相对于机器人的安装姿态
	CRangeSet m_AppAngleRange;         // 实际可用角度范围(由传感器安装位置决定)
    double m_fMaxRange;                  //　扫描最远距离
    double m_fMinRange;                  //　扫描最近距离
    char strIP_c[15];
    std::string strIP = "";
    std::string hostIP = "";
	bool  m_bUseCreateModel;           // 是否利用此扫描器建模(仅用于RoboMapping软件中)
	bool  m_bUseLocalize;              // 是否利用此扫描器定位(仅用于RoboMapping软件中)

public:
	CLaserScannerParam();

    void Clear()
    {
        m_AppAngleRange.clear();
    }

    CLaserScannerParam(const CLaserScannerParam& other)
    {
        Clear();
        this->state = other.state;
        this->LaserId = other.LaserId;
        this->LaserProductor = other.LaserProductor;
        this->m_fStartAngle = other.m_fStartAngle;
        this->m_fEndAngle = other.m_fEndAngle;
        this->m_fRefViewAngle = other.m_fRefViewAngle;
        this->m_nLineCount = other.m_nLineCount;
        this->m_fReso = other.m_fReso;
        this->m_pst = other.m_pst;
        this->m_AppAngleRange = other.m_AppAngleRange;
        this->m_fMaxRange = other.m_fMaxRange;
        this->m_fMinRange = other.m_fMinRange;
        memcpy(this->strIP_c, other.strIP_c, 15*sizeof(char));
        this->strIP = other.strIP;
        this->m_bUseCreateModel = other.m_bUseCreateModel;
        this->m_bUseLocalize = other.m_bUseLocalize;
    }

    CLaserScannerParam& operator = (const CLaserScannerParam& Obj)
    {
        Clear();
        this->state = Obj.state;
        this->LaserId = Obj.LaserId;
        this->LaserProductor = Obj.LaserProductor;
        this->m_fStartAngle = Obj.m_fStartAngle;
        this->m_fEndAngle = Obj.m_fEndAngle;
        this->m_fRefViewAngle = Obj.m_fRefViewAngle;
        this->m_nLineCount = Obj.m_nLineCount;
        this->m_fReso = Obj.m_fReso;
        this->m_pst = Obj.m_pst;
        this->m_AppAngleRange = Obj.m_AppAngleRange;
        this->m_fMaxRange = Obj.m_fMaxRange;
        this->m_fMinRange = Obj.m_fMinRange;
        memcpy(this->strIP_c, Obj.strIP_c, 15*sizeof(char));
        this->strIP = Obj.strIP;
        this->m_bUseCreateModel = Obj.m_bUseCreateModel;
        this->m_bUseLocalize = Obj.m_bUseLocalize;
        return *this;
    }

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
    CScannerGroupParam();

    ~CScannerGroupParam();

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
