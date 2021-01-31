#ifndef __CLaserScannerParam
#define __CLaserScannerParam

#include <vector>
#include "Geometry.h"
#include "Range.h"

using namespace std;

// ����ɨ��������
class CLaserScannerParam
{
public:
    bool state = false;
    int LaserId = 0;
    int LaserProductor = 0; //0 - pf, 1 - hokuyo, 2 - sick
	float m_fStartAngle;               // ��ʼɨ���(��λ������)
	float m_fEndAngle;                 // ��ֹɨ���(��λ������)
    float m_fRefViewAngle;
	int   m_nLineCount;                // ɨ������
	float m_fReso;                     // �ֱ���(��λ������/��)
	CPosture m_pst;                    // ����ڻ����˵İ�װ��̬
	CRangeSet m_AppAngleRange;         // ʵ�ʿ��ýǶȷ�Χ(�ɴ�������װλ�þ���)
    double m_fMaxRange;                  //��ɨ����Զ����
    double m_fMinRange;                  //��ɨ���������
    char strIP_c[15];
    std::string strIP = "";
    std::string hostIP = "";
	bool  m_bUseCreateModel;           // �Ƿ����ô�ɨ������ģ(������RoboMapping�����)
	bool  m_bUseLocalize;              // �Ƿ����ô�ɨ������λ(������RoboMapping�����)

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

	// ������ʼ�ǡ���ֹ�ߺ�ɨ������
	void Set(float fStartAngle, float fEndAngle, int nLineCount);

	// �Ӷ������ļ��ж�ȡ����
	bool LoadBinary(FILE* fp, int nFileVersion = 200);

	// ��������ļ���д�����
	bool SaveBinary(FILE* fp, int nFileVersion = 200);
};

// ������ɨ�����顱������
class CScannerGroupParam : public  vector<CLaserScannerParam>
{
public:
    CScannerGroupParam();

    ~CScannerGroupParam();

	void SetRelativePosture(int nRefScanner, int nCurScanner, const CPosture& pst);

	// �Ӷ������ļ��ж�ȡ����
	bool LoadBinary(FILE* fp, int nFileVersion = 200);

	// ��������ļ���д�����
	bool SaveBinary(FILE* fp, int nFileVersion = 200);

	// ���ø�ɨ�����Ƿ�Ӧ���ڽ�ģ
	void UseWhenCreateModel(int nIdx, bool bYesOrNo);

	// ���ø�ɨ�����Ƿ�Ӧ���ڶ�λ
	void UseWhenLocalize(int nIdx, bool bYesOrNo);
};
#endif
