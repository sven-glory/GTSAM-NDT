#ifndef __CScanPointCloud
#define __CScanPointCloud

#include "ScrnRef.h"
#include <afxwin.h>

///////////////////////////////////////////////////////////////////////////////
// ��CScanPoint���ඨ���˶�άɨ���ĸ��

class CScanPoint : public CPnt
{
public:
	short m_nWeight;     // ��Ӧ��ԭʼ���������������ʾ�õ�ġ�������
	short m_nLineID;     // ��Ӧ��ֱ�߶εı��(����õ�����һ��ֱ�߶���),����-1
	float m_fScanAng;    // �˵��Ӧ��ɨ��������ɨ���

public:
	CScanPoint()
	{
		m_nWeight = 0;
		m_nLineID = -1;
		m_fScanAng = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
// ��CScanPointCloud���ඨ���ά���ơ�

class CScanPointCloud
{
public:
	int         m_nCount;                    // ɨ�������
	CScanPoint* m_pPoints;                   // ָ��ɨ������ݻ�������ָ��

public:
	// ����ָ���ĵ��������ɶ���(ֻ����ռ�)
	CScanPointCloud(int nNum);

	// ��������һ���������ɶ���
	CScanPointCloud(const CScanPointCloud& Cloud);

	// ���ɿյĶ���
	CScanPointCloud();

	~CScanPointCloud();

	CScanPointCloud* GetScanPointCloudPointer() {return this;}

	// Ϊ�������ݷ���ռ�
	BOOL Create(int nNum);

	// converts raw range readings to scan, taking into account interlaced scan and
	// robot velocities.  If deinterlace is TRUE then only every interlace-th
	// scan point is generated. 
	BOOL CreateFromPolarRanges(int nNum, short interlace, bool deinterlace, 
											const USHORT *r, 
											/*const PoseGauss *robotPos, */
											float tvel, float rvel, const CPosture *relPose, 
											float startA, float endA, USHORT maxRange, 
											float scanTime);

	// �������ԭ��������
	void Clear();

	// ���ء�=��������
	void operator = (const CScanPointCloud& Cloud2);

	// ���ء�+=��������
	void operator += (const CScanPointCloud& Cloud2);

	// ��ȫ�����ƶ�ָ���ľ���
	void Move(float fDx, float fDy);

	// ��ȫ������ָ�������ĵ������ת
	void Rotate(float centerX, float centerY, float angle);

	// ������ƿɼ����(Ŀǰδʹ��)
	float AreaSize();

	// ����ɨ�������ռ��ɨ���
	float FieldOfView();

	// ����Ӹ����㵽�����и����ƽ������
	float AverageDistanceFrom(const CPnt& pt);

	// ��������ĵ㵽�����и����ƽ������
	float AverageDistanceFromGravity();

	// ������Ƶ����Ĳ�(�������о���������Զ�����������֮��ľ���)
	float DynamicsGravity();

	// ���ص��Ƶ����ĵ�
	CPnt CenterOfGravity();

	// ������ƵĻ��Ƴ���
	float TotalLength();

	float LeftMost();
	float TopMost();
	float RightMost();
	float BottomMost();
	float Width();
	float Height();

	// ��ָ���ĵ��Ϊ���ģ���ָ���İ뾶�����е���й��ˣ�ֻ���´��ڰ뾶���ڵĵ�
	void Reduce(CPnt& ptCenter, float dRadius);

	int ClosestPoint(const CPnt& ptNew, float* fDist = NULL);

	void Add(int nIndex, int nCurCount, const CScanPoint& sp);

	// �ӵ�����ɾ��һ����
	void Delete(int nIndex);

	// �˶Ե����Ƿ����ָ���ĵ�
	int ContainPoint(const CPnt& pt, float fThreshHoldDist);

	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1);

	void Dump();
	void DebugFilter(float fMinX, float fMaxX, float fMinY, float fMaxY);

	BOOL LoadFromFile(FILE* file);

	BOOL SaveToFile(FILE* file);
};

/*
** Calculates a line in the form (n1, n2) * (x, y) + c = 0,
** so that the leastsquare sum of the corresponding scan points
** is minimized.
** sp is an array of scan points, num specifies the number of scan points,
** n1, n2 and c are the result values.
*/
BOOL RegressionLine(const CScanPoint *sp, long num,
    float *n1, float *n2, float *c);

/*
** Calculates variance of distance from a set of scan points 
** given in (sp, num) to a line given in (n1, n2, c).
*/
float LineDeviation(CScanPoint *sp, long num,
    float n1, float n2, float c);

#endif
