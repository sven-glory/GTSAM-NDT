#include "stdafx.h"
#include <math.h>
#include "misc.h"
#include "Scan.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPoint>
#include <QPainter>
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

static CPosture PoseZero;
extern CPostureGauss PoseGaussZero;

///////////////////////////////////////////////////////////////////////////////

CScan::CScan()
{
}

//
//   构造具有nNum个点的CScan对象。
//
CScan::CScan(int nNum) : CScanPointCloud(nNum)
{
}

CScan::CScan(CScanPointCloud* pCloud)
{
	Create(pCloud);
}

CScan::~CScan()
{
}

//
//   重载“=”操作符
//
void CScan::operator = (const CScan& Scan)
{
	*(GetScanPointCloudPointer()) = *(((CScan&)Scan).GetScanPointCloudPointer());

	m_poseScanner = Scan.m_poseScanner;
	m_poseRelative = Scan.m_poseRelative;
	m_fStartAng = Scan.m_fStartAng;
	m_fEndAng = Scan.m_fEndAng;

	// 为何不拷贝点特征集合？
}

//
//   清除所有数据。
//
void CScan::Clear()
{
	// 清除点云数据
	CScanPointCloud::Clear();

	// 清除其它所有数据
	// ..
}

//
//   生成一个新的扫描。
//
bool CScan::Create(int nNum)
{
	// 在此应将所有数据成员初始化为0
	Clear();

	// 为点云分配空间
	if (!CScanPointCloud::Create(nNum))
		return false;

	return true;
}

//
//   根据给定点云生成一个新的扫描
//
bool CScan::Create(CScanPointCloud* pCloud)
{
	Clear();

	if (!Create(pCloud->m_nCount))
		return false;

	*(GetScanPointCloudPointer()) = *pCloud;

	return true;
}

//
//   分配空间并复制当前扫描。
//
CScan *CScan::Duplicate()
{
	CScan *scan = new CScan(m_nCount);
	if (scan == NULL)
		return NULL;

	*scan = *this;
	return scan;
}

#if 0
//
//   将全部点移动指定的距离。
//
void CScan::Move(float dx, float dy)
{
	if ((dx == 0) && (dy == 0))
		return;

	// 移动点云数据
	CScanPointCloud::Move(dx, dy);

	// 移动激光头的均值点
	m_poseScanner.Move(dx, dy);
}
#endif

static int spcmp(const void *v1, const void *v2)
{
	const CScanPoint *sp1 = (CScanPoint *)v1, *sp2 = (CScanPoint *)v2;

	/* float da = NormAngle(sp1->a - sp2->a); not a sorting order */
	float da = sp1->a - sp2->a;

	if (da < 0.0)
		return -1;
	else if (da > 0.0)
		return 1;

	return 0;
}
///////////////////////////////////////////////////////////////////////////////
//
//
//   根据给定的“强反光门限”值，标记哪些点是属于“强反光”。
//
void CScan::MarkReflectivePoints(int nReflectiveGateValue)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].m_bHighReflective = (m_pPoints[i].m_nIntensity >= nReflectiveGateValue);
}

//
//   启用新的激光扫描器有效范围。
//
void CScan::ApplyNewScannerAngles(const CRangeSet& AngRanges)
{
	// 滤除可用角度之外的扫描数据
	for (int i = 0; i < m_nCount; i++)
	{
		float ang = m_pPoints[i].a;
		if (!AngRanges.Contain(ang))
		{
			m_pPoints[i].r = 0;
			m_pPoints[i].x = 0;
			m_pPoints[i].y = 0;
			m_pPoints[i].m_nIntensity = 0;
		}
	}
}

//
//   启用关于扫描角度范围的约束(处于给定角度范围之外的部分将被滤除)。
//
void CScan::ApplyScanAngleRule(float fMinAngle, float fMaxAngle)
{
	CRange range(fMinAngle, fMaxAngle);
	for (int i = 0; i < m_nCount; i++)
	{
		float ang = m_pPoints[i].a;
		if (!range.Contain(ang))
		{
			m_pPoints[i].r = 0;
			m_pPoints[i].x = 0;
			m_pPoints[i].y = 0;
			m_pPoints[i].m_nIntensity = 0;
		}
	}
}

//
//   启用关于扫描距离的约束(超出给定距离范围之外的部分将被滤除)。
//
void CScan::ApplyScanDistRule(float fMinDist, float fMaxDist)
{
	for (int i = 0; i < m_nCount; i++)
	{
		float r = m_pPoints[i].r / 1000;
		if (r < fMinDist || r > fMaxDist)
		{
			m_pPoints[i].r = 0;
			m_pPoints[i].x = 0;
			m_pPoints[i].y = 0;
			m_pPoints[i].m_nIntensity = 0;
		}
	}
}


//
//   从二进制文件中读取扫描数据。
//
bool CScan::LoadBinary(FILE* fp, const CPosture& pstRobot, const CLaserScannerParam& Param, int nFileVersion)
{
	// 先计算激光器的姿态
	CFrame frame(pstRobot);
	m_poseScanner.SetPosture(Param.m_pst);
	m_poseScanner.InvTransform(frame);

	m_poseRelative = m_poseScanner;
	m_fStartAng = Param.m_fStartAngle;
	m_fEndAng = Param.m_fEndAngle;

	if (!CScanPointCloud::LoadBinary(fp, Param.m_fStartAngle, Param.m_fEndAngle, Param.m_nLineCount, nFileVersion))
		return false;

	return true;
}

//
//   将扫描数据保存到十进制文件。
//
bool CScan::SaveBinary(FILE* fp, int nFileVersion)
{
	if (!CScanPointCloud::SaveBinary(fp, nFileVersion))
		return false;

	return true;
}

//
//   对整个点云进行坐标系变换。
//
void CScan::Transform(const CFrame& frame)
{
	CScanPointCloud::Transform(frame);
	m_poseScanner.Transform(frame);
}

//
//   对整个点云进行坐标系逆变换。
//
void CScan::InvTransform(const CFrame& frame)
{
	CScanPointCloud::InvTransform(frame);
	m_poseScanner.InvTransform(frame);
}

#define ARROW_LEN     20          // 20个像素

#ifdef _MFC_VER
void CScan::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, COLORREF crHighLightPoint,
	bool bShowScanner, int nPointSize)
{
	// 取得扫描时激光头的姿态
	CPosture& pstRef = m_poseScanner.GetPostureObject();

	CPen pen(PS_SOLID, 1, crColorPoint);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CBrush Brush(crColorPoint);
	CBrush* pOldBrush = pDC->SelectObject(&Brush);

	CBrush HighLightBrush(crHighLightPoint);

	CPnt pt;

	for (int i = 0; i < m_nCount; i++)
	{
		if (m_pPoints[i].m_bDelete)
			continue;

		if (!m_pPoints[i].m_bHighReflective)
			pDC->SelectObject(&Brush);
		else
			pDC->SelectObject(&HighLightBrush);

		CPnt pt;

		// 如果需要在世界坐标系中显示，现在需要进行坐标变换
		pt = m_pPoints[i];

		CPoint pnt = ScrnRef.GetWindowPoint(pt);

		int nSize = nPointSize;
		if (m_pPoints[i].m_bDelete)
			nSize /= 2;

		// 在此，支持“微距观察模式”(即在放大倍数很大的情况下，每个点也放大显示)
		if (nSize == 1)
		{
			if (ScrnRef.m_fRatio > 500)
				nSize = 1 * ScrnRef.m_fRatio / 500;
		}

		CRect r(pnt.x - nSize, pnt.y - nSize, pnt.x + nSize, pnt.y + nSize);
		pDC->Ellipse(&r);
	}
	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);

	if (bShowScanner)
		m_poseScanner.Draw(ScrnRef, pDC, RGB(255, 0, 255), 40, 150, 1);
}

#elif defined QT_VERSION

void CScan::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crHighLightPoint,
					  bool bShowScanner, int nPointSize)
{
	// 取得扫描时激光头的姿态
	CPosture& pstRef = m_poseScanner.GetPostureObject();

	QPen pen(crColor);
	pen.setWidth(1);
	pPainter->setPen(pen);

	QBrush brush(crColor);
	QBrush highLightBrush(crHighLightPoint);

	CPnt pt;

	for (int i = 0; i < m_nCount; i++)
	{
		if (m_pPoints[i].m_bDelete)
			continue;

		if (!m_pPoints[i].m_bHighReflective)
			pPainter->setBrush(brush);
		else
			pPainter->setBrush(highLightBrush);

		CPnt pt;

		// 如果需要在世界坐标系中显示，现在需要进行坐标变换
		pt = m_pPoints[i];

		QPoint pnt = ScrnRef.GetWindowPoint(pt);

		int nSize = nPointSize;
		if (m_pPoints[i].m_bDelete)
			nSize /= 2;

		// 在此，支持“微距观察模式”(即在放大倍数很大的情况下，每个点也放大显示)
		if (nSize == 1)
		{
			if (ScrnRef.m_fRatio > 500)
				nSize = static_cast<int>(1 * ScrnRef.m_fRatio / 500);
		}

		QPoint d(nSize, nSize);

		QRect r(pnt-d, pnt+d);
		pPainter->drawEllipse(r);
	}

	if (bShowScanner)
		m_poseScanner.Draw(ScrnRef, pPainter, Qt::magenta, 40, 150);
}

#endif
