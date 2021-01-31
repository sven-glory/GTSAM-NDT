#include "stdafx.h"
#include <math.h>
#include "LineFeature.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define ANGLE_COST_FACTOR       (180.0f/PI*0.1f)     // 角度代价因子(每度折合距离100mm)
#define PROJECT_LINE_LEN        2.0f

///////////////////////////////////////////////////////////////////////////////

CLineFeature::CLineFeature(CPnt& ptStart, CPnt& ptEnd)
{
	Create(ptStart, ptEnd);
	m_nSubType = GENERIC_LINE_FEATURE;

	m_lStart = m_lEnd = 0;
	m_fSigma2 = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;
	m_nId = -1;
	m_bCreateRef = false;
}

CLineFeature::CLineFeature() 
{
	m_lStart = m_lEnd = 0;
	m_fSigma2 = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;
	m_bSelected = false;
	m_nId = -1;
	m_bCreateRef = false;
}

//
//   生成一个复本
//
CLineFeature* CLineFeature::Duplicate() const
{
	CLineFeature* p = new CLineFeature;
	*p = *this;
	return p;
}

int CLineFeature::LoadText(FILE* fp)
{
	int nCount;

	CPnt ptStart, ptEnd;
	if (fscanf(fp, "%f\t%f\t%f\t%f\t%d\t%d", &ptStart.x, &ptStart.y, &ptEnd.x, &ptEnd.y, &m_nWhichSideToUse, &nCount) != 6)
		return -1;

	Create(ptStart, ptEnd);

	m_Ranges.clear();

	CRange range;
	if (nCount == 1)
	{
		range.fFrom = 0;
		range.fTo = Length();
		m_Ranges.push_back(range);
	}
	else
	{
		for (int i = 0; i < nCount; i++)
		{
			if (fscanf(fp, "\t%f\t%f\n", &range.fFrom, &range.fTo) != 2)
				return -1;
			m_Ranges.push_back(range);
		}
	}

	return m_nSubType;
}

int CLineFeature::SaveText(FILE* fp)
{
	int nCount = m_Ranges.size();

	fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%d\t\t%d\n", m_nSubType, m_ptStart.x, m_ptStart.y,
		m_ptEnd.x, m_ptEnd.y, m_nWhichSideToUse, nCount);

	if (nCount > 1)
	{
		for (int i = 0; i < nCount; i++)
			fprintf(fp, "\t%f\t%f\n", m_Ranges[i].fFrom, m_Ranges[i].fTo);
		fprintf(fp, "\n");
	}

	return m_nSubType;
}

//
//   从二进制文件读入数据。
//
int CLineFeature::LoadBinary(FILE* fp)
{
	// 读入两个端点的坐标
	float f[4];
	if (fread(f, sizeof(float), 4, fp) != 4)
		return -1;

	CPnt ptStart, ptEnd;
	ptStart.x = f[0];
	ptStart.y = f[1];
	ptEnd.x = f[2];
	ptEnd.y = f[3];

	Create(ptStart, ptEnd);

	// 读入直线特征的有效朝向及共线数量
	int n[2];
	if (fread(n, sizeof(int), 2, fp) != 2)
		return -1;

	m_nWhichSideToUse = n[0];
	int nCount = n[1];

	m_Ranges.clear();

	CRange range;

	// 如果只有一个共线段
	if (nCount == 1)
	{
		range.fFrom = 0;
		range.fTo = Length();
		m_Ranges.push_back(range);
	}
	else
	{
		// 如果有多个共线段，现在依次添加
		for (int i = 0; i < nCount; i++)
		{
			if (fread(f, sizeof(float), 2, fp) != 2)
				return false;

			range.fFrom = f[0];
			range.fTo = f[1];

			m_Ranges.push_back(range);
		}
	}

	return m_nSubType;
}

//
//   将数据写入到二进制文件。
//
int CLineFeature::SaveBinary(FILE* fp)
{
	// 写入直线类型
	if (fwrite(&m_nSubType, sizeof(int), 1, fp) != 1)
		return -1;

	// 写入两个端点的坐标
	float f[4] = { m_ptStart.x, m_ptStart.y, m_ptEnd.x, m_ptEnd.y };
	if (fwrite(f, sizeof(float), 4, fp) != 4)
		return -1;

	int nCount = (int)m_Ranges.size();
	int n[2] = { m_nWhichSideToUse, nCount };

	// 写入直线特征的有效朝向及共线数量
	if (fwrite(n, sizeof(float), 2, fp) != 2)
		return -1;

	// 如果有多个共线段，现在依次写入各段位置
	if (nCount > 1)
	{
		for (int i = 0; i < nCount; i++)
		{
			f[0] = m_Ranges[i].fFrom;
			f[1] = m_Ranges[i].fTo;

			if (fwrite(n, sizeof(float), 2, fp) != 2)
				return -1;
		}
	}

	return m_nSubType;
}

//
//   生成线段。
//
void CLineFeature::Create(CPnt& ptStart, CPnt& ptEnd)
{
	CMultiSegLine::Create(ptStart, ptEnd);

	m_bSelected = false;
}

//
//   生成线段。
//
void CLineFeature::Create(CLine& ln)
{
	Create(ln.m_ptStart, ln.m_ptEnd);
}

//
//   根据多段线生成特征。
//
void CLineFeature::Create(CMultiSegLine& MultiSegLine)
{
	CMultiSegLine::Create(MultiSegLine);
	m_bSelected = false;
}

//
//   设置扫检测到该直线特征时的激光头姿态，以便计算有效的观测朝向。
//
void CLineFeature::SetDetectPosture(const CPosture& pstDetect)
{
	CPnt ptDetect = pstDetect;

	CAngle ang1(ptDetect, m_ptStart);
	CAngle ang2(ptDetect, m_ptEnd);

	// 取直线段的中点为投影点
	m_bCreateRef = true;
	m_ptProjectFoot = GetMidpoint();
	CAngle ang = SlantAngle();

	// 当从ang1到ang2的转角小于PI时，扫描位置在直线特征的左面(正面)
	if ((ang2 - ang1).NormAngle() < PI)
	{
		m_nWhichSideToUse = FEATURE_DIR_FRONT_SIDE_ONLY;
		ang += PI / 2;
	}
	else
	{
		m_nWhichSideToUse = FEATURE_DIR_BACK_SIDE_ONLY;
		ang -= PI / 2;
	}

	// 构造投影线
	CLine ln(m_ptProjectFoot, ang, PROJECT_LINE_LEN);

	// 记录参考点
	m_ptRef = ln.GetEndPoint();
}

//
//   判断本多段线是否与另一多段线有重叠区。
//
bool CLineFeature::IsOverlapWith(CLineFeature& Feature2)
{
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line1;
		GetSegment(i, Line1);

		for (int j = 0; j < (int)Feature2.m_Ranges.size(); j++)
		{
			CLine Line2;
			Feature2.GetSegment(j, Line2);

			// 下面判断是否有相重叠的部分
			float lambda1, lambda2;

			// 计算线段LineFeature2的起点、终点到本直线(不是线段!)的距离
			Line1.DistanceToPoint(false, Line2.m_ptStart, &lambda1);
			Line1.DistanceToPoint(false, Line2.m_ptEnd, &lambda2);

			// 如果两个投影点都落在线段以外
			if (((lambda1 < 0) && (lambda2 < 0)) || ((lambda1 > 1) && (lambda2 > 1)))
				continue;
			else
				return true;
		}
	}

	return false;
}

bool CLineFeature::ColinearMerge1(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines)
{
	// 先判断是否平行，如果不平行直接返回false
	if (!IsParallelTo(Feature2, fMaxAngDiff))
		return false;

	// 计算两个特征中所含直线分段的数量
	int nNum = m_Ranges.size() + Feature2.m_Ranges.size();
	
	// 并按此数量分配直线数组空间
	CLine* pLine = new CLine[nNum];

	// 复制所有直线分段数据到数组中
	// 先复制本特征中的分段
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line;
		GetSegment(i, Line);
		pLine[i] = Line;
	}

	// 再复制第二个特征中的分段
	for (int i = 0; i < (int)Feature2.m_Ranges.size(); i++)
	{
		CLine Line;
		Feature2.GetSegment(i, Line);
		pLine[m_Ranges.size() + i] = Line;
	}

	// 生成统一的多段线
	CMultiSegLine MultiSegLine;
	MultiSegLine.Create(pLine, nNum);
	MultiSegLine.m_nId = m_nId;

	// 重新生成本特征
	Create(MultiSegLine);

	// 释放临时数组空间
	delete []pLine;

	return true;
}

//
//   将此多段线与另外一条多段线(如果共线的话)进行合并。
//
bool CLineFeature::ColinearMerge(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines)
{
	// 先判断是否平行，如果不平行直接返回false
	if (!IsParallelTo(Feature2, fMaxAngDiff))
		return false;

	// 下面判断是否有相重叠的部分
	float lambda1, lambda2;

	// 计算线段Feature2的起点、终点到本直线(不是线段!)的距离
	float fDist1 = DistanceToPoint(false, Feature2.m_ptStart, &lambda1);
	float fDist2 = DistanceToPoint(false, Feature2.m_ptEnd, &lambda2);

	// 如果距离超限，则不共线
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return false;

	// 如果Feature2的起点位于本线段的起点侧之外
	if (lambda1 < 0)
	{
		// 如果Feature2的终点也位于本线段的起点侧之外，Feature2完全位于本线段以外
		if (lambda2 < 0)
		{
			int nNearPoint;
			float fDist;

			// 如果这两条线段的最近点之间的距离小于给定距离门限，可认为它们是相连的
			nNearPoint = Feature2.FindNearPoint(m_ptStart, &fDist);
			if (fDist < fMaxGapBetweenLines)
			{
				if (nNearPoint == 0)
					Create(Feature2.m_ptEnd, m_ptEnd);
				else
					Create(Feature2.m_ptStart, m_ptEnd);

				return true;
			}
			else
				return false;
		}
		// Feature2的终点位于本线段的终点侧之外
		else if (lambda2 > 1)
		{
			// 说明Feature2包含了本线段
			*this = Feature2;
			return true;
		}
		else
		{
			// 说明Feature2的起始点落在本线段(起始点方向)外面，而其结束点落在本线段上
			Create(Feature2.m_ptStart, m_ptEnd);
			return true;
		}
	}

	// 如果Feature2的起点位于本线段的终点侧之外
	else if (lambda1 > 1)
	{
		// 如果Feature2的终点也位于本线段的终点侧之外
		if (lambda2 > 1)
		{
			int nNearPoint;
			float fDist;

			// 如果这两条线段的最近点之间的距离小于给定距离门限，可认为它们是相连的
			nNearPoint = Feature2.FindNearPoint(m_ptEnd, &fDist);
			if (fDist < fMaxGapBetweenLines)
			{
				if (nNearPoint == 0)
					Create(m_ptStart, Feature2.m_ptEnd);
				else
					Create(m_ptStart, Feature2.m_ptStart);
				return true;
			}
			else
				return false;
		}
		// Feature2的终点位于本线段的起点侧之外
		else if (lambda2 < 0)
		{
			// 说明Feature2(逆向)包含了本线段(采用Feature2，但不应改变本线段原来的方向)
			Create(Feature2.m_ptEnd, Feature2.m_ptStart);
			return true;
		}
		else
		{
			// 说明Feature2的起点落在本线段(终止点方向)外面，而其结束点落在本线段上
			Create(m_ptStart, Feature2.m_ptStart);
			return true;
		}
	}

	// Feature2的起点位于本线段上
	else
	{
		// Feature2的终点位于本线段的起点侧之外
		if (lambda2 < 0)
		{
			Create(Feature2.m_ptEnd, m_ptEnd);
		}
		// 如果Feature2的终点位于本线段的终点侧之外
		else if (lambda2 > 1)
		{
			Create(m_ptStart, Feature2.m_ptEnd);
		}
		// 如果Feature2的终点位于本线段之内(不必改变本线段)
		else
		{
		}
		return true;
	}
	return true;
}

//
//   将特征进行平移。
//
void CLineFeature::Move(float fX, float fY)
{
	CPnt ptStart = m_ptStart;
	CPnt ptEnd = m_ptEnd;

	ptStart.Move(fX, fY);
	ptEnd.Move(fX, fY);

	// 只移动总线段的起点和终点，各分段范围不变
	CLine::Create(ptStart, ptEnd);
}

//
//   将特征进行旋转。
//
void CLineFeature::Rotate(CAngle ang, CPnt ptCenter)
{
	// 计算起点和终点经旋转后的位置
	m_ptStart.Rotate(ang.m_fRad, ptCenter);
	m_ptEnd.Rotate(ang.m_fRad, ptCenter);

	// 只改变总线段的起点和终点，各分段范围不变
	CLine::Create(m_ptStart, m_ptEnd);
}

//
//   以pstScanner为观测姿态，判断本直线特征是否可与给定的另一直线特征进行配准。
//
bool CLineFeature::RegisterWith(CLineFeature& another, CPosture& pstScanner,
	float fDistGate, float fAngGate)
{
	CPnt ptFoot1, ptFoot2;
	float fDist1 = DistanceToPoint(false, pstScanner, NULL, &ptFoot1);
	float fDist2 = another.DistanceToPoint(false, pstScanner, NULL, &ptFoot2);

	CAngle angDiff = AngleToUndirectionalLine(another);
	float fAngDiff = angDiff.m_fRad;
	if (fAngDiff > PI / 2)
		fAngDiff = PI - fAngDiff;

	// 比较两条从扫描点到垂足连线之间的夹角
	CLine ln1(pstScanner, ptFoot1);
	CLine ln2(pstScanner, ptFoot2);

	CAngle angDiff2 = ln1.AngleToLine(ln2);
	float fAngDiff2 = fabs(angDiff2.NormAngle2());

	// 距离、角度差均不超限时，视为配准成功
	if ((fabs(fDist2 - fDist1) < fDistGate) && (fAngDiff < fAngGate) && (fAngDiff2 < fAngGate))
		return true;
	else
		return false;
}

//
//   计算本直线特征到另一直线特征的变换代价。
//
float CLineFeature::TransformCostTo(const CLineFeature& another) const
{
	float fMidAngle;

	// 先取出两条直线的绝对倾角
	float fAng1 = StdSlantAngleRad();
	float fAng2 = another.StdSlantAngleRad();

	// 用绝对倾角大的减去较小的那个，得到两条线的倾角差
	float fAngCost = fAng1 - fAng2;
	if (fAng1 < fAng2)
	{
		fAngCost = -fAngCost;

		// 交换两个角，确保fAng1较大
		float fTemp = fAng1;
		fAng1 = fAng2;
		fAng2 = fTemp;
	}

	// 如果上面的倾角差大于PI/2，说明倾角较大的直线应采用反向角
	if (fAngCost > PI / 2)
	{
		fAngCost = PI - fAngCost;
		fMidAngle = (fAng1 + fAng2 - PI) / 2;
	}
	else
	{
		fMidAngle = (fAng1 + fAng2) / 2;
	}
	fAngCost *= ANGLE_COST_FACTOR;

	// 下面构建两个直线特征段的中间直线
	CPnt ptMid1 = GetMidpoint();
	CPnt ptMid2 = another.GetMidpoint();
	CLine lnTemp(ptMid1, ptMid2);
	CLineBase MidLine(lnTemp.GetMidpoint(), fMidAngle);

	// 计算两个直线特征到此中间直线的距离
	CPnt ptFoot1 = MidLine.GetProjectPoint(ptMid1);

	float fDistCost = ptMid1.DistanceTo(ptFoot1) * 2;

	// 整个变换代价为：角度代价+距离代价
	return fAngCost + fDistCost;
}

//
//   判断该直线特征能否与另外一个直线特征构成一个“角点”。
//
bool CLineFeature::MakeCorner(const CLineFeature& another, float fMinAngle, float fMaxAngle, float fMaxGap, CPnt& ptCorner)
{
	// 计算两条直线的夹角
	CAngle ang = AngleToUndirectionalLine(another);
	float fAng = ang.m_fRad;

	// 如果夹角大小超出范围，刚无法构成角点
	if (fAng < fMinAngle || fAng > fMaxAngle)
		return false;

	// 计算两条直线的交点
	float x, y;
	if (!Intersect(another, &x, &y, NULL, NULL, 0.001f))
		return false;

	CPnt pt(x, y);
	
	// 判断交点是否离第一条直线的某个端点很近
	if (pt.DistanceTo(m_ptStart) > fMaxGap && pt.DistanceTo(m_ptEnd) > fMaxGap)
		return false;

	// 判断交点是否离第二条直线的某个端点很近
	if (pt.DistanceTo(another.m_ptStart) > fMaxGap && pt.DistanceTo(another.m_ptEnd) > fMaxGap)
		return false;

	ptCorner = pt;
	return true;
}

//
//   进行坐标正变换。
//
void CLineFeature::Transform(const CFrame& frame)
{
	CMultiSegLine::Transform(frame);

	if (m_bCreateRef)
	{
		m_ptRef.Transform(frame);
		m_ptProjectFoot.Transform(frame);
	}
}

//
//   进行坐标逆变换。
//
void CLineFeature::InvTransform(const CFrame& frame)
{
	CMultiSegLine::InvTransform(frame);

	if (m_bCreateRef)
	{
		m_ptRef.InvTransform(frame);
		m_ptProjectFoot.InvTransform(frame);
	}
}

///////////////////////////////////////////////////////////////////////////////

#ifdef _MFC_VER
//
//   (在屏幕窗口上)测试指定的点是否在线段上。
//
bool CLineFeature::HitTest(CScreenReference& ScrnRef, CPoint point)
{
	CPoint pntStart = ScrnRef.GetWindowPoint(m_ptStart);
	CPoint pntEnd = ScrnRef.GetWindowPoint(m_ptEnd);

	CPnt ptStart((float)pntStart.x, (float)pntStart.y);
	CPnt ptEnd((float)pntEnd.x, (float)pntEnd.y);
	CPnt pt((float)point.x, (float)point.y);

	float fCheckDist = 5 / ScrnRef.m_fRatio;

	return PointHit(pt, fCheckDist);
}

//
//   绘制直线特征。
//
//   注意：程序中根据特征的扩展参数值确定是否需要显示：1. 直线特征的朝向; 2.特征的ID号
//
void CLineFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
	int nSize, int nShowDisabled, bool bShowSuggested)
{
	bool bShowActiveSide = m_nParam[0];
	bool bShowId = m_nParam[1];
	bool bShowRefPoint = m_nParam[2];

	// 画出直线特征
	if (!m_bSelected)
	{
		// 正常显示禁止项
		if (IsEnabled() || nShowDisabled == DISABLED_FEATURE_NORMAL)
			Draw(ScrnRef, pDC, crColor, nSize, nSize, false);

		// 淡色显示禁止项
		else if (nShowDisabled == DISABLED_FEATURE_UNDERTONE)
		{
			BYTE r = GetRValue(crColor) / 3;
			BYTE g = GetGValue(crColor) / 3;
			BYTE b = GetBValue(crColor) / 3;
			Draw(ScrnRef, pDC, RGB(r, g, b), nSize, nSize, false);
		}
	}
	else
		Draw(ScrnRef, pDC, crSelected, 3 * nSize, nSize, false);

	// 显示参考投影点
	if (bShowRefPoint && m_bCreateRef)
	{
		m_ptRef.Draw(ScrnRef, pDC, RGB(0, 0, 255), 3);
		m_ptProjectFoot.Draw(ScrnRef, pDC, RGB(255, 255, 0), 3);
	}

	// 显示直线特征朝向
	if (bShowActiveSide)
	{
		CAngle ang = SlantAngle();
		if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
			ang += PI / 2;
		else if (m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
			ang -= PI / 2;

		float fArrowLen = 8 / ScrnRef.m_fRatio;
		CLine lnArrow(GetMidpoint(), ang, fArrowLen);
		lnArrow.Draw(ScrnRef, pDC, RGB(64, 64, 64));
	}

	// 显示直线特征编号
	if (bShowId)
	{
		CString str;
		str.Format(_T("%d"), m_nId);

		pDC->SetTextColor(RGB(0, 255, 0));

		CPoint pnt = ScrnRef.GetWindowPoint(GetMidpoint());
		pDC->TextOut(pnt.x - 20, pnt.y - 20, str);
		pDC->SetTextColor(RGB(0, 0, 0));
	}
}
#endif

