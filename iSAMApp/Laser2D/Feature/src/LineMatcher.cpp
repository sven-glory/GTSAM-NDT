#include <stdafx.h>
#include "LineMatcher.h"
#include "DebugTrace.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//
//    给定激光头姿态和一条直线段，计算对应于线段两个端点的扫描角(假定以逆时针方向扫描)。
//
void FindScanAngles(CPosture& pstScanner, CLine& Line, CAngle& ang1, CAngle& ang2)
{
	// 取得激光头的位置和方向角度
	CPnt ptScanner = pstScanner.GetPntObject();
	CAngle angScanner = pstScanner.GetAngle();

	// 构造从激光头到线段起点的直线，并计算它与激光头姿态角的夹角
	CLine lnToStartPoint(ptScanner, Line.m_ptStart);
	CAngle angStartLine = lnToStartPoint.m_angSlant - angScanner;

	// 构造从激光头到线段终点的直线，并计算它与激光头姿态角的夹角
	CLine lnToEndPoint(ptScanner, Line.m_ptEnd);
	CAngle angEndLine = lnToEndPoint.m_angSlant - angScanner;

	// 假定激光头按照逆时针方向旋转，判断激光光线是先扫到直线的起点还是终点
	CAngle angDiff = angEndLine - angStartLine;

	// 如果上述角度差在I, II象限，说明先扫到m_ptStart
	if (angDiff.m_fRad < PI)
	{
		ang1 = angStartLine;
		ang2 = angEndLine;
	}

	// 否则说明先扫到m_ptEnd
	else
	{
		ang1 = angEndLine;
		ang2 = angStartLine;
	}
}

///////////////////////////////////////////////////////////////////////////////

//
//   设置参考特征图。
//
void CLineMatcher::SetRefFeatures(CLineFeatureSet* pLineFeatures)
{
	if (pLineFeatures != NULL)
		m_RefLineFeatures = *pLineFeatures;
	else
		m_RefLineFeatures.Clear();
}

//
//   设置当前特征图。
//
void CLineMatcher::SetCurFeatures(CPosture& pstOdometry, CLineFeatureSet* pLineFeatures)
{
	m_pstOdometry = pstOdometry;

	if (pLineFeatures != NULL)
		m_CurLineFeatures = *pLineFeatures;
	else
		m_CurLineFeatures.Clear();
}

//
//   进行快速匹配，找到从“局部特征”-->“全局特征”的坐标变换。
//   返回值：
//      > 0 : 匹配成功
//      < 0 : 错误代码(-2:特征不足; -3:方程矩阵异常)
//
int CLineMatcher::QuickMatch(CTransform& trans)
{
	// 仅当发现的匹配直线对不少于2对时，才视为配准成功
	if (QuickRegisterLines() < 2)
		return FM_FEATURES_NOT_ENOUGH;

	// 进行直线集匹配
	if (!m_LineMatchList.FindTransform())
		return FM_MATRIX_ERROR;

	trans = m_LineMatchList.GetTransform();

	return FM_OK;
}

//
//   进行本地局部匹配。
//
int CLineMatcher::LocalMatch(CTransform& trans)
{
//	return m_PointMatcher.LocalMatch(trans);
	return -1;
}

//
//   对所有的直线特征进行快速配准，并生成直线匹配表。
//
int CLineMatcher::QuickRegisterLines()
{
	// 先清除原来的匹配直线数据
	m_LineMatchList.clear();

	m_trans.Init(m_pstOdometry);

	DebugTrace(_T("Dumping RefLineFeatures:\n"));
	m_RefLineFeatures.Dump();

	DebugTrace(_T("\nDumping CurLineFeatures:\n"));
	m_CurLineFeatures.Dump();

	// 针对参数直线段集合，依次进行考察
	for (int i = 0; i < m_RefLineFeatures.GetCount(); i++)
	{
		CLineMatchPair Pair;
		CPnt ptFoot1;

		// 匹配计数(要求每个参考直线段最多可与一条当前直线匹配)
		int nMatchCount = 0;

		// 取得参考直线段
		CLineFeature& Line1 = m_RefLineFeatures.GetLineFeature(i);

		// 判断参考直线段的有效识别面是否与当前姿态相符
		CPnt& ptScanner = m_pstOdometry.GetPntObject();
		CPnt& ptStart = Line1.m_ptStart;
		CAngle ang(ptStart, ptScanner);
		CAngle angDiff = ang - Line1.SlantAngle();

		// 对于单面有效的情况，核对工作面是否相符
		if (Line1.m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
		{
			if (angDiff.Quadrant() > 2)          // 角度差大于180度，工作面不符
				continue;
		}
		else if (Line1.m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
		{
			if (angDiff.Quadrant() <= 2)         // 角度差小于180度，工作面不符
				continue;
		}

		Line1.m_nId = i;

		// 计算当前姿态在参考线段上的垂直投影点ptFoot1，并取得投影线长度
		float fDist1 = Line1.DistanceToPoint(false, ptScanner, NULL, &ptFoot1);

		// 计算从当前激将头姿态到参考直线的(逆时针)转角
		CAngle ang1 = Line1.SlantAngle();

		CAngle angScan11, angScan12;

		// 计算线段两个端点到激光头的连线与激光头当前姿态的夹角
		FindScanAngles(m_pstOdometry, Line1, angScan11, angScan12);

		// 得到下面Line2扫描角的允许范围
		angScan11 -= SIASUN_MATCHER_ANGLE_WINDOW;
		angScan12 += SIASUN_MATCHER_ANGLE_WINDOW;

		// 依次考察当前直线集合成员
		for (int j = 0; j < m_CurLineFeatures.GetCount(); j++)
		{
			CPnt ptOrigin(0, 0);
			CPnt ptFoot2;

			// 取得当前直线段
			CLineFeature& line2 = m_CurLineFeatures.GetLineFeature(j);

			CPnt ptStart = line2.m_ptStart;
			CPnt ptEnd = line2.m_ptEnd;

			CLine Line2(ptStart, ptEnd);
			Line2.m_nId = j;

			float fLen2 = Line2.Length();

			// 计算当前姿态在参考线段上的垂直投影点ptFoot2，并取得投影线长度
			float fDist2 = Line2.DistanceToPoint(false, m_pstOdometry, NULL, &ptFoot2);

			// 计算从当前激头姿态到当前直线的(逆时针)转角
			CAngle ang2 = Line2.SlantAngle();

			CAngle angScan21, angScan22;

			// 计算线段两个端点到激光头的连线与激光头当前姿态的夹角
			FindScanAngles(m_pstOdometry, Line2, angScan21, angScan22);

			// 距离变化量不超过400，角度变化不超过15度，且两个端点所在扫描角在允许范围之内
			if ((fabs(fDist2 - fDist1) < 0.4f) &&
				(ang1.GetDifference(ang2) < SIASUN_MATCHER_ANGLE_WINDOW) /*&&
				angScan21.InRange(angScan11, angScan12) &&
				angScan22.InRange(angScan11, angScan12)*/)
			{
				Pair.Create(j, i, Line2, Line1, /*m_pstOdometry*/CPosture(0, 0, 0), ang1 - ang2);

				m_LineMatchList.Add(Pair);
			}
		}
	}

#ifdef SHOW_DEBUG_MSG
	DebugTrace(_T("pstOdometry: x=%.4f, y=%.4f, t=%.4f\n"), m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);

	DebugTrace(_T("Dumping Line match List 1:\n"));
	m_LineMatchList.Dump();
#endif

	// 最后对匹配表进行过滤处理，去除那些不合理的、或者是多重对应的项
	m_LineMatchList.Filter();

#ifdef SHOW_DEBUG_MSG
	DebugTrace(_T("Dumping Line match List 2:\n"));
	m_LineMatchList.Dump();
#endif

	// 判断是否有足够进行定位的直线匹配对
	int nCount = 0;
	for (int i = 0; i < m_LineMatchList.GetCount() - 1; i++)
	{
		CLine& Line1 = m_LineMatchList.at(i).m_lnLocal;
		for (int j = i + 1; j < m_LineMatchList.GetCount(); j++)
		{
			CLine& Line2 = m_LineMatchList.at(j).m_lnLocal;
			CAngle angDiff = Line1.AngleToUndirectionalLine(Line2);

			// 如果两直线夹角大于30度，则可用于定位
			if (angDiff > CAngle(30.0f, IN_DEGREE))
				nCount++;
		}
	}

	// 返回可用的匹配直线对数量
	return nCount;
}

bool CLineMatcher::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\n", &m_pstOdometry.x, &m_pstOdometry.y, &m_pstOdometry.fThita);

	m_RefLineFeatures.clear();
	m_CurLineFeatures.clear();
	
	m_RefLineFeatures.LoadText(fp);
	m_CurLineFeatures.LoadText(fp);
	return true;
}

bool CLineMatcher::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\n", m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);
	fprintf(fp, "0\n");
	m_CurLineFeatures.SaveText(fp);
	return true;
}
