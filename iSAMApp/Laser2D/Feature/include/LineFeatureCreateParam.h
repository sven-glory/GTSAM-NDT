#pragma once

#include "Geometry.h"

#define MAX_DIST_POINT_TO_POINT         0.08f   // 在1米远处垂直于激光束的面上，两个相邻点间的最大允许距离
#define MAX_DIST_POINT_TO_LINE          0.03f   // 属于直线段上的点到它所属直线的最大允许距离
#define MAX_SIGMA                       0.03f   // 直线段所允许的最大sigma值
#define MAX_SIGMA_RATIO                 0.2f    // 直线sigma值(平均误差距离)与直线长度的最大允许比值
#define MIN_POINTS_ON_LINE              30      // 1米远处垂直于激光束的直线段所含的最少点数
#define MIN_LINE_LEN                    0.04f//0.2f    // 直线段所允许的最小长度
#define MIN_SCAN_TO_LINE_ANGLE          TO_RADIAN(10)   // 扫描线同直线特征所成角度的最小值(单位：弧度)

#define MAX_LINE_MERGE_DIST             0.08f    // 两条平行直线的间距小于此值时可认为是一条直线，可以合并
#define MAX_LINE_MERGE_ANGLE            (5*PI/180)    // 两条直线的夹角小于此值时可认为是相互平行
#define MIN_CORNER_ANGLE                (PI/6)   // 形成角点的两条直线的最小夹角
#define MAX_HIDDEN_LINE_LEN             25.0f    // 如果两直线段的延长线超过此距离，生成的交点不采用


///////////////////////////////////////////////////////////////////////////////
// 从点云生成直线时所用的参数
struct CLineFeatureCreationParam
{
public:
	float fMaxDistPointToPoint;          // max distance for a cloud of points (grouping)
	float fMaxDistPointToLine;				 // 属于直线段上的点到它所属直线的最大允许距离
	float fMaxSigma;							 // max sigma value for a line
	int   nMinPointsOnLine;					 // 直线段所含的最少点数
	float fMinLineLength;					 // 直线段的最小长度
	float fMinScanToLineAngle;           // 扫描线与直线特征所成夹角的最小值(单位:弧度)
	float fMaxLineMergeDist;				 // 两条平行直线的间距小于此值时可认为是一条直线，可以合并
	float fMaxLineMergeAngle;				 // 两条直线的夹角小于此值时可认为是相互平行

public:
	CLineFeatureCreationParam()
	{
		fMaxDistPointToPoint = MAX_DIST_POINT_TO_POINT;
		fMaxDistPointToLine = MAX_DIST_POINT_TO_LINE;
		fMaxSigma = MAX_SIGMA;
		nMinPointsOnLine = MIN_POINTS_ON_LINE;
		fMinLineLength = MIN_LINE_LEN;
		fMinScanToLineAngle = MIN_SCAN_TO_LINE_ANGLE;
		fMaxLineMergeDist = MAX_LINE_MERGE_DIST;
		fMaxLineMergeAngle = MAX_LINE_MERGE_ANGLE;
	}

	// 从二进制文件装入参数
	bool Load(FILE* fp)
	{
		if (fread(&nMinPointsOnLine, sizeof(int), 1, fp) != 1)
			return false;

		float f[7];
		if (fread(f, sizeof(float), 7, fp) != 7)
			return false;

		fMaxDistPointToPoint = f[0];
		fMaxDistPointToLine = f[1];
		fMaxSigma = f[2];
		fMinLineLength = f[3];
		fMinScanToLineAngle = f[4];
		fMaxLineMergeDist = f[5];
		fMaxLineMergeAngle = f[6];

		return true;
	}

	// 将参数保存到二进制文件
	bool Save(FILE* fp)
	{
		if (fwrite(&nMinPointsOnLine, sizeof(int), 1, fp) != 1)
			return false;

		float f[7];
		f[0] = fMaxDistPointToPoint;
		f[1] = fMaxDistPointToLine;
		f[2] = fMaxSigma;
		f[3] = fMinLineLength;
		f[4] = fMinScanToLineAngle;
		f[5] = fMaxLineMergeDist;
		f[6] = fMaxLineMergeAngle;

		if (fwrite(f, sizeof(float), 7, fp) != 7)
			return false;

		return true;
	}
};
