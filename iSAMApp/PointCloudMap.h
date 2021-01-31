#ifndef _POINT_CLOUD_MAP_
#define _POINT_CLOUD_MAP_

#include "stdafx.h"

#include "Geometry/include/geometry.h"
#include "PclPointCloud.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>

#define _RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

using namespace gtsam;
using namespace std;

class CPointCloudMap
{
public:
	vector<CPosture> poses;
	vector<CPclPointCloud> clouds;

	Eigen::Affine3d odompos;

	unsigned int    input_cloud_count;
	unsigned int    input_ndtcloud_count;

public:
	CPointCloudMap();

	bool AddCloudandPose(CPclPointCloud cloud, Eigen::Affine3d odometry, bool loop = false);

	void AddCloudandAbsPose(CPclPointCloud cloud, CPosture abspose, bool loop);
	void CorrectPoses(Values& cur_estimate);

	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, int nPointSize);
};

#endif
