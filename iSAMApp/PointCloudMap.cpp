

/*
author:zjj
data:2020.10.29
modify:
*/

#include "PointCloudMap.h"
#include "AffinePosture.h"
using namespace gtsam;
using namespace std;

CPointCloudMap::CPointCloudMap()
{
	poses.clear();
	clouds.clear();
	input_cloud_count = 0;//控制往缓存中添加cloud的数量
	input_ndtcloud_count = 0;
}

bool CPointCloudMap::AddCloudandPose(CPclPointCloud cloud, Eigen::Affine3d odometry, bool loop)
{
	if (poses.empty())
	{
		CPosture pos = AffineToPosture(odometry);
		poses.push_back(pos);
		clouds.push_back(cloud);
		odompos = odometry;
		input_cloud_count++;
		return true;
	}
	else
	{
		input_cloud_count++;
		/*int size = poses.size();
		CPosture pos = poses[size - 1];
		Eigen::Affine3d posa = PostureToAffine(pos);*/
#if 1		
		odompos = odompos * odometry;
		CPosture pos = AffineToPosture(odompos);
#else
		CPosture pos = AffineToPosture(odometry);
#endif
		if (input_cloud_count % 50 == 0 || loop)
		{
			poses.push_back(pos);
			clouds.push_back(cloud);
			return true;
		}

		return false;
	}
}

void CPointCloudMap::AddCloudandAbsPose(CPclPointCloud cloud, CPosture abspose, bool loop)
{
	//CPosture pos = AffineToPosture(abspose);
	poses.push_back(abspose);
	clouds.push_back(cloud);
	input_ndtcloud_count++;
}

void CPointCloudMap::CorrectPoses(Values& cur_estimate)
{
	int num_poses = cur_estimate.size();
	for (int i = 0; i < num_poses; ++i)
	{
		poses[i].x = cur_estimate.at<gtsam::Pose2>(i).x();
		poses[i].y = cur_estimate.at<gtsam::Pose2>(i).y();
		poses[i].fThita = cur_estimate.at<gtsam::Pose2>(i).theta();
	}
}

void CPointCloudMap::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, int nPointSize)
{
	CPosture pos;
	//pos = poses[0];
	for (int i = 0; i < clouds.size(); i++)
	{
		/*if (i % 100 == 0 && i > 0)
		{
		int x = 0;
		}*/
		/*if (i % 50 != 0)
			continue;*/

		pos = poses[i];
		CTransform trans(pos.GetPntObject(), pos.GetAngle());

		CPen pen(PS_SOLID, 1, crColorPoint);
		CPen* pOldPen = pDC->SelectObject(&pen);

		CBrush Brush(_RGB(0, 0, 0));
		CBrush* pOldBrush = pDC->SelectObject(&Brush);

		CScan scloud = PclCloud2Scan(clouds[i]);

		for (int j = 0; j < scloud.m_nCount; j++)
		{	
			/*if (j % 5 != 0)
				continue;*/

			CPnt pt;

			// 如果需要在世界坐标系中显示，现在需要进行坐标变换
			pt = trans.GetWorldPoint(scloud.m_pPoints[j]);

			CPoint pnt = ScrnRef.GetWindowPoint(pt);

			CRect r(pnt.x - nPointSize, pnt.y - nPointSize, pnt.x + nPointSize, pnt.y + nPointSize);
			pDC->Ellipse(&r);
		}
		pDC->SelectObject(pOldPen);
		pDC->SelectObject(pOldBrush);
	}
}