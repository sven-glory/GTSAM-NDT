#include <stdafx.h>
#include "PclPointCloud.h"

///////////////////////////////////////////////////////////////////////////////

CPclPointCloud Scan2PclCloud(const CScan& Scan)
{
	CPclPointCloud cloud;

	for (int i = 0; i < Scan.m_nCount; i++)
	{
		pcl::PointXYZ pt;
		pt.x = Scan.m_pPoints[i].x;
		pt.y = Scan.m_pPoints[i].y;
		pt.z = 0;

		cloud.points.push_back(pt);
	}
	return cloud;
}

CScan PclCloud2Scan(const CPclPointCloud& cloud)
{
	CScan Scan(cloud.points.size());

	for (int i = 0; i < cloud.points.size(); i++)
	{
		const pcl::PointXYZ& pt = cloud.points[i];
		Scan.m_pPoints[i].x = pt.x;
		Scan.m_pPoints[i].y = pt.y;
	}
	return Scan;
}
