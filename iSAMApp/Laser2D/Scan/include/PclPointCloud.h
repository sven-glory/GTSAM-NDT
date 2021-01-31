#ifndef __Scan2PclCloud
#define __CPclPointCloud

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Scan.h"

typedef pcl::PointCloud<pcl::PointXYZ> CPclPointCloud;

CPclPointCloud Scan2PclCloud(const CScan& Scan);
CScan PclCloud2Scan(const CPclPointCloud& cloud);

#endif
