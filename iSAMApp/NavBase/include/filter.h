#ifndef FILTER_H
#define FILTER_H

#include "scan.h"
#include "LineFeatureSet.h"

void ScanMedianFilter(CScan *scan);
/* filter that generates median from scan */

void ScanReductionFilter(CScan *scan, float reduce_radius);
/* 
** Filter that reduces number of scan points by grouping
** points that lie in a circle with radius reduce_radius.
*/

void ScanAngleReductionFilter(CScan *scan, float da);
/* 
** Filter that reduces number of scan points by grouping
** points that lie in the same angular distance da.
*/

void ScanSegmentFilter(CScan *scan, short numSegments);
/*
** filter that reduces number of points
** by replacing points lying in the same segement
** by the scan point which is closest to the range finder.
** Each segment is defined by a cone with a constant angle.
*/

void ScanLineFilter(CScan *scan, int mode);
/* filter that removes scan points which do not ly on a line */

void ScanLineFilterFast(CScan *scan);
/* like ScanLineFilter but assumes m_nLineID is already set correctly */

void ScanNotLineFilterFast(CScan *scan);
/* removes all points on a line, assumes m_nLineID is set correctly */

void ScanNotLineButCloseFilterFast(CScan *scan, float maxDist);
/* removes all points on a line and those that are far from a line,
   assumes m_nLineID is set correctly */

void ScanClusterFilter(CScan *scan, float radius, int minNum);
/* removes all points whose neighbors within radius make a cluster
   of less than minNum points.  This removes single points */

float ScanCommonPointsFilter(CScan *scan1, CScan *scan2,
    float max_dist);
/*
** filter that removes scan points of scan1 which have no correspondence
** partner in the other scan.  The correspondence partner is chosen
** by searching for a point within max_dist distance.
*/

void ScanCommonPointsFilter2(CScan *scan1, CScan *scan2, 
    float max_dist, float *reduced1, float *reduced2);
/*
** Applies ScanCommonPointsFilter to both scans.
*/

float ScanProjectionFilter(CScan *scan, float scanX, float scanY,
    CScan *frame, float maxred);
/*
** filter that removes scan points which are not visible
** from another location.  If frame != NULL then points which lie
** behind the frame scan are removed, too.
** In maxred you specify a maximum reduction of scan points.
** If more points then maxred * num_points have to be removed
** then a reduction value larger than maxred is returned.
** The scan might not be changed in this case.
** Returns percentage [0 .. 1] of points removed.
*/

void ScanProjectionFilter2(CScan *scan1, CScan *scan2,
    float *rem1, float *rem2, float maxred);
/*
** filter that removes scan points which are not visible
** from each others position.  In maxred you specify a 
** maximum reduction of scan points.
** Returns number of points removed in each scan in percent
** in rem1 and rem2 respectively.
** If more points then maxred * num_points have to be removed
** in one scan then a reduction value larger than maxred is returned
** for this scan.  The other value might be 0 in case the other
** scan has not been touched yet.
** A scan might not be changed in case more points than maxred 
** have to be removed.
*/

void ScanAntennaFilter(CScan *scan, 
    float blindStartA, float blindEndA);
/*
** Filters out range data that lies in a given angular range.
** This filter is used for the SICK laser range finder
** which is mounted on a Pioneer 1 robot where the Antenna
** of the radio modem is in the scanners sight.
*/

void ScanMaxRangeFilter(CScan *scan, float maxRange);
/*
** Removes all points with range larger or equal to maxRange.
*/ 

void ScanRemoveOutliersFilter(CScan *scan);
/*
** finds and removes outliers.
*/

#endif

