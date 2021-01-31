#include "stdafx.h"

#include "misc.h"
#include "scan.h"
#include "LineFeatureSet.h"
#include "filter.h"
#include "config.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


/*
** ScanMedianFilter
**
** filter that generates median from scan
*/

#define MEDIAN_NUMPOINTS 5

static int mediancmp(const void *l1, const void *l2)
{
    if(*(float *)l1 < *(float *)l2)
        return(-1);
    if(*(float *)l1 > *(float *)l2)
        return(1);
    return(0);
}

void ScanMedianFilter(CScan *scan)
{
    if(scan && scan->m_nCount > 0)
    {
    CPosture scanpos;
        CScanPoint *sp = scan->m_pPoints;
        long num = scan->m_nCount, i, j;
        float d[5];
        float x, y;

        scan->GetScannerAbsPos(scanpos);

        for(i=0; i < num; i++)
        {
            /*
            ** get median.
            ** There are algorithms whih are more efficient than
            ** just using qsort and taking the middle element.
            ** But for only a small number of elements that shouldn't matter.
            */

            for(j=0; j < MEDIAN_NUMPOINTS; j++)
                d[j] = sp[(i + j-MEDIAN_NUMPOINTS/2 + num) % num].r;
            qsort(d, MEDIAN_NUMPOINTS, sizeof(float), mediancmp);

            /* calculate new point */
            x = sp[i].x - scanpos.x;
            y = sp[i].y - scanpos.y;

            sp[i].x = scanpos.x + x * d[MEDIAN_NUMPOINTS/2] / sp[i].r;
            sp[i].y = scanpos.y + y * d[MEDIAN_NUMPOINTS/2] / sp[i].r;
        }

        /*
        ** Update distance information
        */
        for(i=0; i < num; i++)
        {
            x = sp[i].x - scanpos.x;
            y = sp[i].y - scanpos.y;

            sp[i].r = sqrt(x*x + y*y);
        }
    }
}

/*
** ScanReductionFilter
**
** filter that reduces number of points
** by replacing clouds of points by their center of gravity.
*/

void ScanReductionFilter(CScan *scan, float reduce_radius)
{
	if (scan && scan->m_nCount > 0)
	{
		CScanPoint *sp = scan->m_pPoints;
		long num, i, j = 0;
		float x, y;
		CPosture scanpos;

		scan->GetScannerAbsPos(scanpos);

		x = sp[0].x;
		y = sp[0].y;
		num = 1;

		for (i = 1; i < scan->m_nCount; i++)
		{
			if (sp[i-num].DistanceTo(sp[i]) < 2.0 * reduce_radius)
			{
				 x += sp[i].x;
				 y += sp[i].y;
				 num++;
			}
			else
			{
				x /= (float)num;
				y /= (float)num;
				sp[j].x = x;
				sp[j].y = y;
				x -= scanpos.x;
				y -= scanpos.y;

				sp[j].a = NormAngle(atan2(y, x) - scanpos.fThita);
				sp[j].r = _hypot(x, y);
				sp[j].m_nWeight = num;

				sp[j].m_nLineID = sp[i-num].m_nLineID;
				j++;

				x = sp[i].x;
				y = sp[i].y;
				num = 1;
			}
		}

		x /= (float)num;
		y /= (float)num;
		sp[j].x = x;
		sp[j].y = y;
		x -= scanpos.x;
		y -= scanpos.y;
		sp[j].a = NormAngle(atan2(y, x) - scanpos.fThita);
		sp[j].r = _hypot(x, y);
		sp[j].m_nWeight = num;
		sp[j].m_nLineID = sp[i-num].m_nLineID;

		j++;

		scan->m_nCount = j;
	}
}

/*
** ScanAngleReductionFilter
*/

#define MAX_ANGLES 1000

struct da
{
    float d, a;
};

int dacmp(const void *p1, const void *p2)
{
    const struct da *da1 = (struct da*)p1, *da2 = (struct da*)p2;

    if(da1->d < da2->d)
    return(-1);
    else if(da1->d > da2->d)
    return(1);
    return(0);
}

#define CLUSTER_RADIUS 50.0

static void chooseangledist(struct da *das, long num, float *a, float *d)
{
    long i, max_cluster_start = 0, max_cluster_num = 0, start;
    float d_start;

    /*
    ** Cluster the set of ranges and use the median of the
    ** largest cluster.
    */
    qsort(das, num, sizeof(struct da), dacmp);
    start = 0;
    d_start = das[start].d;
    for(i=1; i < num; i++)
    {
    if(das[i].d - d_start > 2 * CLUSTER_RADIUS)
    {
        if(i - start > max_cluster_num)
        {
        max_cluster_start = start;
        max_cluster_num = i - start;
        }
        start = i;
        d_start = das[start].d;
    }
    }
    if(i - start > max_cluster_num) 
    {    
    max_cluster_start = start; 
    max_cluster_num = i - start; 
    } 

    *a = das[max_cluster_start + max_cluster_num/2].a;
    *d = das[max_cluster_start + max_cluster_num/2].d;
}

void ScanAngleReductionFilter(CScan *scan, float da)
{
    if(scan && scan->m_nCount > 0)
    {
    static struct da das[MAX_ANGLES];
        CScanPoint *sp = scan->m_pPoints;
        long num, i, j = 0;
    CPosture scanpos;
        float d, a, dda;

        scan->GetScannerAbsPos(scanpos);

    das[0].a = sp[0].a;
    das[0].d = sp[0].r;
    num = 1;
        for(i=1; i < scan->m_nCount; i++)
        {
        dda = sp[i-num].AngleDistanceTo(sp[i]);

        if(dda >= 0.0 && dda < da)
            {
        if(num >= MAX_ANGLES)
            fprintf(stderr, "ScanAngleReductionFilter: "
            "Warning! Too many readings in angle intervall.\n"
            "Increase MAX_ANGLES (= %d) value!\n", MAX_ANGLES);
        else
        {
            das[num].a = sp[i].a;
            das[num].d = sp[i].r;
                    num++;
        }
            }
            else
            {
        chooseangledist(das, num, &a, &d);
        sp[j].a = a;
        sp[j].r = d;
        a = scanpos.fThita + a;
                sp[j].x = scanpos.x + d*cos(a);
                sp[j].y = scanpos.y + d*sin(a);
                sp[j].m_nWeight = num;
                j++;

        das[0].a = sp[i].a;
        das[0].d = sp[i].r;
                num = 1;
            }
        }
    chooseangledist(das, num, &a, &d);
    sp[j].a = a;
    sp[j].r = d;
    a = scanpos.fThita +  a;
    sp[j].x = scanpos.x + d*cos(a);
    sp[j].y = scanpos.y + d*sin(a);
    sp[j].m_nWeight = num;
    j++;

        scan->m_nCount = j;
    }
}

/*
** ScanSegmentFilter
**
** filter that reduces number of points
** by replacing points lying in the same segement
** by the scan point which is closest to the range finder.
** Each segment is defined by a cone with a constant angle.
*/

void ScanSegmentFilter(CScan *scan, short numSegments)
{ 
    CScanPoint *sp;
    float step_a, seg_a;
    long i, j, min;

    if(scan == NULL || scan->m_nCount <= 0)
    return;                 /* nothing to do */
    if(numSegments <= 0)
    {
    scan->m_nCount = 0;           /* clear scan */
    return;
    }
    if(numSegments >= scan->m_nCount)     /* nothing to do */
    return;

    step_a = NormAngle(scan->m_fEndAng - scan->m_fStartAng);
    if(step_a <= 0.0)
    step_a += 2.0*PI;
    step_a /= (float)numSegments;

    sp = scan->m_pPoints;
    min = -1;
    seg_a = sp[0].a;
    j = 0;
    for(i=1; i < scan->m_nCount; i++)
    {
    if(NormAngle(sp[i].a - seg_a) <= step_a)
    {
        if(min < 0 || sp[i].r < sp[min].r)
        min = i;
    }
    else
    {
        if(min >= 0)
        sp[j++] = sp[min];
        min = -1;
        seg_a += step_a;
    }
    }
    if(min >= 0)
    sp[j++] = sp[min];
    scan->m_nCount = j;
}

/*
** ScanLineFilter
**
** filter that removes scan points which do not lie on a line
*/

void ScanLineFilter(CScan *scan, int mode)
{
	CLineFeatureSet *ls = new CLineFeatureSet(*scan);
	if (ls != NULL)
	{
		ScanLineFilterFast(scan);
		ls->Clear();
	}
}

void ScanLineFilterFast(CScan *scan)
{
	if (scan) 
	{
		CScanPoint *sp = scan->m_pPoints;

		int i, j = 0;
		for (i = 0; i < scan->m_nCount; i++) 
		{
		  if (sp[i].m_nLineID >= 0) 
			  sp[j++] = sp[i];
		}
		scan->m_nCount = j;
	}
}

void ScanNotLineFilterFast(CScan *scan)
{
    if (scan) {
    CScanPoint *sp = scan->m_pPoints;
    int i, j = 0;

    for (i=0; i < scan->m_nCount; i++) {
        if (sp[i].m_nLineID < 0) {
        sp[j++] = sp[i];
        }
    }
    scan->m_nCount = j;
    }
}

void ScanNotLineButCloseFilterFast(CScan *scan, float maxDist)
{
    if (scan) {
    float maxDist2 = maxDist * maxDist;
    CScanPoint *sp = scan->m_pPoints;
    int start=0, i=0, n = 0;

    while (i < scan->m_nCount) {
        for (i = start; i < scan->m_nCount; i++) {
        if (sp[i].m_nLineID >= 0) {
            float px = sp[i].x, py = sp[i].y;
            int j;

            for (j = i-1; j >= start; j--) {
            float dx = sp[j].x - px, dy = sp[j].y - py;
            float d2 = dx*dx + dy*dy;
            
            if (d2 > maxDist2) {
                break;
            }
            }
            for (++j; j < i; j++) {
            sp[n++] = sp[j];
            }
            break;
        }
        }
        for (++i; i < scan->m_nCount; i++) {
        if (sp[i].m_nLineID < 0) {
            float px = sp[i-1].x, py = sp[i-1].y;
            int j;

            for (j=i; j < scan->m_nCount && sp[j].m_nLineID < 0; j++) {
            float dx = sp[j].x - px, dy = sp[j].y - py;
                float d2 = dx*dx + dy*dy;
            
                if (d2 > maxDist2) {
                    break;
            }
            sp[n++] = sp[j];
            }
            start = j;
            break;
        }
        }
    }
    scan->m_nCount = n;
    }
}

void ScanClusterFilter(CScan *scan, float radius, int minNum)
{
    if (scan) {
    float maxDist2 = 4.0 * radius * radius;
        CScanPoint *sp = scan->m_pPoints;
    long i = 0, n = 0;

    while (i < scan->m_nCount) {
        float px = sp[i].x;
        float py = sp[i].y;
        long start = i;
        bool ok = FALSE;

        for (++i; i < scan->m_nCount; i++) {
        float x = sp[i].x, y = sp[i].y;
        float dx = x - px, dy = y - py;
        float d2 = dx*dx + dy*dy;

        if (d2 > maxDist2) {
            if (!ok) {
            i = start + 1;
            }
            break;
        }
        if (ok) {
            sp[n++] = sp[i];
            px = x;
            py = y;
        } else if (i + 1 - start >= minNum) {
            while (start <= i) {
            sp[n++] = sp[start++];
            }
            ok = TRUE;
            px = x;
            py = y;
        }
        }
    }

    scan->m_nCount = n;
    }
}

/*
** ScanCommonPointsFilter
**
** filter that removes scan points which have no correspondence
** partner in the other scan.
*/

float ScanCommonPointsFilter(CScan *scan1, CScan *scan2,
    float max_dist)
{
    long i, j, k=0;
    CScanPoint *sp1, *sp2;

    if(scan1 == NULL || scan2 == NULL)
    return(0.0);

    sp1 = scan1->m_pPoints;
    sp2 = scan2->m_pPoints;
    for(i=0; i < scan1->m_nCount; i++)
    for(j=0; j < scan2->m_nCount; j++)
        if(sp1[i].DistanceTo(sp2[j]) < max_dist)
        {
        sp1[k++] = sp1[i];
        break;
        }
    scan1->m_nCount = k;

    return((float)(i - k) / (float)i);
}
    
void ScanCommonPointsFilter2(CScan *scan1, CScan *scan2, 
    float max_dist, float *reduced1, float *reduced2)
{
    float reduced;

    if(scan1 == NULL || scan2 == NULL)
    return;

    reduced = ScanCommonPointsFilter(scan1, scan2, max_dist);
    if(reduced1 != NULL)
    *reduced1 = reduced;
    reduced = ScanCommonPointsFilter(scan2, scan1, max_dist);
    if(reduced2 != NULL)
    *reduced2 = reduced;
}

/*
** ScanProjectionFilter
**
** filter that removes scan points which are not visible
** from another location
*/

enum { VISIBLE=0, WRONG_DIRECTION, OCCLUDED, NOT_IN_INTERSECTION };

#define REDUCE_RADIUS 100.0
#define PROJECT_A_TOLERANCE DEG2RAD(2.0)
#define OCCLUDE_A_TOLERANCE DEG2RAD(2.0)
#define MIN_PPDIST 100.0    
#define MAX_PPDIST 500.0
#define MAX_DIST_OCCLUDED 1000.0

#define ANGLE_TOLERANCE DEG2RAD(10.0)

/*
** Tolerances since the scan points are recorded under noise
** conditions.
*/

#define PROJECT_WINDOW 3

struct project_help
{
    float x, y;        /* scan point's position */
    float a;                   /* scan point's angle */
    float d;                   /* distance to new position */
    int visible;                /* scan point's visible state */
    long start, end;        /* index range for this point */
    struct
    {
        long start, end;       /* first and last project_help index */
    } ar;                       /* angle range */
};

static float distancephtoph(struct project_help *ph1, struct project_help *ph2)
{
    float dx = ph1->x - ph2->x;
    float dy = ph1->y - ph2->y;
    
    return(sqrt(dx*dx + dy*dy));
}

static struct project_help *build_project_help(CScan *scan,
    float scanPosX, float scanPosY, long *pnum_phs)
{
    CScanPoint *sp = scan->m_pPoints;
    struct project_help *ph = NULL;
    float xsum, ysum, xstart, ystart, x, y;
    long num, i, num_phs = 0;
    const float maxd = 2.0 * REDUCE_RADIUS, maxd2 = maxd * maxd;

    ph = (project_help*)SSmalloc(scan->m_nCount * sizeof(struct project_help));
    if(ph == NULL)
    return(NULL);

    /*
    ** internal reduction filter
    */
    xstart = xsum = sp[0].x;
    ystart = ysum = sp[0].y;
    num = 1;
    for (i = 1; i < scan->m_nCount; i++)
    {
    float x = sp[i].x;
    float y = sp[i].y;
    float dx = xstart - x;
    float dy = ystart - y;
    float d2 = dx*dx + dy*dy;

    if (d2 < maxd2)
    {
        xsum += x;
        ysum += y;
        num++;
    }
    else
    {
        xstart = x;
        ystart = y;
        x = xsum / (float)num;
        y = ysum / (float)num;
        ph[num_phs].x = x;
            ph[num_phs].y = y;
            x -= scanPosX;
            y -= scanPosY;
        ph[num_phs].a = atan2(y, x);
            ph[num_phs].d = _hypot(x, y);
        ph[num_phs].visible = VISIBLE;
        ph[num_phs].start = i-num;
        ph[num_phs].end = i-1;
        num_phs++;

        xsum = xstart;
        ysum = ystart;
        num = 1;
    }
    }
    x = xsum / (float)num;
    y = ysum / (float)num;
    ph[num_phs].x = x;
    ph[num_phs].y = y;
    x -= scanPosX;
    y -= scanPosY;
    ph[num_phs].a = atan2(y, x);
    ph[num_phs].d = _hypot(x, y);
    ph[num_phs].visible = VISIBLE;
    ph[num_phs].start = i-num;
    ph[num_phs].end = i-1;
    num_phs++;

    *pnum_phs = num_phs;
    return ph;
}

static long mark_wrong_direction(struct project_help *ph, long num_phs)
{
    long i, j1, j2, k1=0, k2=0, k, visible, rem = 0;
    float dist=0.0, da;

    /*
    ** Remove points with wrong angular order.
    */
    for(i=0; i < num_phs; i++)
    {
        /* search neigbour points with distance >= MIN_PPDIST */
        for(j1 = 1; j1 < num_phs; j1++)
        {
            k1 = (i+j1) % num_phs;
        dist = distancephtoph(&ph[i], &ph[k1]);
            if(dist >= MIN_PPDIST)
                break;
        }
        if(j1 >= num_phs || dist > MAX_PPDIST) /* next point found? */
            continue;                           /* no remove, goto next point */

        for(j2 = 1; j2 < num_phs; j2++)
        {
            k2 = (i + num_phs - j2) % num_phs;
        dist = distancephtoph(&ph[i], &ph[k2]);
            if(dist >= MIN_PPDIST)
                break;
        }
        if(j2 >= num_phs || dist > MAX_PPDIST) /* next point found? */
            continue;                           /* no remove, goto next point */

        da = NormAngle(ph[k1].a - ph[k2].a);
        if(da < 0.0)
            ph[i].visible = WRONG_DIRECTION;              /* point invisible */
    }

    /*
    ** Make visibility function more smooth.
    */
    for(i=0; i < num_phs; i++)
    {
    visible = 0;
    for(k=0; k < PROJECT_WINDOW; k++)
        visible += (ph[(i + k) % num_phs].visible == VISIBLE)? 1 : 0;

        k = (i + PROJECT_WINDOW / 2) % num_phs;
    if(visible > PROJECT_WINDOW / 2)
        ph[k].visible = VISIBLE;
    else
    {
        ph[k].visible = WRONG_DIRECTION;
        rem++;
    }
    }

    return rem;
}

static long build_angular_groups(struct project_help *ph, long num_phs)
{
    long start, visible, i;
    long num_entries = 0;

    /*
    ** Build map of angle ranges by grouping
    */
    start = 0;
    visible = ph[0].visible;
    for(i = start + 1; i < num_phs; i++)
    {
        bool newgroup = FALSE;
    int close_group = 0;
    float da;

        if(ph[i].visible != visible)            /* change of vis. state? */
            newgroup = TRUE;
        else if(distancephtoph(&ph[i-1], &ph[i]) > MAX_PPDIST)
            newgroup = TRUE;
        else
        {
            da = NormAngle(ph[i].a - ph[start].a);
            if(visible == WRONG_DIRECTION)  /* wrong order? */
                da = -da;
            if(da < -PROJECT_A_TOLERANCE || da > PI/2.0)
        {
                newgroup = TRUE;
        close_group = 1;
        }
        }

        if(newgroup)
        {
            ph[num_entries].ar.start = start;
            ph[num_entries].ar.end = (i-1 + close_group) % num_phs;
            num_entries++;

            start = i;
            visible = ph[i].visible;
        }
    }
    ph[num_entries].ar.start = start;
    ph[num_entries].ar.end = num_phs - 1;
    if(ph[0].visible == ph[num_phs-1].visible && 
    distancephtoph(&ph[0], &ph[num_phs-1]) <= MAX_PPDIST)
        ph[num_entries].ar.end = 0;
    num_entries++;

#if 0
    for(i=0; i < num_entries; i++)
    {
        printf("ar %d: visible: %d \t %f deg - %f deg\n", i,
            ph[ph[i].ar.start].visible,
            ph[ph[i].ar.start].a * 180.0 / PI,
            ph[ph[i].ar.end].a * 180.0 / PI);
    }
#endif

    return(num_entries);
}

static long mark_occluded_help(
    struct project_help *ph1, long num_phs, long i,
    struct project_help *ph2, long num_phs2, long j, 
    float min_dist)
{
    long i_start = ph1[i].ar.start, i_end = ph1[i].ar.end;
    float ai_start = ph1[i_start].a, ai_end = ph1[i_end].a;
    long j_start = ph2[j].ar.start, j_end = ph2[j].ar.end;
    float aj_start = ph2[j_start].a, aj_end = ph2[j_end].a;
    float da1, da2;
    bool finish = FALSE;
    long k, l = j_start, ladd = 1, lend = j_end, lnew, rem = 0;
    bool lexchange = FALSE;

    if(NormAngle(ai_end - ai_start) < 0.0)
    {
    ai_end = ai_start;
    ai_start = ph1[i_end].a;
    lexchange = !lexchange;
    }

    if(NormAngle(aj_end - aj_start) < 0.0)
    {
    aj_end = aj_start;
    aj_start = ph2[j_end].a;
    lexchange = !lexchange;
    }

    if(lexchange)
    {
    l = j_end;
    ladd = -1;
    lend = j_start;
    }

    /*
    ** Check for overlap
    */
    aj_start -= OCCLUDE_A_TOLERANCE;
    aj_end += OCCLUDE_A_TOLERANCE;
    da1 = NormAngle(aj_end - ai_start);
    da2 = NormAngle(aj_start - ai_end);

    if(da1 < 0.0 || da2 > 0.0)
    return 0;               /* no overlap */

    /*
    ** We have overlap, now check all points
    ** of group i
    */
    for(k = i_start; !finish; k++)
    {
    if(k == num_phs)
        k = 0;
    if(k == i_end)
        finish = TRUE;      /* last point */
    if (ph1[k].visible != VISIBLE)
        continue;           /* already checked */
    da1 = NormAngle(ph1[k].a - aj_start);
    da2 = NormAngle(ph1[k].a - aj_end);
    if(da1 < 0.0 || da2 > 0.0)
        continue;                   /* no overlap */

    /* check distance */
    da1 = fabs(NormAngle(ph1[k].a - ph2[l].a));
    while(l != lend)
    {
        lnew = (l + ladd + num_phs2) % num_phs2;
        da2 = fabs(NormAngle(ph1[k].a - ph2[lnew].a));
        if(da2 >= da1)
        break;
        l = lnew;
        da1 = da2;
    }
    if(ph1[k].d > ph2[l].d + min_dist) 
    {
        ph1[k].visible = OCCLUDED;      /* occluded point */
        rem++;
    }
    }
    return rem;
}

static long mark_occluded(struct project_help *ph, long num, long num_phs,
    struct project_help *ph2, long num2, long num_phs2)
{
    long i, rem = 0;

    for(i=0; i < num; i++)
    {
    long i_start = ph[i].ar.start, j;

        if(ph[i_start].visible == WRONG_DIRECTION)
            continue;           /* no need to inspect this group */

    for(j=0; j < num; j++)
        if(i != j)
        rem += mark_occluded_help(ph, num_phs, i, ph, num_phs, j, 0.0);

    if(ph2 != NULL)
        for(j=0; j < num2; j++)
        rem += mark_occluded_help(ph, num_phs, i, 
            ph2, num_phs2, j, MAX_DIST_OCCLUDED);
    }

    return rem;
}

static long mark_notinintersection(struct project_help *ph, long num, 
    long num_phs, struct project_help *ph2, long num2, long num_phs2)
{
    long i, j, k, rem = 0;

    for(i=0; i < num; i++)
    {
    long i_start = ph[i].ar.start, i_end = ph[i].ar.end;
    float i_start_a = ph[i_start].a, i_end_a = ph[i_end].a;
    bool all_visible = FALSE;
    bool all_invisible = TRUE;
    bool finish = FALSE;

        if(ph[i_start].visible == WRONG_DIRECTION)
            continue;           /* no need to inspect this group */

    for(j=0; j < num2; j++)
    {
        long j_start = ph2[j].ar.start, j_end = ph2[j].ar.end;
        float j_start_a = NormAngle(ph2[j_start].a - ANGLE_TOLERANCE);
        float j_end_a = NormAngle(ph2[j_end].a + ANGLE_TOLERANCE);
        float da1, da2;

        if(NormAngle(j_end_a - j_start_a) <= 0.0)
        continue;

        da1 = NormAngle(j_start_a - i_start_a);
        da2 = NormAngle(j_end_a - i_end_a);
        if(da1 <= 0.0 && da1 > -PI/2.0 && 
        da2 >= 0.0 && da2 < PI/2.0)
        {
        all_visible = TRUE;
        break;
        }
        else if(NormAngle(j_end_a - i_start_a) > 0.0 && 
        NormAngle(j_start_a - i_end_a) < 0.0)
        all_invisible = FALSE;
    }

    if(all_visible)
        continue;
    if(all_invisible)
    {
        for(k = i_start; !finish; k++)
        {
        if(k == num_phs)
            k = 0;
        if(k == i_end)
            finish = TRUE;      /* last point */
        if(ph[k].visible == VISIBLE)
        {
            ph[k].visible = NOT_IN_INTERSECTION;
                rem++;
        }
        }
        continue;
    }

    /* we have to check all points of this group */
    for(k = i_start; !finish; k++)
    {
        if(k == num_phs)
        k = 0;
        if(k == i_end)
        finish = TRUE;      /* last point */
        if(ph[k].visible != VISIBLE)
        continue;
        for(j=0; j < num2; j++)
        {
        long j_start = ph2[j].ar.start, j_end = ph2[j].ar.end;
        float j_start_a = NormAngle(ph2[j_start].a - ANGLE_TOLERANCE);
        float j_end_a = NormAngle(ph2[j_end].a + ANGLE_TOLERANCE);
        
        if(NormAngle(j_end_a - j_start_a) <= 0.0)
            continue;

        if(NormAngle(ph[k].a - j_start_a) >= 0.0 && 
        NormAngle(ph[k].a - j_end_a) <= 0.0)
            break;      /* point is in intersection */
        }
        if(j >= num2)
        {
        ph[k].visible = NOT_IN_INTERSECTION;
        rem++;
        }
    }
    }
    return rem;
}

static int spcmp(const void *v1, const void *v2)
{
    const CScanPoint *sp1 = (CScanPoint*)v1, *sp2 = (CScanPoint*)v2;

    if(sp1->a < sp2->a)
    return(-1);
    else if(sp1->a > sp2->a)
    return(1);
    return(0);
}

static void remove_nonvisible(CScan *scan, 
    struct project_help *ph, long num_phs)
{
    CScanPoint *sp = scan->m_pPoints;
    long i, j = 0, k, add = 0;

    /*
    ** Remove all invisible points from scan and
    ** calculate reduction rate.
    */
    for(i=0; i < num_phs; i++)
    {
        if(ph[i].visible == VISIBLE)
    {
        for(k=ph[i].start; k <= ph[i].end; k++, j++)
        {
                sp[j] = sp[k];
        sp[j].m_nWeight = 1;
        }   
    }
    else if(j > 0)
        sp[j-1].m_nWeight += ph[i].end + 1 - ph[i].start;
    else
        add += ph[i].end + 1 - ph[i].start;
    }
    scan->m_nCount = j;
    if(j > 0)
        sp[j-1].m_nWeight += add;
    qsort(sp, j, sizeof(CScanPoint), spcmp); 
}

float ScanProjectionFilter(CScan *scan, 
    float scanPosX, float scanPosY, CScan *frame, float maxred)
{
    struct project_help *ph = NULL, *ph2 = NULL;
    long num, num2 = 0, num_phs = 0, num_phs2 = 0, rem = 0;

    if(scan == NULL || scan->m_nCount == 0)
        return(0.0);

    ph = build_project_help(scan, scanPosX, scanPosY, &num_phs);
    if(ph == NULL)
    {
        printf("WARNING: ScanProjectionFilter out of memory,"
        " scan not filtered!\n");
        return(0.0);
    }

    rem += mark_wrong_direction(ph, num_phs);
    if (rem <= num_phs * maxred)
    {
    if(frame && frame->m_nCount > 0)
    {
        ph2 = build_project_help(frame, scanPosX, scanPosY, &num_phs2);
        if(ph2 == NULL)
        printf("WARNING: ScanProjectionFilter out of memory,"
            " frame scan ignored!\n");
    }

    num = build_angular_groups(ph, num_phs);
    if(ph2 != NULL)
    {
        num2 = build_angular_groups(ph2, num_phs2);
#if 0
            float view1 = scan->FieldOfView();
            float view2 = frame->FieldOfView();

            if(MAX(view1, view2) >= DEG2RAD(300))
            keep_notinintersection = TRUE;
#endif
        rem += mark_notinintersection(ph, num, num_phs, 
            ph2, num2, num_phs2);
    }

    if (rem <= num_phs * maxred)
        rem += mark_occluded(ph, num, num_phs, ph2, num2, num_phs2);

    if (rem <= num_phs * maxred)
    {
        remove_nonvisible(scan, ph, num_phs);
    }

    if(ph2)
        SSfree(ph2);
    }
    if(ph)
    SSfree(ph);

    return ((float)rem / num_phs);
}

void ScanProjectionFilter2(CScan *scan1, CScan *scan2,
    float *pred1, float *pred2, float maxred)
{
    CPosture pos;
    float red1, red2 = 0.0;
    CScan *scan1orig = scan1->Duplicate();

    scan2->GetScannerAbsPos(pos);
    red1 = ScanProjectionFilter(scan1, pos.x, pos.y, scan2, maxred);
    if (red1 <= maxred)
    {
    scan1->GetScannerAbsPos(pos);
    red2 = ScanProjectionFilter(scan2, pos.x, pos.y, scan1orig, maxred);
    }
    delete scan1orig;

    if(pred1)
        *pred1 = red1;
    if(pred2)
        *pred2 = red2;
}

void ScanAntennaFilter(CScan *scan, float blindStartA, float blindEndA)
{
    CScanPoint *sp;
    int i, j;

    if(scan == NULL)
        return;

    sp = scan->m_pPoints;
    for(i=0, j=0; i < scan->m_nCount; i++)
        if(sp[i].a < blindStartA || sp[i].a > blindEndA)
            sp[j++] = sp[i];
    scan->m_nCount = j;
}

void ScanMaxRangeFilter(CScan *scan, float maxRange)
{
    CScanPoint *sp;
    int i, j;

    if(scan == NULL)
        return;

    sp = scan->m_pPoints;
    for(i=0, j=0; i < scan->m_nCount; i++)
    if (sp[i].r < maxRange)
            sp[j++] = sp[i];
    scan->m_nCount = j;
}

#define OUTLIERS_DIST 300.0
#define OUTLIERS_COUNT 7

void ScanRemoveOutliersFilter(CScan *scan)
{
    long i, j = 0, start = 0;
    CScanPoint *sp, *prev, *copy;
    float d;

    if(scan == NULL)
    return;

    sp = copy = scan->m_pPoints;
    for(i=1; i < scan->m_nCount; i++)
    {
    prev = sp++;
    d = _hypot(sp->x - prev->x, sp->y - prev->y);
    if(d > OUTLIERS_DIST)
    {
        if(i - start >= OUTLIERS_COUNT)
        {
        while(start < i)
            scan->m_pPoints[j++] = copy[start++]; 
        }
        start = i;
    }
    }

    if(i - start >= OUTLIERS_COUNT)
    {
    while(start < i)
        scan->m_pPoints[j++] = copy[start++]; 
    }

    scan->m_nCount = j;
}
