#include "stdafx.h"
#include "misc.h"
#include "leastsquare.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


int LeastSquare(
	CPoint2dPair *pp,   /* set of point pairs */
	int n,          /* number of point pairs */
	float rx,      /* the center for rotation */
	float ry,          
	float *pdx,        /* result */
	float *pdy,
	float *pda)
{
	CPoint2dPair mean;

	float s_xx = 0.0, s_xy = 0.0, s_yx = 0.0, s_yy = 0.0;
	float sina, cosa, dx, dy, da;
	int i;

	if (n < 2)
		return FALSE;          // 需要至少3个点

	// 分别计算出两个点集的均值点
	for (i = 0; i < n; i++)
	{
		mean.pt1.x += pp[i].pt1.x;
		mean.pt1.y += pp[i].pt1.y;

		mean.pt2.x += pp[i].pt2.x;
		mean.pt2.y += pp[i].pt2.y;
	}

	mean.pt1.x /= n;
	mean.pt1.y /= n;

	mean.pt2.x /= n;
	mean.pt2.y /= n;


	for (i = 0; i < n; i++)
	{
		s_xx += (pp[i].pt1.x - mean.pt1.x) * (pp[i].pt2.x - mean.pt2.x);
		s_xy += (pp[i].pt1.x - mean.pt1.x) * (pp[i].pt2.y - mean.pt2.y);
		s_yx += (pp[i].pt1.y - mean.pt1.y) * (pp[i].pt2.x - mean.pt2.x);
		s_yy += (pp[i].pt1.y - mean.pt1.y) * (pp[i].pt2.y - mean.pt2.y);
	}

	da = atan2(s_xy - s_yx, s_xx + s_yy);
	sina = sin(da);
	cosa = cos(da);
	dx = mean.pt1.x - rx;
	dy = mean.pt1.y - ry;

	if (pdx != NULL)
		*pdx = mean.pt2.x - rx - (dx * cosa - dy * sina);

	if (pdy != NULL)
		*pdy = mean.pt2.y - ry - (dx * sina + dy * cosa);

	if (pda != NULL)
		*pda = da;

	return TRUE;
}

