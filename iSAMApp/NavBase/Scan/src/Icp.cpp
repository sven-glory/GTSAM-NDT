#include "stdafx.h"
#include <math.h>
#include "polar_match.h"

/** @brief Calculate the distance of a point from a line section. 

计算一个点P(x3, y3)到一条线段AB的距离(A(x1, y1), B(x2, y2))。如果交点落到线段之外，则返回-1。
同时，P点到AB的垂足点的坐标C(x, y)也被返回。
此函数在ICP中用到。

Calculates the distance of the point (x3,y3) from a line defined by (x1,y1)
and (x2,y2). Returns the distance to the line or -1 if the
projection of (x3,y3) falls outside the line segment defined by (x1,y1)
and (x2,y2). The projection of (x3,y3) onto the line is also returned in x,y.
This function is used in ICP.
@param x1,y1 The start point of the line section.
@param x2,y2 The end point of the line section.
@param x3,y3 The point of which distance it sought.
@param x,y (x3,y3) projected onto the line segment is returned here.
@return The distance from the line or -1 if the projection falls outside of the line segment.
*/
float point_line_distance(float x1, float y1, float x2, float y2,
									 float x3, float y3, float *x, float *y)
{
	float ax, ay, t1, D;

	ax = x2 - x1;
	ay = y2 - y1;

	D = sqrtf(ax * ax + ay * ay);
	if (D < 0.0001)
	{
		ASSERT(FALSE);    // unexpected D
		return -1;
	}

	t1 =	- ( -ax*x3 + ay*y1 + ax*x1 - ay*y3 ) / ( ax*ax+ay*ay );
	if (t1 < 0 || t1 > 1)    // Projection falls outside the line segment?
		return -1;

	*x = x1 + t1 * ax;
	*y = y1 + t1 * ay;
	return (sqrtf((x3 - *x) * (x3 - *x) + (y3 - *y) * (y3 - *y)));    //distance of line to p.
}

/** @brief Matches two laser scans using the iterative closest point method.

Minimizes least square error of points through changing lsa->rx, lsa->ry, lsa->th
by using ICP. It interpolates associated 
points. Only the best 80% of points are used in the pose calculation.
Scan projection is done at each iteration.

For maintanence reasons changed scan projection to that of psm.
*/
float pm_icp (	const CPmScan *lsr,CPmScan *lsa )
{
#define INTERPOLATE_ICP	//comment out if no interpolation of ref. scan points iS	necessary
	CPmScan		act,	ref;//copies of current and reference scans
	float	 rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
	float	 t13,t23,LASER_Y = PM_LASER_Y;
	int			 new_bad[PM_L_POINTS];//bad flags of the projected current scan range readings
	float	 new_r[PM_L_POINTS];//ranges of current scan projected into ref. frame for occlusion check
	float	 nx[PM_L_POINTS];//current scanpoints in ref coord system
	float	 ny[PM_L_POINTS];//current scanpoints in ref coord system
	int			 index[PM_L_POINTS][2];//match indices current,refernce
	float	 dist[PM_L_POINTS];// distance for the matches
	int			 n = 0;//number of valid points
	int			 iter,i,j,small_corr_cnt=0,k,imax;
	int			 window			 = PM_SEARCH_WINDOW;//+- width of search for correct orientation
	float	 abs_err=0,dx=0,dy=0,dth=0;//match error, current scan corrections
	float	 co,si;

#ifdef	PM_GENERATE_RESULTS
	double start_tick, dead_tick,end_tick,end_tick2;
	FILE *f;
	f = fopen ( PM_TIME_FILE,"w" );
	dead_tick = 0;
	start_tick =pm_msec();
#endif

	act = *lsa;
	ref = *lsr;

	rx =	ref.rx; ry = ref.ry; rth = ref.th;
	ax =	act.rx; ay = act.ry; ath = act.th;

	//transformation of current scan laser scanner coordinates into reference
	//laser scanner coordinates
	t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
	t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

	ref.rx = 0;	 ref.ry = 0;	 ref.th = 0;
	act.rx = t13; act.ry = t23; act.th = ath-rth;

	ax = act.rx; ay = act.ry; ath = act.th;
	//from now on act.rx,.. express the lasers position in the ref frame
	
	//intializing x,y of act and ref
	for ( i=0;i<PM_L_POINTS;i++ )
	{
		ref.x[i] = ref.r[i]*pm_co[i];
		ref.y[i] = ref.r[i]*pm_si[i];

		act.x[i] = act.r[i]*pm_co[i];
		act.y[i] = act.r[i]*pm_si[i];
	}//for i

	iter = -1;
	while ( ++iter<PM_MAX_ITER_ICP && small_corr_cnt<3 ) //have to be a few small corrections before stop
	{

		if ( ( fabsf ( dx ) +fabsf ( dy ) +fabsf ( dth ) *PM_R2D ) <PM_STOP_COND_ICP )
			small_corr_cnt++;
		else
			small_corr_cnt=0;

#ifdef	PM_GENERATE_RESULTS
		end_tick =pm_msec();
		fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
							 end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
		end_tick2 =pm_msec();
		dead_tick += end_tick2- end_tick;
#endif

#ifdef GR
		dr_erase();
		dr_circle ( ax,ay,5.0,"green" );
		dr_line ( 0,-100,200,-100,"black" );
		dr_line ( 0,-200,200,-200,"black" );
#endif
				
		//CScan projection
		act.rx = ax;act.ry = ay;act.th = ath;
		pm_scan_project(&act,	new_r, new_bad);
				
		// transformation the cartesian coordinates of the points:
		co = cosf ( ath );
		si = sinf ( ath );		
		for ( i=0;i<PM_L_POINTS;i++ )
		{
			nx[i]		 = act.x[i]*co - act.y[i]*si + ax;
			ny[i]		 = act.x[i]*si + act.y[i]*co + ay;
#ifdef GR
			if ( ref.bad[i] )
				dr_circle ( ref.x[i],ref.y[i],4,"yellow" );
			 else
				dr_circle ( ref.x[i],ref.y[i],4,"black" );
			if ( new_bad[i] )
				dr_circle ( nx[i],ny[i],4,"green" );
			else
				dr_circle ( nx[i],ny[i],4,"blue" );
#endif
		}
//		dr_zoom();
#ifdef GR
		cout <<"interpolated ranges. press enter"<<endl;
/*		for ( i=0;i<PM_L_POINTS;i++ )
			dr_circle ( new_r[i]*pm_co[i],new_r[i]*pm_si[i],6,"red" );*/
		dr_zoom();
#endif

		//Correspondence search: go through the points of the current
		//scan and find the closest point in the reference scan 
		//lying withing a search interval. 
		n=0;
		float d,min_d;
		int min_idx;

		for ( i=0;i<PM_L_POINTS;i++ )
		{
			min_d = 1000000;
			min_idx = -1;
			if ( !new_bad[i] )
			{
				int imin,imax;
				imin = i-window ;
				if ( imin<0 )
					imin =0;
				imax = i+window ;
				if ( imax>PM_L_POINTS )
					imax =PM_L_POINTS;
					
				for ( j=imin;j<imax;j++ )
				{
					if ( !ref.bad[j] )
					{
						d =	SQ ( nx[i]-ref.x[j] ) + SQ ( ny[i]-ref.y[j] );//square distance
						if ( d<min_d )
						{
							min_d	= d;
							min_idx = j;
						}
					}
				}//for
				if ( min_idx>=0 && sqrtf ( min_d ) <PM_MAX_ERROR ) // was there any match closer than 1m?
				{
					index[n][0] = i;
					index[n][1] = min_idx;
					dist[n] = sqrtf ( min_d );
					n++;
#ifdef GR
					dr_line ( nx[i],ny[i],ref.x[min_idx],ref.y[min_idx],"blue" );
#endif
				}
			}//if
		}//for
//		dr_zoom();

		if ( n<PM_MIN_VALID_POINTS )
		{
			ASSERT(FALSE);         // Not enough points

#ifdef	PM_GENERATE_RESULTS
			fclose ( f );
#endif
			throw 1;
		}

		//sort the matches with bubble sort
		//put the largest 20 percent to the end
		imax = ( int ) ( ( double ) n*0.2 );
		for ( i=0;i<imax;i++ )
			for ( j=1;j< ( n-i );j++ )
			{
				if ( dist[j]<dist[j-1] ) //are they in the wrong order?
				{
					//swap them
					k						 = index[j][0];
					index[j][0]	 = index[j-1][0];
					index[j-1][0] = k;

					k						 = index[j][1];
					index[j][1]	 = index[j-1][1];
					index[j-1][1] = k;

					d						 = dist[j];
					dist[j]			 = dist[j-1];
					dist[j-1]		 = d;
				}
			}//for j

#ifdef INTERPOLATE_ICP
		//------------------------INTERPOLATION---------------------------
		//comment out if not necessary
		float ix[PM_L_POINTS],iy[PM_L_POINTS];//interp. ref. points.
		//replace nx,xy with their interpolated... where suitable
		{

			float d0,d1,d2;
			float minx1,miny1,minx2,miny2;
			int max_i = n-imax;
			for ( i=0;i<max_i;i++ )
			{

				//d1 = point_line_distance(1, 2,2,3, 2,2, &minx1, &miny1);	//debug

#ifdef GR
				dr_circle ( nx[index[i][0]],ny[index[i][0]],1.0,"brown" );
				dr_circle ( ref.x[index[i][1]],ref.y[index[i][1]],1.0,"brown" );
				dr_circle ( ref.x[index[i][1]-1],ref.y[index[i][1]-1],1.0,"brown" );
				dr_circle ( ref.x[index[i][1]+1],ref.y[index[i][1]+1],1.0,"brown" );
#endif
				d1=-1;d2=-1;
				if ( index[i][1]>0 ) //not associated to the first point?
				{
					d1 = point_line_distance ( ref.x[index[i][1]-1], ref.y[index[i][1]-1],
																		 ref.x[index[i][1]],	 ref.y[index[i][1]],
																		 nx[index[i][0]],			ny[index[i][0]],
																		 &minx1, &miny1 );
				}

				if ( index[i][1]< ( PM_L_POINTS-1 ) ) //not associated to the last point?
				{
					d2 = point_line_distance ( ref.x[index[i][1]],	ref.y[index[i][1]],
																		 ref.x[index[i][1]+1],ref.y[index[i][1]+1],
																		 nx[index[i][0]],		 ny[index[i][0]],
																		 &minx2, &miny2 );
				}

				ix[index[i][1]] = ref.x[index[i][1]];
				iy[index[i][1]] = ref.y[index[i][1]];
				d0 = sqrtf ( SQ ( ref.x[index[i][1]]-nx[index[i][0]] ) + SQ ( ref.y[index[i][1]]-ny[index[i][0]] ) );

				//is the first point closer?
				if ( d1>0 && d1<d0 )
				{
					ix[index[i][1]] = minx1;
					iy[index[i][1]] = miny1;
					d0 = d1;
				}

				//is the second point closer?
				if ( d2>0 && d2<d0 )
				{
					ix[index[i][1]] = minx2;
					iy[index[i][1]] = miny2;
				}
#ifdef GR
				dr_line ( nx[index[i][0]],ny[index[i][0]],ix[index[i][1]],iy[index[i][1]],"green" );
#endif
			}//for
		}
#endif


		//pose estimation
		//------------------------------------------translation-------------

		// do the weighted linear regression on the linearized ...
		// include angle as well
		//computation of the new dx1,dy1,dtheta1
		float sxx=0,sxy=0,syx=0,syy=0;
		float meanpx,meanpy,meanppx,meanppy;
		meanpx = 0;meanpy = 0;
		meanppx= 0;meanppy= 0;

		abs_err=0;
		imax = n-imax;
		for ( i=0;i<imax;i++ )
		{
			//weight calculation
			// do the cartesian calculations....
			meanpx +=	nx[index[i][0]];
			meanpy +=	ny[index[i][0]];

#ifdef INTERPOLATE_ICP
			meanppx +=	ix[index[i][1]];
			meanppy +=	iy[index[i][1]];
#else
			meanppx +=	ref.x[index[i][1]];
			meanppy +=	ref.y[index[i][1]];
#endif

#ifdef GR
			dr_line ( nx[index[i][0]],ny[index[i][0]],ref.x[index[i][1]],ref.y[index[i][1]],"red" );
#endif
		}//for
		meanpx /= imax;
		meanpy /= imax;

		meanppx /= imax;
		meanppy /= imax;

		for ( int i=0;i<imax;i++ )
		{
#ifdef INTERPOLATE_ICP
			sxx += ( nx[index[i][0]] - meanpx ) * ( ix[index[i][1]] - meanppx );
			sxy += ( nx[index[i][0]] - meanpx ) * ( iy[index[i][1]] - meanppy );
			syx += ( ny[index[i][0]] - meanpy ) * ( ix[index[i][1]] - meanppx );
			syy += ( ny[index[i][0]] - meanpy ) * ( iy[index[i][1]] - meanppy );
#else
			sxx += ( nx[index[i][0]] - meanpx ) * ( ref.x[index[i][1]] - meanppx );
			sxy += ( nx[index[i][0]] - meanpx ) * ( ref.y[index[i][1]] - meanppy );
			syx += ( ny[index[i][0]] - meanpy ) * ( ref.x[index[i][1]] - meanppx );
			syy += ( ny[index[i][0]] - meanpy ) * ( ref.y[index[i][1]] - meanppy );
#endif
		}
		//computation of the resulting translation and rotation
		//for method closest point match
		dth = atan2f ( sxy-syx,sxx+syy );
		dx	= meanppx - ax - ( cosf ( dth ) * ( meanpx- ax ) - sinf ( dth ) * ( meanpy - ay ) );
		dy	= meanppy - ay - ( sinf ( dth ) * ( meanpx- ax ) + cosf ( dth ) * ( meanpy - ay ) );

		ax += dx;
		ay += dy;
		ath+= dth;
		ath = norm_a ( ath );

//		//for SIMULATION iteration results..
//		cout <<iter<<"		 "<<ax<<"		"<<ay<<"		"<<ath*PM_R2D<<" ;"<<endl;
#ifdef GR
		cout <<"iter "<<iter<<" "<<ax<<" "<<ay<<" "<<ath*PM_R2D<<" "<<dx<<" "<<dy<<endl;
//			if(iter==0)
		dr_zoom();
		usleep ( 10000 );

#endif

	}//for iter
	//cout <<iter<<endl;
#ifdef	PM_GENERATE_RESULTS
	end_tick =pm_msec();
	fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
						 end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
	fclose ( f );
#endif

	lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
	return ( abs_err/n );
}//pm_icp

