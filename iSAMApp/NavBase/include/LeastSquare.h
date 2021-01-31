#ifndef LEASTSQUARE_H
#define LEASTSQUARE_H

#include "ZTypes.h"
#include "Geometry.h"

class CPoint2dPair
{
public:
/*
    float x1, y1;
    float x2, y2;
*/
	CPoint2d pt1;
	CPoint2d pt2;

public:
	CPoint2dPair()
	{
		pt1.x = pt1.y = 0;
		pt2.x = pt2.y = 0;
	}
};

int LeastSquare(        /* returns TRUE on success, FALSE on failure */
	CPoint2dPair *pp,   /* set of point pairs */
	int n,          /* number of point pairs */
	float rx,      /* the center for rotation */
	float ry,      
	float *dx,     /* results */
	float *dy,
	float *da  
	);


#endif
