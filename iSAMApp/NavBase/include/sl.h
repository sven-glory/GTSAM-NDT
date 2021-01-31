#ifndef SL_H
#define SL_H

enum
{
    SLERR_MEM = 1,              /* not enough memory */
    SLERR_NOSCANS,              /* missing scans */
    SLERR_NOTENOUGHPOINTSMATCHED,   /* not enough correspondences found */
    SLERR_LEASTSQUARE,          /* error from least square */
    SLERR_MATRIXINVERSE,    /* error on getting inverse matrix */
    SLERR_NEGATIVEMATRIX,       /* resulting error matrix invalid! */
    SLERR_DETZERO,              /* determinante is zero */
    SLERR_UNRELIABLE,       /* scan match unreliable */
    SLERR_AMBIGOUS,     /* ambigous situation */
};

#define SL_MIN_POINTS_IN_SCAN 6

extern short slMaxIterations;
extern float slEpsD, slEpsA;

#endif
