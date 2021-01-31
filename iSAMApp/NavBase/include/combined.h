#ifndef SL_COMBINED_H
#define SL_COMBINED_H

#include "scan.h"
#include "LineFeatureSet.h"

#define SLCOMBINED_LINE_PERCENTAGE 0.50

/* 
** Same as slCombinedScanMatch but you can provide line scans for
** better speed 
*/
int slCombinedScanMatch2(CScan *refScan, CScan *curScan, PoseGauss *match, float *error);

/*
** returns OK or error code.
** On OK match will hold pose prob of scan match relative to
** pose of refScan, error reflects how good the match is: near 0 -> good.
*/
int slCombinedScanMatch(CScan *refScan, CScan *curScan,
            PoseGauss *match, float *error);
#endif
