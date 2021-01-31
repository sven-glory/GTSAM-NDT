#ifndef SL_IDC_H
#define SL_IDC_H

#include "ZTypes.h"

int slIDCScanMatch(CScan *refScan, CScan *curScan,
            PoseGauss *match, float *error);
/*
** returns OK or error code.
** On OK match will hold pose prob of match relative to
** pose of refScan, error reflects how good the match is: near 0 -> good.
*/

int slIDCScanMatch2(const CScan *refScan, /*const CLineFeatureSet *refls,*/
    const CScan *curScan, PoseGauss *match, float *error);
/*
** Same as slIDCScanMatch but you can provide a line scan for the reference
** scan for better speed.
*/

#endif
