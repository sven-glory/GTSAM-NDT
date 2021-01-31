#ifndef __CScanGroup
#define __CScanGroup

#include "Scan.h"

CScan *MergeScans(CScan **scans, int num, 
    const PoseGauss *mergePos);
/*
** Merges a set of scans to one scan.
** if mergePos is NULL then the new scan's robot position will be 
** center of gravity of all scan positions.
*/

void FreeScans(CScan **scans, long num);
CScan **LoadScans(const char *filename, long *pnum);
bool SaveScans(CScan **scans, long num, const char *filename);

bool SaveMap(CScan **scans, long num, const char *filename);
/*
** Saves all scans into a single map file by writing out
** the (x,y) positions of all scan points.  Returns true on success
** and false on failure.
*/


CScan **AddScan(CScan **scans, CScan *scan, long *pnum);

void ScansDimensions(CScan **scans, long num, 
    float *xoff, float *yoff, float *width, float *height);

FILE *SaveDividedMapHeader(const char *filename);
/*
** Opens the filename and returns the FILE opened, writes the header.
** Returnes NULL on failure, and a FILE pointer on success.
*/

bool SaveDividedMapScansHeader(CScan **scans, long num, FILE *file);
/*
** Saves scan meta data into given file.  Returns true on success and
** false on failure.
**/

bool SaveDividedMapScansData(CScan **scans, long num, FILE *file);
/*
** Saves all scans points into given file by writing the (x, y)
** positions of all scan points.  Returns true on success and false on
** failure.
**/

#endif
