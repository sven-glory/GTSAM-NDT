#ifndef MISC_H
#define MISC_H

#include "ZTypes.h"

#ifdef WIN32
#include <math.h>
#include <float.h>
#endif

#ifdef DEBUG
#define DPRINTF(x) printf x
#else
#define DPRINTF(x)
#endif

#ifdef WIN32
// Turn off warning about loosing from the conversion to double.
#pragma warning(disable:4244)
// Turn off warning about inetgral size mismatch, conversion supplied
#pragma warning(disable:4761)
// turn off warning about truncation from 'const double' to float
#pragma warning(disable:4305)
// turn off warning about zero sized arrays
#pragma warning(disable:4200)
typedef unsigned long ulong;
typedef unsigned short USHORT;
#ifndef usleep
#define usleep(num) Sleep(num/100)
#endif // usleep
#ifndef drand48
#define drand48() (double) rand() * (1.0 /(double) RAND_MAX)
#endif // drand48
#ifndef isnan
#define isnan _isnan
#endif // isnan
#ifndef drem
/* #define drem fmod     -> this is slightly different */
#define drem(x, y) ((x) - (int)floor((x) / (y) + .5) * (y))
#endif // drem
#ifndef strcasecmp 
#define strcasecmp _stricmp
#endif // strcasecmp
#else // not windows
extern double drand48(void); 
#endif

#include <stdio.h>
#ifndef WIN32
#include <unistd.h>
#endif

#include <ctype.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#ifndef WIN32
#include <sys/time.h>
#else
#include <winsock2.h>
#endif
#include <sys/stat.h>

#ifndef __cplusplus
typedef char bool;
#endif
typedef unsigned char uchar;

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#ifndef OK
#define OK 0
#define NOK 1000
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b))? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b))? (a) : (b))
#endif

#ifndef ABS
#define ABS(x) (((x) >= 0)? (x) : -(x))
#endif

#ifndef ISSPACE
#define ISSPACE(x) isspace((unsigned char)x)
#endif

#ifndef NEAR_ZERO
#define NEAR_ZERO(x) (ABS(x) < 1.0e-16)
#endif

#define DEG2RAD(x) ((x) * PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / PI)

#ifndef ROUND
#define ROUND(x)  (floor((x) + 0.5))
#endif

#define ANGLE_INVALID HUGE_VAL

extern struct timeval TimevalZero;

void MiscInit(void);
/*
** Initialization, e.g. for MsgPrint
*/

void MsgPrint(char *fmt, ...);
/*
** Prints formatted text with date stamp.
*/

float NormAngle(float a);
/*
** Normalizes the angle to the interval [-PI;PI].
*/

float NormAngle180(float a);
/*
** Normalizes the angle to the interval [-PI/2;PI/2].
*/

float NormAngleMod(float a, float mod);
/*
** Normnalizes the angle modulo mod.  Returns angle in [-mod/2,mod/2].
*/

float Distance(float x1, float y1, float x2, float y2);
/*
** Returns euclidian distance from (x1,y1) to (x2,y2).
*/

FILE *ifopen(const char *filename, const char *mode);
/*
** improved fopen, support for stdin, stdout, pipes, and gzipped files.
*/

#ifdef WIN32
  typedef HANDLE MutexType;
#else
#include <pthread.h>
  typedef pthread_mutex_t MutexType;
#endif

extern MutexType ScanStudioMutex;
/* this is sort of a hack, but a global mutex type, it should probably
   be in mapper handle or something and passed around */

int SSMutexCreate(MutexType *mutex);
/* initializes the mutex given */

int SSMutexDestroy(MutexType *mutex);
/* destroys the mutex given */

int SSMutexLock(MutexType *mutex);
/* locks the mutex given */

int SSMutexUnlock(MutexType *mutex);
/* unlocks the mutex given */

/* our own memory functions which lock around the memory calls if a
   mutex has been created */
void *SScalloc(size_t nmemb, size_t size);

void *SSmalloc(size_t size);

void SSfree(void *ptr);

void *SSrealloc(void *ptr, size_t size);



#endif
