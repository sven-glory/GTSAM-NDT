#include "stdafx.h"
#include <io.h>
#include "misc.h"
#include <stdarg.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


struct timeval TimevalZero = { 0, 0 };

static struct timeval start_tv = { 0, 0 };
void MiscInit(void)
{
#ifndef WIN32 /* HELP HERE someday, maybe */
  gettimeofday(&start_tv, NULL);
#endif
}

void MsgPrint(char *fmt, ...)
{
	static struct timeval last_tv = { -1, -1 };
	va_list ap;

/* HELP HERE someday, maybe */
#ifndef WIN32
	struct timeval tv;

	gettimeofday(&tv, NULL);
	if(timercmp(&last_tv, &tv, !=))
	{
	last_tv = tv;
	timersub(&tv, &start_tv, &tv);
	printf("%5ld.%03ld: ", tv.tv_sec, tv.tv_usec / 1000);
	}
	else
	{
	  printf("%*s ", 10, "");
	}
#endif
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

float NormAngle(float a)
{
	float fa = fabs(a);

	if (fa >= PI) 
	{
		if (fa > 3*PI) 
		{
			if (fa > 1e20) 
			{
				fprintf(stderr, "NormAngle: strange angle: %g\n", a);
			}
			a = drem(a, 2*PI);
		}
		else if (a < -PI) 
		{
			a += 2*PI;
		}
		else if (a >= PI) 
		{
			a -= 2*PI;
		}
	}
	return a;
}

float NormAngle180(float a)
{
	float fa = fabs(a);

	if (fa >= PI/2) 
	{
		if (fa > 3*PI/2) 
		{
			if (fa > 1e20/2) 
			{
				fprintf(stderr, "NormAngle180: strange angle: %g\n", a);
			}
			a = drem(a, PI);
		}
		else if (a < -PI/2) 
		{
			a += PI;
		}
		else if (a >= PI/2) 
		{
			a -= PI;
		}
	}
	return a;
}

float NormAngleMod(float a, float mod)
{
	float fa = fabs(a);
	float mod2 = mod/2.0;

	if (fa >= mod2) 
	{
		if (fa > mod + mod2) 
		{
			if (fa > 1e20) 
			{
				fprintf(stderr, "NormAngleMod: strange angle: %g\n", a);
			}
			a = drem(a, mod);
		}
		else if (a < -mod2) 
		{
			a += mod;
		}
		else if (a >= mod2) 
		{
			a -= mod;
		}
	}
	return a;
}

float Distance(float x1, float y1, float x2, float y2)
{
	return (float)_hypot(x1 - x2, y1 - y2);
}

FILE *ifopen(const char *filename, const char *mode)
{
	if (filename == NULL || mode == NULL)
		return NULL;

	/* check for unknown modes */
	if (mode[0] != 'r' && mode[0] != 'w' && mode[0] != 'a')
		return(fopen(filename, mode));

	if (!strcmp(filename, "-"))
	{
		int fd = _dup((mode[0] == 'r')? 0 : 1);

		if (fd < 0)
			return NULL;

		return _fdopen(fd, mode);
	}
	else
		return fopen(filename, mode);
}


/* the global mutex object, see comment in misc.h */
MutexType ScanStudioMutex;

/* if this is false then there have been no mutexs created, in which
   case we will assume that we shouldn't be locking   */
int ScanStudioMutexCreated = FALSE;

int SSMutexCreate(MutexType *mutex)
{
  ScanStudioMutexCreated = TRUE;
#ifdef WIN32
  *mutex = CreateMutex(0, 1, 0);
  if (!(*mutex))
  {
	fprintf(stderr, "SSMutexCreate: Failed to initialize mutex\n");
	return FALSE;
  }
  else
  {
	SSMutexUnlock(mutex);
	return TRUE;
  }
#else
  if (pthread_mutex_init(mutex, 0) < 0)
  {
	fprintf(stderr, "SSMutexCreate: Failed to initialize mutex\n");
	return FALSE;
  }
  else
  {
	SSMutexUnlock(mutex);
	return TRUE;
  }
#endif
}

int SSMutexDestroy(MutexType *mutex)
{
#ifdef WIN32
  if (!CloseHandle(*mutex))
  {
	fprintf(stderr, "SSMutexDestroy: Failed to destroy mutex\n");
	return FALSE;
  }
  else 
	return TRUE;
#else
  if (pthread_mutex_destroy(mutex) < 0)
  {
	fprintf(stderr, "SSMutexDestroy: Failed to destroy mutex\n");
	return FALSE;
  }
  else
	return TRUE;
#endif
}

int SSMutexLock(MutexType *mutex)
{
#ifdef WIN32
  DWORD ret;
  ret = WaitForSingleObject((*mutex), INFINITE);
  if (ret == WAIT_ABANDONED)
  {
	fprintf(stderr, "SSMutexLock: tried to lock a mutex which was locked by another thread that has since exited... recoverable error\n");
	return SSMutexLock(mutex);
  }
  else if (ret == WAIT_OBJECT_0)
	return TRUE;
  else
  {
	fprintf(stderr, "SSMutexLock: Failed to lock due to unknown error");
	return FALSE;
  }

#else
  if (pthread_mutex_lock(mutex) < 0)
  {
	if (errno == EDEADLK)
	  fprintf(stderr, "SSMutexLock: Trying to lock a mutex which is already locked by this thread\n");
	else
	  fprintf(stderr, "SSMutexLock: Failed to lock due to unknown error\n");
	return FALSE;
  }
  else
	return TRUE;
#endif
}

int SSMutexUnlock(MutexType *mutex)
{
#ifdef WIN32
  if (!ReleaseMutex(*mutex))
  {
	fprintf(stderr, "SSMutexUnlock: Failed to unlock for an unknown reason");
	return FALSE;
  }
  else
	return TRUE;
#else
  if (pthread_mutex_unlock(mutex) < 0)
  {
	if (errno == EPERM)
	  fprintf(stderr, "SSMutexUnlock: Trying to unlock a mutex which this thread does not own\n");
	else
	  fprintf(stderr, "SSMutexUnlock: Failed to unlock for an unknown reason\n");
	return FALSE;
  }
  else
	return TRUE;
#endif
}



void *SScalloc(size_t nmemb, size_t size)
{
  void *ret;

  if (ScanStudioMutexCreated)
	SSMutexLock(&ScanStudioMutex);

  ret = calloc(nmemb, size);
  
  if (ScanStudioMutexCreated)
	SSMutexUnlock(&ScanStudioMutex);
  
  return ret;
}

void *SSmalloc(size_t size)
{
  void *ret;

  if (ScanStudioMutexCreated)
	SSMutexLock(&ScanStudioMutex);

  ret = malloc(size);
  
  if (ScanStudioMutexCreated)
	SSMutexUnlock(&ScanStudioMutex);
  
  return ret;
}

void SSfree(void *ptr)
{
  if (ScanStudioMutexCreated)
	SSMutexLock(&ScanStudioMutex);

  free(ptr);
  
  if (ScanStudioMutexCreated)
	SSMutexUnlock(&ScanStudioMutex);
}

void *SSrealloc(void *ptr, size_t size)
{
  void *ret;

  if (ScanStudioMutexCreated)
	SSMutexLock(&ScanStudioMutex);

  ret = realloc(ptr, size);
  
  if (ScanStudioMutexCreated)
	SSMutexUnlock(&ScanStudioMutex);
  
  return ret;
}
