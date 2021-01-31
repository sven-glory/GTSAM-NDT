//                           - PfR2000LaserScanner.cpp -
//
//   The interface of class "cSick581LaserScanner".
//
//   Author: sfe1012
//   Date:   2018. 01. 18
//
#include"sensor/Sick581LaserScanner.h"
#include "lcm/lcm.h"
#include "lcm/lcm_coretypes.h"
void* LCMRecvTask(LPVOID pParam)
{
     cSick581LaserScanner* oLaserScanner = reinterpret_cast<cSick581LaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
         lcm_handle(oLaserScanner->m_plcm);
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}

cSick581LaserScanner::cSick581LaserScanner(int fAngRes, float fStartAng, float fEndAng):
    CRangeScanner(fAngRes, fStartAng, fEndAng, PEPPERL_FUCHS)
{
    m_iNetType = -1;
    m_iLaserPort = 0;
    m_iScanFrequency = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
}

cSick581LaserScanner::~cSick581LaserScanner()
{
    Stop();
}

bool cSick581LaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);

    PthreadJoin(m_Thread);

    if (m_hKillThread != NULL)
    {
        CloseHandle(m_hKillThread);
        m_hKillThread = NULL;
    }

    if (m_hThreadDead != NULL)
    {
        CloseHandle(m_hThreadDead);
        m_hThreadDead = NULL;
    }

    m_bStarted = false;
    return true;
}
int cSick581LaserScanner::LCMInit( void )
{
    m_plcm = lcm_create( NULL );

    if( !m_plcm )
    return -1;

   laser_t_subscribe(m_plcm, "LASER_DATA",LaserDataHandle, this);

    return 0;
}

void cSick581LaserScanner::LaserDataHandle(const lcm_recv_buf_t *rbuf, const char * channel,
                     const laser_t *laserdata,    void * user)
{
    //printf( "call %s\n", __func__ );
    cSick581LaserScanner *pLaser = (cSick581LaserScanner *)user;
    pLaser -> LaserDataHandle_callBack( rbuf, channel, laserdata );
}

void cSick581LaserScanner::LaserDataHandle_callBack(const lcm_recv_buf_t *rbuf, const char * channel, const laser_t *laserdata)
{
    timeval begintime;
    timeval endtime;
    timeval time1;
    static timeval time2;

    //printf("  -----navi get laser data ----%d -\n", laserdata->utime);

    gettimeofday(&time1,NULL);
    gettimeofday(&begintime,NULL);

    //NAVI_PutLaserData(laserdata->rad0, laserdata->radstep, laserdata->nranges, laserdata->ranges);

    std::vector<std::uint32_t> distance;
    std::vector<std::uint32_t> intensity;

    for(int i = 0 ; i < laserdata->nranges ; i++)
    {
        distance.push_back( ((unsigned int)(laserdata->ranges[i]*1000)) ); //米 转 毫米
        intensity.push_back( 500  /* ( (unsigned int)(laserdata->intensities[i]) )*/ );
    }
    //FIXME:临时用当前接收到点云数据的时间作为当前帧的产生时间
    unsigned long long raw_time = GetTickCount();
    AddRawPointCloud( distance , intensity , raw_time , raw_time );

    gettimeofday(&endtime,NULL);

    //printf("loc time = %d\n", (endtime.tv_sec - begintime.tv_sec)*1000000 + (endtime.tv_usec -begintime.tv_usec));
    //printf("laser time = %d\n", (time1.tv_sec - time2.tv_sec)*1000 + (time1.tv_usec -time2.tv_usec)/1000);

    time2= time1;
}


BOOL cSick581LaserScanner::Start(const char*device_name,
                                 const char*host_name,
                                 const int laserid,
                                 const int iPort,
                                 const int iNetType,
                                 const int iScanFrequency,
                                 const int iSamplesPerScan)
{
    if(m_bStarted){
        return TRUE;
    }

    LCMInit();
    m_nConnectTime = GetTickCount();

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return FALSE;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return FALSE;

    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_Thread,&attr,LCMRecvTask,reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat R2000ScanSupport Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
        std::cout<<"Creat R2000ScanSupport Pthread OK"<<std::endl;

    pthread_attr_destroy(&attr);

    m_bStarted = true;

    return TRUE;
}
