//                           - PfR2000LaserScanner.H -
//
//   The interface of class "cSick581LaserScanner".
//
//   Date:   2020. 04. 26
//

#ifndef CSICK_581_LASER_SCANNER_H
#define CSICK_581_LASER_SCANNER_H

#include <deque>
#include "ZTypes.h"
#include "RangeScanner.h"
#include"Tools.h"
#include "laser_t.h"
//网络数据类型
#define TCP_DATA 1
#define UDP_DATA 2

class cSick581LaserScanner : public CRangeScanner
{
  public:
        cSick581LaserScanner(int fAngRes, float fStartAng, float fEndAng);
        ~cSick581LaserScanner();
       virtual bool Stop();

  public:
        // 启动激光扫描传感器
        virtual BOOL Start(const char*device_name,
                           const char*host_name = NULL,
                           const int laserid = 0,
                           const int iPort = 80,
                           const int iNetType = TCP_DATA,
                           const int iScanFrequency = 20/*20*/,
                           const int iSamplesPerScan = 3600/*3600*/);
        int  LCMInit( void );
        void LaserDataHandle_callBack(const lcm_recv_buf_t *rbuf, const char * channel,
                                        const laser_t *laserdata);
        static void LaserDataHandle(const lcm_recv_buf_t *rbuf, const char * channel,
                             const laser_t *laserdata,    void * user);
  public:
        HANDLE      m_hKillThread;       // Handle of "Kill thread" event
        HANDLE      m_hThreadDead;       // Handle of "Thread dead" event
        lcm_t*      m_plcm;
  private:
        pthread_t   m_Thread;
        int         m_iLaserPort;
        int         m_iScanFrequency;
        int         m_iSamplesPerScan;
        int         m_iNetType; //1:Tcp 2:Udp

};
#endif // CMAPPING_H
