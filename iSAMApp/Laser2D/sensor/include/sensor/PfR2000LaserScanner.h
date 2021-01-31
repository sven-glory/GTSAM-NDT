//                           - PfR2000LaserScanner.H -
//
//   The interface of class "cPfR2000LaserScanner".
//
//   Author: sfe1012
//   Date:   2018. 01. 18
//

#ifndef CPF_R2000_LASER_SCANNER_H
#define CPF_R2000_LASER_SCANNER_H

#include <deque>
#include <pepperl_fuchs_r2000/r2000_driver.h>
#include "ZTypes.h"
#include "RangeScanner.h"

#define DEFAULT_SCANNER_RESO   0.1/*0.1f*/                               // 缺省角分辨率(度)
#define SCANNER_RESO_RAD       (DEFAULT_SCANNER_RESO/180.f*PI)  // 缺省角分辨率(弧度)
#define SCAN_COUNT             ((int)(360/DEFAULT_SCANNER_RESO))
#define MAX_LINE_NUM           2000                            // 最大直线段数量
#define MAX_CIRCLE_NUM         1000                           // 最大圆数量
#define MAX_DATA_COUNT_QUEUE	18							//队列中数据最大数量

//网络数据类型
#define TCP_DATA 1
#define UDP_DATA 2

class cPfR2000LaserScanner : public CRangeScanner
{
public:
        cPfR2000LaserScanner(int fAngRes, float fStartAng, float fEndAng);
        ~cPfR2000LaserScanner();

       virtual bool Stop();

  public:
        void    SupportMeasure();
        void ReturnDist(int *GetData, const int &Size);
        // 启动激光扫描传感器
        virtual BOOL Start(const char*device_name,
                           const char*host_name = NULL,
                           const int laserid = 0,
                           const int iPort = 80,
                           const int iNetType = TCP_DATA,
                           const int iScanFrequency = 20/*20*/,
                           const int iSamplesPerScan = 3600/*3600*/);
        BOOL ConnectToR2000Laser();
        BOOL CloseR2000LaserConnection();
  public:
        //deque<vector<unsigned int>>pf;  //distance
        //deque<vector<unsigned int>>pfAmplitude; //Amplitude
        HANDLE      m_hKillThread;       // Handle of "Kill thread" event
        HANDLE      m_hThreadDead;       // Handle of "Thread dead" event
        pepperl_fuchs::R2000Driver m_oR2000Driver;
  private:
        pthread_t   m_R2000ScanThread[4] = {0};
        std::string m_LaserScannerIp;
        int         m_iLaserPort;
        int         m_iScanFrequency;
        int         m_iSamplesPerScan;
        int         m_iNetType; //1:Tcp 2:Udp
};
#endif // CMAPPING_H
