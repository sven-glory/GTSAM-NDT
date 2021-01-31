//                           - PfR2000LaserScanner.H -
//
//   The interface of class "cHokuyoLaserScanner".
//
//   Author: sfe1012
//   Date:   2019. 01. 28
//

#ifndef CPF_HOKUYO_LASER_SCANNER_H
#define CPF_HOKUYO_LASER_SCANNER_H

#include <deque>
#include <vector>
#include"ZTypes.h"
#include "RangeScanner.h"

#include "Urg_driver.h"
#include "Connection_information.h"
#include <iostream>
#include "math_utilities.h"

using namespace qrk;
using namespace std;


#define HUKUYO_DEFAULT_SCANNER_RESO   0.25/*0.1f*/                               // 缺省角分辨率(度)  0.25
#define HUKUYO_SCANNER_RESO_RAD       (HUKUYO_DEFAULT_SCANNER_RESO/180.f*PI)  // 缺省角分辨率(弧度)
#define HUKUYO_SCAN_COUNT             ((int)(270/HUKUYO_DEFAULT_SCANNER_RESO))
//#define MAX_LINE_NUM           2000                            // 最大直线段数量
//#define MAX_CIRCLE_NUM         1000                           // 最大圆数量
//#define MAX_DATA_COUNT_QUEUE	18							//队列中数据最大数量


#include"LinuxSetting.h"

//网络数据类型
#define TCP_DATA 1
#define UDP_DATA 2

class cHokuyoLaserScanner : public CRangeScanner
{
public:
        cHokuyoLaserScanner(int fAngRes, float fStartAng, float fEndAng);
        ~cHokuyoLaserScanner();

        virtual bool Stop();

  public:
        void    SupportMeasure();
        // 启动激光扫描传感器
        virtual BOOL Start(const char*device_name,
                           const char*host_name = NULL,
                           const int laserid = 0,
                           const int iPort = 80,
                           const int iNetType = TCP_DATA,
                           const int iScanFrequency = 20/*20*/,
                           const int iSamplesPerScan = 3600/*3600*/);

        BOOL ConnectToHokuyoLaser(Connection_information &information);
        BOOL CloseHokuyoLaserConnection();
  public:
        HANDLE      m_hKillThread;       // Handle of "Kill thread" event
        HANDLE      m_hThreadDead;       // Handle of "Thread dead" event

        // Connects to the sensor
        Urg_driver             m_urg;

  private:
        pthread_t   m_R2000ScanThread[4]= {0};
        std::string m_LaserScannerIp;
        int         m_iLaserPort;
        int         m_iScanFrequency;
        int         m_iSamplesPerScan;
        int         m_iNetType; //1:Tcp 2:Udp
};

#endif // CMAPPING_H
