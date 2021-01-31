//                           - PfR2000LaserScanner.cpp -
//
//   The interface of class "cHokuyoLaserScanner".
//
//   Author: sfe1012
//   Date:   2019. 01. 28
//

#include"sensor/HokuyoLaserScanner.h"
#include"LinuxSetting.h"
#include"Project.h"


void print_data(const Urg_driver& urg,
                const vector<long>& data,
                const vector<unsigned short>& intensity,
                long time_stamp)
{
#if 1
    // Shows only the front step
//	int z;
//	for(z=-540;z<=540;z++)
//	{
   int front_index = urg.step2index(0);

    cout << data[front_index] << " [mm], "
        << intensity[front_index] << " [1], ("
        << time_stamp << " [msec])" << endl;
    //}

    //double radian = urg.index2rad(front_index);
    //long x = static_cast<long>( data[front_index] * cos(radian));
    //long y = static_cast<long>( data[front_index] * sin(radian));
    //cout << "(" << x << ", " << y << ")" << endl;


#else
    static_cast<void>(urg);

    // Prints the range/intensity values for all the measurement points
    size_t data_n = data.size();
    cout << "# n = " << data_n << ", timestamp = " << time_stamp << endl;
    for (size_t i = 0; i < data_n; ++i) {
        cout << i << ", " << data[i] << ", " << intensity[i] << endl;
    }
#endif
}

void* HokuyoLaserScanProc(LPVOID pParam)
{
     cHokuyoLaserScanner* oLaserScanner = reinterpret_cast<cHokuyoLaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportMeasure();
        Sleep(20);
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}

void* HokuyoLaserScanProc1(LPVOID pParam)
{
     cHokuyoLaserScanner* oLaserScanner1 = reinterpret_cast<cHokuyoLaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner1->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner1->SupportMeasure();
        Sleep(20);
     }
     SetEvent(oLaserScanner1->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}

void* HokuyoLaserScanProc2(LPVOID pParam)
{
     cHokuyoLaserScanner* oLaserScanner2 = reinterpret_cast<cHokuyoLaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner2->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner2->SupportMeasure();
        Sleep(20);
     }
     SetEvent(oLaserScanner2->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}

void* HokuyoLaserScanProc3(LPVOID pParam)
{
     cHokuyoLaserScanner* oLaserScanner3 = reinterpret_cast<cHokuyoLaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner3->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner3->SupportMeasure();
        Sleep(20);
     }
     SetEvent(oLaserScanner3->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}

cHokuyoLaserScanner::cHokuyoLaserScanner(int fAngRes, float fStartAng, float fEndAng):
    CRangeScanner(fAngRes, fStartAng, fEndAng, HOKUYO)
{
    m_iNetType = -1;
    m_LaserScannerIp = "";
    m_iLaserPort = 0;
    m_iScanFrequency = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
}

cHokuyoLaserScanner::~cHokuyoLaserScanner()
{
    Stop();
}

bool cHokuyoLaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    CloseHokuyoLaserConnection();

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);
    if(m_nLaserId >= 0 && m_nLaserId < 4) {
        PthreadJoin(m_R2000ScanThread[m_nLaserId]);
    }

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

BOOL cHokuyoLaserScanner::ConnectToHokuyoLaser(Connection_information &information)
{

    // Connects to the sensor
    if (!m_urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        cout << "Urg_driver::open(): "
             << information.device_or_ip_name() << ": " << m_urg.what() << endl;
        return FALSE;
    }
    m_urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);

   return TRUE;
}

BOOL cHokuyoLaserScanner::CloseHokuyoLaserConnection()
{
    std::cout << "Trying to stop capture" << std::endl;
    m_urg.stop_measurement();
    std::cout << "Stopping capture" << std::endl;
    m_urg.close();
    return TRUE;
}
// 启动激光扫描传感器
BOOL cHokuyoLaserScanner::Start(const char*device_name,
                                const char*host_name,
                                 const int laserid,
                                 const int iPort,
                                 const int iNetType,
                                 const int iScanFrequency,
                                 const int iSamplesPerScan)
{
    if (m_bStarted){
        return true;
    }

    m_LaserScannerIp = device_name;
    m_iLaserPort = iPort;
    m_iNetType = iNetType;
    m_iScanFrequency = iScanFrequency;
    m_iSamplesPerScan = iSamplesPerScan;
    m_nLaserId = laserid;
    m_nFrequency = iScanFrequency;
    Connection_information m_information(15,&device_name);
    m_nConnectTime = GetTickCount();
    // Connects to the sensor
//    Urg_driver             m_urg;

    if(!ConnectToHokuyoLaser(m_information))
    {
        std::cout<<"ConnectToHokuyoLaser Failed: "<<device_name<<std::endl;
        return FALSE;
    }
    else
    {
       std::cout<<"ConnectToHokuyoLaser OK"<<std::endl;
    }

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return FALSE;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return FALSE;

    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    switch (m_nLaserId)
    {
    case 0:
        if(pthread_create(&m_R2000ScanThread[0],&attr,HokuyoLaserScanProc,reinterpret_cast<LPVOID>(this)) != 0)
        {
            std::cout<<"Creat HokuyoLaserScanSupport Pthread Failed"<<std::endl;
            return FALSE;
        }
        else
            std::cout<<"Creat HokuyoLaserScanSupport Pthread OK"<<std::endl;
        break;
    case 1:
        if(pthread_create(&m_R2000ScanThread[1],&attr,HokuyoLaserScanProc1,reinterpret_cast<LPVOID>(this)) != 0)
        {
            std::cout<<"Creat HokuyoLaserScanSupport1 Pthread Failed"<<std::endl;
            return FALSE;
        }
        else
            std::cout<<"Creat HokuyoLaserScanSupport1 Pthread OK"<<std::endl;
        break;
    case 2:
        if(pthread_create(&m_R2000ScanThread[2],&attr,HokuyoLaserScanProc2,reinterpret_cast<LPVOID>(this)) != 0)
        {
            std::cout<<"Creat HokuyoLaserScanSupport2 Pthread Failed"<<std::endl;
            return FALSE;
        }
        else
            std::cout<<"Creat HokuyoLaserScanSupport2 Pthread OK"<<std::endl;
        break;
    case 3:
        if(pthread_create(&m_R2000ScanThread[3],&attr,HokuyoLaserScanProc3,reinterpret_cast<LPVOID>(this)) != 0)
        {
            std::cout<<"Creat HokuyoLaserScanSupport3 Pthread Failed"<<std::endl;
            return FALSE;
        }
        else
            std::cout<<"Creat HokuyoLaserScanSupport3 Pthread OK"<<std::endl;
        break;
    default:
        break;
    }
    pthread_attr_destroy(&attr);
    m_bStarted = true;

    return TRUE;
}

//
//	获取Urg Scanner数据
//
void cHokuyoLaserScanner::SupportMeasure()
{
    vector<long> data;
    vector<unsigned short> intensity;
    long time_stamp = 0;
    if (!m_urg.get_distance_intensity(data, intensity, &time_stamp))
    {
        //cout << "Urg_driver::get_distance(): " << m_urg.what() << endl;
    }
    else
    {
        //FIXME:临时用当前接收到点云数据的时间作为当前帧的产生时间
        unsigned long long raw_time = GetTickCount();
        AddRawPointCloud(data, intensity, raw_time, raw_time);
    }
}
