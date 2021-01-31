//                           - PfR2000LaserScanner.cpp -
//
//   The interface of class "cHokuyoLaserScanner".
//
//   Author: sfe1012
//   Date:   2019. 01. 28
//


#include"sensor/SickSafetyLaserScanner.h"

//#include"LinuxSetting.h"

//#include"Project.h"


//void* SickSafetyLaserScanProc(LPVOID pParam)
//{
//     sick::cSickSafetyLaserScanner* oLaserScanner = (sick::cSickSafetyLaserScanner*)pParam;
//     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
//     {
//        oLaserScanner->SupportMeasure();
//        Sleep(20);
//       // cout <<"Scanproc alive"<<endl;
//     }
//     SetEvent(oLaserScanner->m_hThreadDead);

//   return NULL;
//}

namespace sick {

cSickSafetyLaserScanner::cSickSafetyLaserScanner(float fAngRes, float fStartAng, float fEndAng):
    CRangeScanner(fAngRes, fStartAng, fEndAng, SICK)
//  : m_initialised(false)
//  , m_time_offset(0.0)
//  , m_range_min(0.0)
//  , m_range_max(0.0)
//  , m_angle_offset(-90.0)
//  , m_use_pers_conf(false)
{
    m_time_offset = 0.0f;
    m_range_min = m_range_max = 0.0f;
    m_angle_offset = -90.0f;
    m_use_pers_conf = false;
    m_nDequeCount = 0;
    //readParameters();
    m_bStarted = false;
}

bool cSickSafetyLaserScanner::readParameters()
{
   // return true;
    //load form LaserParm.jason
 //std::string sensor_ip_adress = "192.168.1.102";
//  if (!m_private_nh.getParam("sensor_ip", sensor_ip_adress))
//  {
//    //    sensor_ip_adress = sick_safetyscanners::SickSafetyscannersConfiguration_sensor_ip;
//    ROS_WARN("Using default sensor IP: %s", sensor_ip_adress.c_str());
//  }
// m_communication_settings.setSensorIp(sensor_ip_adress);


 // std::string host_ip_adress = "192.168.1.101";
//  if (!m_private_nh.getParam("host_ip", host_ip_adress))
//  {
//    ROS_WARN("Using default host IP: %s", host_ip_adress.c_str());
//  }
 // m_communication_settings.setHostIp(host_ip_adress);

 // int host_udp_port = 6060;
//  if (!m_private_nh.getParam("host_udp_port", host_udp_port))
//  {
//    ROS_WARN("Using default host UDP Port: %i", host_udp_port);
//  }
  //m_communication_settings.setHostUdpPort(host_udp_port);

//  ROS_WARN("If not further specified the default values for the dynamic reconfigurable parameters "
//           "will be loaded.");


//  int channel = 0;
//  m_private_nh.getParam("channel", channel);
 // m_communication_settings.setChannel(channel);

//  bool enabled;
//  m_private_nh.getParam("channel_enabled", enabled);
 // m_communication_settings.setEnabled(enabled);

  int skip = 0;
//  m_private_nh.getParam("skip", skip);
  m_communication_settings.setPublishingFrequency(skip + 1);

// float angle_start = 0.0f;
//  m_private_nh.getParam("angle_start", angle_start);

 // float angle_end = 0.0f;
//  m_private_nh.getParam("angle_end", angle_end);

//  // Included check before calculations to prevent rounding errors while calculating
//  if (angle_start == angle_end)
//  {
//    m_communication_settings.setStartAngle(sick::radToDeg(0));
//    m_communication_settings.setEndAngle(sick::radToDeg(0));
//  }
//  else
//  {
//    m_communication_settings.setStartAngle(sick::radToDeg(angle_start) - m_angle_offset);
//    m_communication_settings.setEndAngle(sick::radToDeg(angle_end) - m_angle_offset);
//  }

//  bool general_system_state;
//  m_private_nh.getParam("general_system_state", general_system_state);

//  bool derived_settings;
//  m_private_nh.getParam("derived_settings", derived_settings);

//  bool measurement_data;
//  m_private_nh.getParam("measurement_data", measurement_data);

//  bool intrusion_data;
//  m_private_nh.getParam("intrusion_data", intrusion_data);

//  bool application_io_data;
//  m_private_nh.getParam("application_io_data", application_io_data);

//  m_communication_settings.setFeatures(
//    general_system_state, derived_settings, measurement_data, intrusion_data, application_io_data);

//  m_private_nh.getParam("frame_id", m_frame_id);

//  m_private_nh.getParam("use_persistent_config", m_use_pers_conf);

  return true;
//}
}

void cSickSafetyLaserScanner::receivedUDPPacket(const sick::datastructure::Data& data)
{
    if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
    {
        vector<float> distence;
        vector<float> intensity;
        uint16_t num_scan_points = data.getDerivedValuesPtr()->getNumberOfBeams();
        std::vector<sick::datastructure::ScanPoint> scan_points =
                data.getMeasurementDataPtr()->getScanPointsVector();

        std::cout <<"num: "<< num_scan_points << std::endl;

        for (uint16_t i = 0; i < num_scan_points; ++i)
        {
            const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
            float a,b;

            a = static_cast<float>(scan_point.getDistance()) *
                    data.getDerivedValuesPtr()->getMultiplicationFactor(); // mm -> m
            b = static_cast<float>(scan_point.getReflectivity());
            if(scan_point.getReflectorBit())
                b = 255;
            if(i == 0)
                std::cout <<"i: "<< i << " nDist: " << a <<  ", " << "nIntensity: " << b << std::endl;

            distence.push_back(a);
            intensity.push_back(b);
        }
        unsigned long long raw_time = GetTickCount();
        AddRawPointCloud(distence, intensity, raw_time, raw_time);
//        m_CritSection.Lock();
//        if(m_nDequeCount > 1)
//        {
//            pf.pop_front();
//            pfAmplitude.pop_front();
//            pf.push_back(distence);
//            pfAmplitude.push_back(intensity);
//            m_nDequeCount = 2;
//        }
//        else
//        {
//            pf.push_back(distence);
//            pfAmplitude.push_back(intensity);
//            m_nDequeCount += 1;
//        }
//        m_CritSection.Unlock();
    }
//    else
//    {
//        pf.clear();
//        pfAmplitude.clear();
//        m_nDequeCount = 0;
//    }
}

void cSickSafetyLaserScanner::readTypeCodeSettings()
{
  //ROS_INFO("Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(m_communication_settings, type_code);
  m_communication_settings.setEInterfaceType(type_code.getInterfaceType());
  m_range_min = 0.1;
  m_range_max = type_code.getMaxRange();
}

void cSickSafetyLaserScanner::readPersistentConfig()
{
  //ROS_INFO("Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(m_communication_settings, config_data);
  m_communication_settings.setStartAngle(config_data.getStartAngle());
  m_communication_settings.setEndAngle(config_data.getEndAngle());
}

BOOL cSickSafetyLaserScanner::Start(const char*device_name,
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
    m_nLaserId = laserid;
    m_nFrequency = iScanFrequency;
    m_nConnectTime = GetTickCount();

    std::string device_ip_adress = device_name;
    m_communication_settings.setSensorIp(device_ip_adress);
    std::string host_ip_adress = host_name;
    m_communication_settings.setHostIp(host_ip_adress);

    m_communication_settings.setPublishingFrequency(2);

    std::cout << "start nano3" << std::endl;
    Start();

    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hKillThread != NULL);
    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hThreadDead != NULL);

//    switch (laserid)
//    {
//    case 0:
//        if(pthread_create(&m_SickSafetyScanThread[0],NULL,SickSafetyLaserScanProc,(void *)this) != 0)
//        {
//            std::cout<<"Creat SickSafetyLaserScanSupport Pthread Failed"<<std::endl;
//            return FALSE;
//        }
//        else
//            return TRUE;

}

bool cSickSafetyLaserScanner::Start()
{
    //cout<<"start"<<endl;
    // tcp port can not be changed in the sensor configuration, therefore it is hardcoded
    m_communication_settings.setSensorTcpPort(2122);

    m_device = std::make_shared<sick::SickSafetyscannersBase>(
      boost::bind(&cSickSafetyLaserScanner::receivedUDPPacket, this, _1), &m_communication_settings);
    m_device->run();
//    m_get_raw_data_thread_ptr.reset(receivedUDPPacket());
//    m_get_raw_data_thread_ptr.join();
//    readTypeCodeSettings();

//    if (m_use_pers_conf)
//    {
//      readPersistentConfig();
//    }

//    m_device->changeSensorSettings(m_communication_settings);
    m_bStarted = true;
    readTypeCodeSettings();
    m_device->changeSensorSettings(m_communication_settings);

    return m_bStarted;
}

bool cSickSafetyLaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    if(!m_device){
        m_device.reset();
        m_device = nullptr;
    }

    m_bStarted = false;
    return true;
}

//BOOL cSickSafetyLaserScanner::UpdateRangeData()
//{
//    vector<float>RData;
//    vector<float>RIntensityData;

//    //BOOL bRet = FALSE;

//   // m_CritSection.Lock();
//    if(!pf.empty())
//    {
//        RData = pf.front();
//        RIntensityData = pfAmplitude.front();

//        if(bFirst)
//        {
//            bFirst = FALSE;
//            RDataTemp = RData;
//            RIntensityDataTemp = RIntensityData;
//        }
//        else
//        {
//            if( RDataTemp == RData && RIntensityDataTemp == RIntensityData)
//            {
//                if(m_iLaserDataLostCount > 2)//Laser Data Lost 3 time
//                {
//                    bLaserEvent = true;
//                    return false;//zhao 20200212
////                  std::cout<<"All Same"<<std::endl;
//                }
//                m_iLaserDataLostCount++;
//            }
//            else
//            {
//                m_iLaserDataLostCount = 0;
//                bLaserEvent = false;
////                std::cout<<"Not All Same"<<std::endl;
//            }
//            RDataTemp = RData;
//            RIntensityDataTemp = RIntensityData;
//        }

//        int datasize = RData.size();
//        for(int i = 0; i < datasize; i++)
//        {
//            m_nDist[i] = (int)(RData[i]*1000);
//            m_nIntensityData[i] = (int)(RIntensityData[i]);
//            //std::cout <<"i: "<< i << " nDist: " << m_nDist[i] <<  ", " << "nIntensity: " <<  m_nIntensityData[i] << std::endl;
//        }
//       // bRet = TRUE;
////        int s = RData.size();
////        int StartP = (s - m_nPointCount) / 2;
////        int EndP = s - StartP;
////        int n = 0;
////        int m = 0;
////        for(vector<unsigned int>::const_iterator it = RData.begin(); it != RData.end(); ++it)
////        {
////            if(n > (StartP + 1) && n < (EndP - 1))
////            {
////                m_nDist[m] = *it;
////                m++;
////            }
////            n++;
////        }
////        RData.clear();
////        n = 0, m = 0;
////        for(vector<unsigned int>::const_iterator it1 = RIntensityData.begin(); it1 != RIntensityData.end(); ++it1)
////        {
////            if(n > (StartP + 1) && n < (EndP - 1))
////            {
////                m_nIntensityData[m] = *it1;
////                m++;
////            }
////            n++;
////        }
////        RIntensityData.clear();
////        bRet = TRUE;
//    }
//  //  m_CritSection.Unlock();

//    return true;
//}

//BOOL cSickSafetyLaserScanner::DataReady()
//{
////    m_CritSection.Lock();
////    if (!(pf.empty() && pfAmplitude.empty()))
////    {
////        m_CritSection.Unlock();
////        return TRUE;
////    }
////    m_CritSection.Unlock();
////    return FALSE;
//   // m_CritSection.Lock();
//    if (!(pf.empty() && pfAmplitude.empty()))
//    {
//        return TRUE;
//    }
//   // m_CritSection.Unlock();
//    return FALSE;
//}

}//namespace sick
