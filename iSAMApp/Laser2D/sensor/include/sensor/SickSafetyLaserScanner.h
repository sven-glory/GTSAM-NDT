//                           - SickSafetyLaserScanner.H -
//
//   The interface of class "cSickSafetyLaserScanner".
//
//   Author: zhaojcchn
//   Date:   2020. 01. 06
#ifndef SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
#define SICK_SAFETYSCANNERS_SICKSAFETYSCANNERSROS_H
//
#include"ZTypes.h"
#include "RangeScanner.h"
// STD
#include <string>
#include <vector>

// Package
//#include <Sick/include/sick_safetyscanners_base/ExtendedLaserScanMsg.h>
//#include <sick_safetyscanners/FieldData.h>
//#include <sick_safetyscanners/OutputPathsMsg.h>
//#include <sick_safetyscanners/RawMicroScanDataMsg.h>
//#include <Sick/include/sick_safetyscanners_base/SickSafetyscannersConfigurationConfig.h>
//#include <../Navigation/Sick/include/sick_safetyscanners_base/SickSafetyscannersBase.h>
//#include <Sick/include/sick_safetyscanners_base/datastructure/CommSettings.h>
//#include <Sick/include/sick_safetyscanners_base/datastructure/FieldData.h>
//#include <Sick/include/sick_safetyscanners_base/datastructure/Data.h>
//#include <Sick/include/sick_safetyscanners_base/datastructure/TypeCode.h>
#include "sick_safetyscanners_base/SickSafetyscannersBase.h"
#include <cmath>


namespace sick {
class cSickSafetyLaserScanner: public CRangeScanner
{
private:
    std::shared_ptr<sick::SickSafetyscannersBase> m_device;

    sick::datastructure::CommSettings m_communication_settings;

    //dynamic_reconfigure::Server<sick_safetyscanners::SickSafetyscannersConfigurationConfig>
    //m_dynamic_reconfiguration_server;

    std::string m_frame_id;
    double m_time_offset;
    double m_range_min;
    double m_range_max;

    bool m_use_sick_angles;
    float m_angle_offset;
    bool m_use_pers_conf;
    sick::datastructure::Data m_Data;
//    pthread_t   m_SickSafetyScanThread[4];

    /*!
     * \brief Function which is called when a new complete UDP Packet is received
     * \param data, the assortment of all data from the sensor
     */
    void receivedUDPPacket(const sick::datastructure::Data& data);
    void readTypeCodeSettings();
    void readPersistentConfig();
    CCriticalSection m_CritSection;

public:
    HANDLE      m_hKillThread;       // Handle of "Kill thread" event
    HANDLE      m_hThreadDead;       // Handle of "Thread dead" event
    std::string m_LaserScannerIp;
    int         m_nLaserPort;
    int         m_nScanFrequency;
    int         m_nSamplesPerScan;
    deque<vector<float>>pf;  //distance
    deque<vector<float>>pfAmplitude; //Amplitude
    vector<float>RDataTemp;
    vector<float>RIntensityDataTemp;
    int			m_nDequeCount;			//队列中数据个数

    int   m_iLaserDataLostCount;
    BOOL        bFirst;

    cSickSafetyLaserScanner(float fAngRes, float fStartAng, float fEndAng);
    ~cSickSafetyLaserScanner();
    /*!
     * @brief Reads and verifies the ROS parameters.
     * @return True if successful.
     */
    bool readParameters();
   // void SupportMeasure();
    virtual BOOL Start(const char*device_name,
                       const char*host_name = NULL,
                       const int laserid = 0,
                       const int iPort = 80,
                       const int iNetType = 0,
                       const int iScanFrequency = 20/*20*/,
                       const int iSamplesPerScan = 3600/*3600*/);
    bool Start();
    //virtual BOOL DataReady();
    //virtual BOOL UpdateRangeData();
    virtual bool Stop();
};
}//namespace sick

#endif
