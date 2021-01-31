//
//   The interface of class "CSensorFamily".
//

#include"sensor/SensorFamily.h"
#include <fstream>
#include "json/json.h"
#include"sensor/Sick581LaserScanner.h"
#include "blackboxhelper.hpp"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

#define DAEMON_THREAD_CTRL_CYCLE     1000  //1000ms

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSensorFamily".
namespace sensor {

CSensorFamily::CSensorFamily()
{
    sensor_family.clear();
    m_aCount = 0;
    m_bStarted = false;
}

CSensorFamily::~CSensorFamily()
{
    Clear();
}

void CSensorFamily::Clear()
{
    for (unsigned int i = 0; i < sensor_family.size(); i++)
    {
        if (sensor_family[i] != NULL) {
            delete sensor_family[i];
            sensor_family[i] = NULL;
        }
    }
    sensor_family.clear();
}

bool CSensorFamily::LoadLaserParam()
{
    bool bRet = true;

    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm)
    {
        return false;
    }

    if(Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        sensor_family.clear();
        if (!LaserParmRoot["laser"].isNull())
        {
            m_aCount = LaserParmRoot["laser"].size();
        }

        for(int i = 0; i < m_aCount.load(); i++)
        {
            CLaserScannerParam* parm = new CLaserScannerParam();
            CSensorData* data = new CSensorData();
            if(parm == NULL || data == NULL) {
                bRet = false;
                continue;
            }

            parm->LaserId = i;

            if (!LaserParmRoot["laser"][i]["State"].isNull()) {
                parm->state = LaserParmRoot["laser"][i]["State"].asBool();
            }

            if (!LaserParmRoot["laser"][i]["IP"].isNull()) {
                parm->strIP = LaserParmRoot["laser"][i]["IP"].asString();
            }

            if (!LaserParmRoot["laser"][i]["hostIP"].isNull()) {
                parm->hostIP = LaserParmRoot["laser"][i]["hostIP"].asString();
            }

            if (!LaserParmRoot["laser"][i]["LaserProductor"].isNull()) {
                parm->LaserProductor = LaserParmRoot["laser"][i]["LaserProductor"].asInt();
            }

            if (!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                parm->m_fStartAngle = LaserParmRoot["laser"][i]["StartAngle"].asDouble()/180.0*PI;
            }

            if (!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                parm->m_fEndAngle = LaserParmRoot["laser"][i]["EndAngle"].asDouble()/180.0*PI;
            }

            if (!LaserParmRoot["laser"][i]["LaserCount"].isNull()) {
                parm->m_nLineCount = LaserParmRoot["laser"][i]["LaserCount"].asInt();
            }

            if (!LaserParmRoot["laser"][i]["x"].isNull()) {
                parm->m_pst.x = LaserParmRoot["laser"][i]["x"].asDouble() /** 1000*/;
            }

            if (!LaserParmRoot["laser"][i]["y"].isNull()) {
                parm->m_pst.y = LaserParmRoot["laser"][i]["y"].asDouble() /** 1000*/;
            }

            if (!LaserParmRoot["laser"][i]["thita"].isNull()) {
                parm->m_pst.fThita = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
            }

            parm->m_fRefViewAngle = CAngle::NormAngle(parm->m_fEndAngle - parm->m_fStartAngle);

            if (!LaserParmRoot["laser"][i]["MaxRange"].isNull()) {
                parm->m_fMaxRange = LaserParmRoot["laser"][i]["MaxRange"].asDouble();
            }

            if (!LaserParmRoot["laser"][i]["MinRange"].isNull()) {
                parm->m_fMinRange = LaserParmRoot["laser"][i]["MinRange"].asDouble();
            }

            int nRangeCount = 0;
            float fRange[2] = {0.0};
            if (!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                nRangeCount = LaserParmRoot["laser"][i]["VisualRange"].size();
            }

            for(int j = 0; j < nRangeCount; j++)
            {
                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    fRange[0] = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble() / 180 * PI;
                }
                else {
                    fRange[0] = 0.0;
                }

                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    fRange[1] = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble() / 180 * PI;
                }
                else {
                    fRange[1] = 0.0;
                }
                CRange range(fRange[0], fRange[1]);
                parm->m_AppAngleRange.push_back(range);
            }

            //
            data->parm = parm;
            data->scanner = NULL;
            sensor_family.push_back(data);
        }
    }
    FileLaserParm.close();

    return bRet;
}

// the daemon thread
void CSensorFamily::SupportRoutineProxy()
{
    // 通信超时后重联
    if(sensor_family.empty()){
        return;
    }
    for (unsigned int i = 0; i < sensor_family.size(); i++){
        if (sensor_family[i] == NULL || sensor_family[i]->scanner == NULL || sensor_family[i]->parm == NULL
                || !sensor_family[i]->parm->state){
            continue;
        }
        unsigned long long time_now = GetTickCount();
        unsigned long long time_raw = sensor_family[i]->scanner->GetRawTimeStamp();
        unsigned long long connect_time = sensor_family[i]->scanner->m_nConnectTime;
        std::string str_ip = sensor_family[i]->parm->strIP;
        std::string host_ip = sensor_family[i]->parm->hostIP;
        int laser_id = sensor_family[i]->parm->LaserId;
        int ping_ack = 0;
        if(labs(static_cast<long int>(time_now - time_raw)) > SCANNER_RECONNECT_TIME
                && labs(static_cast<long int>(time_now - connect_time)) > SCANNER_CONNECT_SPAN)
        {
            // ping ok
            ping_ack = 1;//
            m_Ping.Ping(str_ip);//没有ping 暂时去掉 zjc
            //cout<<"ping: "<<i<<": "<<ping_ack<<endl;
            if(/*sensor_family[i]->parm->LaserProductor != 2 &&*/ ping_ack == 1){
                // stop the scanner thread
                sensor_family[i]->scanner->Stop();
                usleep(100 * 1000);

                // start the scanner thread
                bool bRet = sensor_family[i]->scanner->Start(str_ip.c_str(), host_ip.c_str(), laser_id);
                usleep(100 * 1000);

                cout<<"Reconnect laser"<<endl;
#ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "the scanner reconnect,i=", i, ",Ret=", (int)bRet);
#endif
            }
            else {
            }
        }
    }
}

bool CSensorFamily::Initialize()
{
    bool bRet = true;

    if(!LoadLaserParam()) {
        return false;
    }

    if(sensor_family.empty()) {
        return false;
    }

    for(int i = 0; i < sensor_family.size(); i++)
    {
        if(sensor_family[i]->parm == NULL) {
            bRet = false;
            continue;
        }

        switch (sensor_family[i]->parm->LaserProductor)
        {
        case 0:
        {
            cPfR2000LaserScanner* pScanner_Pf = new cPfR2000LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                         sensor_family[i]->parm->m_fStartAngle,
                                                                         sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Pf == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Pf;
            break;
        }

        case 1:
        {
            cHokuyoLaserScanner* pScanner_Hky = new cHokuyoLaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                        sensor_family[i]->parm->m_fStartAngle,
                                                                        sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Hky == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Hky;
            break;
        }

        case 2:
        {
            sick::cSickSafetyLaserScanner* pScanner_Sick = new sick::cSickSafetyLaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                                             sensor_family[i]->parm->m_fStartAngle,
                                                                                             sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Sick == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Sick;
            break;
        }

        case 3:
        {
            cSick581LaserScanner * pScanner_Sick = new cSick581LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                            sensor_family[i]->parm->m_fStartAngle,
                                                                            sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Sick == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Sick;
            break;
        }

        default:
            break;
        }

        // start the scanner thread
        std::string str_ip = sensor_family[i]->parm->strIP;
        std::string host_ip = sensor_family[i]->parm->hostIP;
        // ping the scanner
        int ping_ack =1;
        m_Ping.Ping(str_ip);//没有ping暂时去掉 zjc
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "ping ,i=", i, " , ", ping_ack);
#endif
        if(sensor_family[i]->parm->state && sensor_family[i]->scanner != NULL && (ping_ack == 1)){
            //cout<<"connect laser"<<endl;
            sensor_family[i]->scanner->Start(str_ip.c_str(), host_ip.c_str(), sensor_family[i]->parm->LaserId);
        }
    }

    // 延迟１秒
    usleep(1000 * 1000);
    // create the daemon thread
    if(!CreateThread(DAEMON_THREAD_CTRL_CYCLE)){
        bRet = false;
    }

    m_bStarted = true;
    return bRet;
}

unsigned int CSensorFamily::GetCount()
{
    unsigned int count = static_cast<unsigned int>(sensor_family.size());
    return count;
}

bool CSensorFamily::GetState(unsigned int index)
{
    bool bRet = false;
    if(sensor_family.empty()) {
        return false;
    }

    if(index < GetCount()) {
        if(NULL != sensor_family[index]->parm) {
            bRet = sensor_family[index]->parm->state;
        }
    }
    return bRet;
}

bool CSensorFamily::DataReady(unsigned int index)
{
    bool bRet = false;
    if(sensor_family.empty()) {
        return false;
    }

    if(index < GetCount()) {
        if(NULL != sensor_family[index]->scanner) {
            bRet = sensor_family[index]->scanner->DataReady();
        }
    }
    return bRet;
}

bool CSensorFamily::Stop()
{
    if (!m_bStarted)
        return false;

    // close the communicate thread
    for (unsigned int i = 0; i < sensor_family.size(); i++){
        if (sensor_family[i] != NULL && sensor_family[i]->scanner != NULL) {
            sensor_family[i]->scanner->Stop();
        }
    }

    // stop the daemon thread.
    StopThread();

    m_bStarted = false;
    return true;
}

bool CSensorFamily::GetPointCloud(unsigned int index,int*& pDist, int*& pIntensity)
{
    bool bRet = false;
    if(sensor_family.empty() || index >= GetCount()) {
        return false;
    }

    if(sensor_family[index]->scanner != NULL) {
        bRet = sensor_family[index]->scanner->GetPointCloud(pDist, pIntensity);
    }

    return bRet;
}

bool CSensorFamily::GetRawPointCloud(unsigned int index, std::shared_ptr<sensor::CRawPointCloud>& pCloud)
{
    bool bRet = false;
    if(sensor_family.empty() || index >= GetCount()) {
        return false;
    }

    if(sensor_family[index]->scanner != NULL) {
        bRet = sensor_family[index]->scanner->GetRawPointCloud(pCloud);
    }
    //cout<<"pcloud: "<<pCloud->distance.size()<<endl;//test

    return bRet;
}

CSensorData* CSensorFamily::GetSensorData(unsigned int index)
{
    if(sensor_family.empty() || index >= GetCount()) {
        return NULL;
    }

    return sensor_family[index];
}

bool CSensorFamily::IsBlocked()
{
    if(sensor_family.empty()) {
        return false;
    }
    for (unsigned int i = 0; i < sensor_family.size(); i++){
        if (sensor_family[i] != NULL && sensor_family[i]->scanner != NULL) {
            if(sensor_family[i]->scanner->IsBlocked()) {
                return true;
            }
        }
    }
    return false;
}

} // namespace sensor
