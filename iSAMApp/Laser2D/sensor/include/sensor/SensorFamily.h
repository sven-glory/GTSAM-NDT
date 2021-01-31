//
//   The interface of class "CSensorFamily".
//

#pragma once

#include <vector>
#include <mutex>
#include <atomic>
#include "MagicSingleton.h"
#include "sensor/RangeScanner.h"
#include "sensor/PfR2000LaserScanner.h"
#include "sensor/HokuyoLaserScanner.h"
#include "sensor/SickSafetyLaserScanner.h"
#include "sensor/ScannerParam.h"
#include "sensor/Sick581LaserScanner.h"
#include "ThreadHelper.h"
#include "CPing.h"

namespace sensor {

class CSensorData
{
public:
    int id;
    CRangeScanner* scanner;
    CLaserScannerParam* parm;

public:
    CSensorData()
    {
        id = 0;
        scanner = NULL;
        parm = NULL;
    }

    ~CSensorData()
    {
        Clear();
    }

    void Clear()
    {
        if (scanner != NULL) {
            delete scanner;
            scanner = NULL;
        }

        if (parm != NULL) {
            delete parm;
            parm = NULL;
        }
    }
};

class CSensorFamily : public CThreadHelper
{
private:
    std::mutex sensor_mtx;
    std::atomic<int> m_aCount;
    std::vector<CSensorData*> sensor_family;
    bool m_bStarted;

    CPing m_Ping;

private:
    CSensorFamily();

    void Clear();

    bool LoadLaserParam();

    friend MagicSingleton<CSensorFamily>;

protected:
    virtual void SupportRoutineProxy();

public:
    ~CSensorFamily();

    bool Initialize();

    unsigned int GetCount();

    bool GetState(unsigned int index);

    bool DataReady(unsigned int index);

    bool Stop();

    bool GetPointCloud(unsigned int index,int*& pDist, int*& pIntensity);

    bool GetRawPointCloud(unsigned int index, std::shared_ptr<sensor::CRawPointCloud>& pCloud);

    CSensorData* GetSensorData(unsigned int index);

    bool IsBlocked();
};

} // namespace sensor

using SensorFamilySingleton = MagicSingleton<sensor::CSensorFamily>;

