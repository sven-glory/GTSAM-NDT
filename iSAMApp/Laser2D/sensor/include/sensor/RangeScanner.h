#ifndef __CRangeScanner
#define __CRangeScanner

#include "Geometry.h"
#include <deque>
#include <mutex>
#include <memory>
#include <atomic>
#include "RawScan.h"
#include <afxmt.h>

#if 0
#include "Scan.h"
#endif

#if 0
class CLineFeatureSet;
#endif

#define MAX_POINT_CLOUD_COUNT       2
#define POINT_CLOUD_TIMEOUT         500     //500ms
#define SCANNER_RECONNECT_TIME      10000   //10s
#define SCANNER_CONNECT_SPAN        5000    //5s

enum LaserType { PEPPERL_FUCHS = 0, HOKUYO, SICK };

///////////////////////////////////////////////////////////////////////////////
//   定义“轮廓扫描传感器”。此类为轮廓扫描传感器类的基类，各种不同的轮廓
//   扫描传感器可在其基础上派生。
class DllExport CRangeScanner 
{
protected:
    bool    m_bStarted;

public:
	int    m_nPointCount;         // 扫描点的数量
	float  m_fAngRes;             // 扫描器的角分辨率(度)
	float  m_fStartAng;           // 扫描起始角(度)
	float  m_fEndAng;             // 扫描终止角(度)
    short  m_uType;               // 0 - pf, 1 - hokuyo, 2 - sick
    int    m_nLaserId;

//	float* m_fDist;
        int* m_nDist;
        int* m_nDistTemp;

	int* m_nIntensityData;
        int* m_nIntensityDataTemp;
        int  m_nFrequency;
#if 0
	CScan* m_pScan;               // 扫描数据缓冲区首指针
#endif


    std::deque<std::shared_ptr<sensor::CRawPointCloud>> m_pClouds;
    std::atomic<int> m_aCloudCount;
    std::mutex range_mtx;

    unsigned long long m_nConnectTime;


	#define URG_ERROR_CODE_LADAR_DATA_ERROR	(1)
        int  m_nErrorCode;
	void SetErrorCode(int nErrorCode);
        int  GetErrorCode();
	void ClearErrorCode();
        CCriticalSection m_CritSection;

protected:
	// 进行虚拟扫描采样(此函数暂时未被使用)
//	virtual BOOL VirtualScan(CPosture& pstAssume, CLineFeatureSet& LineFeatureSet);

	// 更新测得的极坐标数据
    bool UpdateRangeData();

public:
        CRangeScanner(int PointCount, float fStartAng, float fEndAng, short uType);
	~CRangeScanner();

	// 启动激光扫描传感器
        virtual BOOL Start(const char*device_name,
                           const char*host_name = NULL,
                           const int laserid = 0,
                           const int iPort = 80,
                           const int iNetType = 1,
                           const int iScanFrequency = 20/*20*/,
                           const int iSamplesPerScan = 3600)=0;
       // virtual BOOL Start(const LaserParm laserparm) = 0;

	// 停止激光扫描传感器
    virtual bool Stop() {return TRUE;}

	// 判断是否有新的(完整的)扫描数据到来
    bool DataReady();

    bool IsBlocked();

    unsigned long long GetRawTimeStamp();

        virtual bool GetPointCloud(int*& pDist, int*& pIntensity);

        template<typename T1, typename T2>
        inline bool AddRawPointCloud(std::vector<T1>& distance, std::vector<T2>& intensity,
                            unsigned long long time_raw, unsigned long long time_sync)
        {
            bool bRet = false;
            {
                std::lock_guard<std::mutex> lock(range_mtx);
                auto pCloud = std::make_shared<sensor::CRawPointCloud>();
                //std::shared_ptr<sensor::CRawPointCloud> pCloud(new sensor::CRawPointCloud());
                if(!pCloud) {
                    return false;
                }

                auto dis_size = distance.size();
                auto inten_size = intensity.size();
                pCloud->timestamp_raw = time_raw;
                pCloud->timestamp_sync = time_sync;
                pCloud->num_points = m_nPointCount;
                pCloud->laser_type = m_uType;
                pCloud->laser_id = m_nLaserId;
                pCloud->start_angle = m_fStartAng;
                pCloud->end_angle = m_fEndAng;
                pCloud->distance.reserve(dis_size);
                pCloud->intensity.reserve(inten_size);

                for(int i = 0 ; i < dis_size ; i++) {
                    pCloud->distance.push_back(static_cast<unsigned int>(distance[i]));
                }
                for(int j = 0 ; j < inten_size ;j++) {
                    pCloud->intensity.push_back(static_cast<unsigned int>(intensity[j]));
                }
				//m_CritSection.Lock();
                if(m_aCloudCount.load() >= MAX_POINT_CLOUD_COUNT) {
                    if(m_pClouds.front() != nullptr) {
                        m_pClouds.front().reset();
                        m_pClouds.front() = nullptr;
                    }
                    m_pClouds.pop_front();
                    m_pClouds.push_back(pCloud);
                    m_aCloudCount = MAX_POINT_CLOUD_COUNT;
                }
                else {
                    m_pClouds.push_back(pCloud);
                    m_aCloudCount += 1;
                }
				//m_CritSection.Unlock();

                pCloud.reset();
                pCloud = nullptr;
                //pCloud.use_count();
                bRet = true;
            }
            return bRet;
        }

        bool GetRawPointCloud(std::shared_ptr<sensor::CRawPointCloud>& pCloud);


#if 0
	// 取得扫描结果数据
	virtual CScan* GetScanData(CPostureGauss& pstAssume, float& ViewAngle);

	virtual CScan* GetScanDataLocate(CPostureGauss& pstAssume, float& ViewAngle);
#endif
};
#endif
