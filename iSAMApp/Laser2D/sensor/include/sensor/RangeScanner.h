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
//   ���塰����ɨ�贫������������Ϊ����ɨ�贫������Ļ��࣬���ֲ�ͬ������
//   ɨ�贫���������������������
class DllExport CRangeScanner 
{
protected:
    bool    m_bStarted;

public:
	int    m_nPointCount;         // ɨ��������
	float  m_fAngRes;             // ɨ�����ĽǷֱ���(��)
	float  m_fStartAng;           // ɨ����ʼ��(��)
	float  m_fEndAng;             // ɨ����ֹ��(��)
    short  m_uType;               // 0 - pf, 1 - hokuyo, 2 - sick
    int    m_nLaserId;

//	float* m_fDist;
        int* m_nDist;
        int* m_nDistTemp;

	int* m_nIntensityData;
        int* m_nIntensityDataTemp;
        int  m_nFrequency;
#if 0
	CScan* m_pScan;               // ɨ�����ݻ�������ָ��
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
	// ��������ɨ�����(�˺�����ʱδ��ʹ��)
//	virtual BOOL VirtualScan(CPosture& pstAssume, CLineFeatureSet& LineFeatureSet);

	// ���²�õļ���������
    bool UpdateRangeData();

public:
        CRangeScanner(int PointCount, float fStartAng, float fEndAng, short uType);
	~CRangeScanner();

	// ��������ɨ�贫����
        virtual BOOL Start(const char*device_name,
                           const char*host_name = NULL,
                           const int laserid = 0,
                           const int iPort = 80,
                           const int iNetType = 1,
                           const int iScanFrequency = 20/*20*/,
                           const int iSamplesPerScan = 3600)=0;
       // virtual BOOL Start(const LaserParm laserparm) = 0;

	// ֹͣ����ɨ�贫����
    virtual bool Stop() {return TRUE;}

	// �ж��Ƿ����µ�(������)ɨ�����ݵ���
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
	// ȡ��ɨ��������
	virtual CScan* GetScanData(CPostureGauss& pstAssume, float& ViewAngle);

	virtual CScan* GetScanDataLocate(CPostureGauss& pstAssume, float& ViewAngle);
#endif
};
#endif
