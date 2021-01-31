#include <stdafx.h>
#include "sensor/RangeScanner.h"
//#include "LineFeatureSet.h"
//#include "blackboxhelper.hpp"
//#include "Project.h"

//#define SAFE_KUICK_SUING 1

bool bLaserEvent = false;

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

///////////////////////////////////////////////////////////////////////////////
//   定义“CRangeScanner”。

//
//   构造函数。
//
CRangeScanner::CRangeScanner(int PointCount, float fStartAng, float fEndAng, short uType)
{
    m_nPointCount = PointCount;
	m_fStartAng = fStartAng;
	m_fEndAng = fEndAng;
    m_uType = uType;

	// 计算每个扫描周期内的采样点数量
    m_fAngRes = (float)m_nPointCount / CAngle::NormAngle(m_fEndAng - m_fStartAng) * PI / 180;

	// 根据采样点数量一次性分配缓冲区
//	m_fDist = new float[m_nPointCount];
	m_nDist = new int[m_nPointCount];
	m_nIntensityData = new int[m_nPointCount];

#ifdef SAFE_KUICK_SUING
    m_nDistTemp = new int[m_nPointCount];
    m_nIntensityDataTemp =new int[m_nPointCount];
#else
    m_nDistTemp = NULL;
    m_nIntensityDataTemp = NULL;
#endif

#if 0
	// 为扫描结果数据分配存储空间
	m_pScan = new CScan(m_nPointCount);
#endif

	ClearErrorCode();

    m_aCloudCount = 0;
    m_pClouds.clear();
    m_nConnectTime = 0;
    m_bStarted = false;
}

//
//   析构函数。
//
CRangeScanner::~CRangeScanner()
{
//    if (m_fDist != NULL)
//    {
//        delete []m_fDist;
//        m_fDist = NULL;
//    }
	if (m_nDist != NULL)
    {
        delete []m_nDist;
        m_nDist = NULL;
    }

#if 0
	if (m_pScan != NULL)
    {
		delete m_pScan;
        m_pScan = NULL;
    }
#endif

	if(m_nIntensityData != NULL)
    {
		delete []m_nIntensityData;
        m_nIntensityData = NULL;
    }
    if (m_nDistTemp != NULL)
    {
        delete []m_nDistTemp;
        m_nDistTemp = NULL;
    }
    if(m_nIntensityDataTemp != NULL)
    {
        delete []m_nIntensityDataTemp;
        m_nIntensityDataTemp = NULL;
    }

    for (unsigned int i = 0; i < m_pClouds.size(); i++)
    {
        if(m_pClouds[i] != nullptr) {
            m_pClouds[i].reset();
            m_pClouds[i] = nullptr;
        }
    }
    m_pClouds.clear();
    m_aCloudCount = 0;
}

//
//   更新距离测量值。
//
bool CRangeScanner::UpdateRangeData()
{
#if 0
    vector<unsigned int>RData;
    vector<unsigned int>RIntensityData;

    std::lock_guard<std::mutex> lock(range_mtx);

    if(!pf.empty())
    {
        RData = pf.front();
        RIntensityData = pfAmplitude.front();

        if(bFirst)
        {
            bFirst = FALSE;
            RDataTemp = RData;
            RIntensityDataTemp = RIntensityData;
        }
        else
        {
            if( RDataTemp == RData && RIntensityDataTemp == RIntensityData)
            {
                if(m_iLaserDataLostCount > 2)//Laser Data Lost 3 time
                {
                    bLaserEvent = true;
                }
                m_iLaserDataLostCount++;
            }
            else
            {
                m_iLaserDataLostCount = 0;
                bLaserEvent = false;
            }
            RDataTemp = RData;
            RIntensityDataTemp = RIntensityData;
        }


        //FIXME: why discard the front two points and the last point ???
        int s = RData.size();
        int StartP = (s - m_nPointCount) / 2;
        int EndP = s - StartP;
        int n = 0;
        int m = 0;

        for(vector<unsigned int>::const_iterator it = RData.begin(); it != RData.end(); ++it)
        {
            if(n > (StartP + 1) && n < (EndP - 1))
            {
                m_nDist[m] = *it;
                m++;
            }
            n++;
        }
        RData.clear();
        n = 0, m = 0;
        for(vector<unsigned int>::const_iterator it1 = RIntensityData.begin(); it1 != RIntensityData.end(); ++it1)
        {
            if(n > (StartP + 1) && n < (EndP - 1))
            {
                m_nIntensityData[m] = *it1;
                m++;
            }
            n++;
        }
        RIntensityData.clear();

        return TRUE;
    }
#endif
    return false;
}

bool CRangeScanner::DataReady()
{
    std::lock_guard<std::mutex> lock(range_mtx);
    if(m_pClouds.empty()) {
        return false;
    }

    if (!m_pClouds.back()->distance.empty() && !m_pClouds.back()->intensity.empty()) {
        return true;
    }
    return false;
}

bool CRangeScanner::IsBlocked()
{
    std::lock_guard<std::mutex> lock(range_mtx);
    if(m_pClouds.empty()) {
#ifdef USE_BLACK_BOX
       FILE_BlackBox(LocBox, "m_pClouds.empty!!!");
#endif
        return true;
    }

    unsigned long long tmNow = GetTickCount();
    if(tmNow - m_pClouds.back()->timestamp_raw > POINT_CLOUD_TIMEOUT) {
#ifdef USE_BLACK_BOX
       FILE_BlackBox(LocBox, "timeout!!!");
#endif
        return true;
    }
    return false;
}

unsigned long long CRangeScanner::GetRawTimeStamp()
{
    std::lock_guard<std::mutex> lock(range_mtx);
    if(m_pClouds.empty()) {
        return 0;
    }
    unsigned long long time_stamp = m_pClouds.back()->timestamp_raw;
    return time_stamp;
}

//
//   取得扫描结果点云数据.
//
bool CRangeScanner::GetPointCloud(int*& pDist, int*& pIntensity)
{
    // 先获取最新一次扫描的极径数据
    if (!UpdateRangeData())
    {
        SetErrorCode(URG_ERROR_CODE_LADAR_DATA_ERROR);
        return false;
    }


    ClearErrorCode();

    //FIXME:临时处理代码
     // 此时采样得到的极坐标数据都已保存在m_nDist缓冲区中，现根据估算姿态求出迪卡尔坐标
    pDist = m_nDist;
    pIntensity = m_nIntensityData;

    // 获取扫描数据成功
    return true;
}

bool CRangeScanner::GetRawPointCloud(std::shared_ptr<sensor::CRawPointCloud>& pCloud)
{
    std::lock_guard<std::mutex> lock(range_mtx);
	//m_CritSection.Lock();
    if(m_pClouds.empty()) {
        return false;
    }

    pCloud = m_pClouds.back();
	//m_CritSection.Unlock();
    return true;
}

#if 0
//
//   进行虚拟扫描采样(此函数暂时未被使用)。
//
BOOL CRangeScanner::VirtualScan(CSlamPosture& pstAssumed, CLineFeatureSet& LineFeatureSet)
{
	int nDetectCount = 0;

	for (int i = 0; i < m_nPointCount; i++)
	{
		CSlamAngle ang(pstAssumed.fThita + m_fStartAng + i * m_fAngRes);
		CSlamLine ScanLine(pstAssumed.GetPntObject(), ang, DEFAULT_SCAN_MAX_RANGE);

		BOOL bFound = FALSE;
		float fMinDist = DEFAULT_SCAN_MAX_RANGE;
		CSlamPoint2d ptClosest;

		for (int j = 0; j < LineFeatureSet.m_nCount; j++)
		{
			float fDist;
			CSlamPoint2d pt;
			CSlamLine& ln = LineFeatureSet.m_pLines[j];
			if (ScanLine.IntersectLineAt(ln, pt, fDist))
			{
				if (fDist < fMinDist)
				{
					bFound = TRUE;
					ptClosest = pt;
					fMinDist = fDist;
				}
			}
		}

		// 如果发现扫描返回点，则记录最近的那个点
		if (bFound)
			m_fDist[nDetectCount] = fMinDist;
		else
			m_fDist[nDetectCount] = DEFAULT_SCAN_MAX_RANGE;

		m_nDist[nDetectCount] = (int)(m_fDist[nDetectCount]);

		TRACE(_T("%.3f "), m_fDist[nDetectCount]/1000.0f);
		nDetectCount++;
	}

	TRACE(_T("\n"));
	return TRUE;
}
#endif


#if 0
//
//   取得扫描结果数据：1. 先取得极径数据; 2. 计算各点的迪卡尔坐标; 3. 提取直线特征。
//
CScan* CRangeScanner::GetScanData(CPostureGauss& pstAssumed, float& ViewAngle)
{
	// 先获取最新一次扫描的极径数据
	if (!UpdateRangeData())
	{
		SetErrorCode(URG_ERROR_CODE_LADAR_DATA_ERROR);

		return NULL;
	}


	ClearErrorCode();

	m_fStartAng = pstAssumed.fThita - ViewAngle / 2;
	m_fEndAng = pstAssumed.fThita + ViewAngle / 2;

#ifdef SAFE_KUICK_SUING
     m_CritSection.Lock();
     memset(m_nDistTemp,0,m_nPointCount*sizeof(int));
     memset(m_nIntensityDataTemp,0,m_nPointCount*sizeof(int));
     memcpy(m_nDistTemp,m_nDist,m_nPointCount*sizeof(int));
     memcpy(m_nIntensityDataTemp,m_nIntensityData,m_nPointCount*sizeof(int));
     m_CritSection.Unlock();
    // 此时采样得到的极坐标数据都已保存在m_nDist缓冲区中，现根据估算姿态求出迪卡尔坐标
     m_pScan->PolarRangesToScan(m_nPointCount, (const int*)m_nDistTemp, &pstAssumed,
        NULL, /*m_fStartAng, m_fEndAng*//*-ViewAngle / 2, ViewAngle/2*/-PI, PI, DEFAULT_SCAN_MAX_RANGE, (const int*)m_nIntensityDataTemp);
#else
//     m_CritSection.Lock();
     // 此时采样得到的极坐标数据都已保存在m_nDist缓冲区中，现根据估算姿态求出迪卡尔坐标
     m_pScan->PolarRangesToScan(m_nPointCount, (const int*)m_nDist, &pstAssumed,
        NULL, /*m_fStartAng, m_fEndAng*/-ViewAngle / 2, ViewAngle/2,/*-PI, PI,*/ DEFAULT_SCAN_MAX_RANGE, (const int*)m_nIntensityData);
    // m_CritSection.Unlock();
#endif
     // 生成直线特征集合
	m_pScan->CreateLineFeatures();
	//m_pScan->CreateReflectorFeatures();
    m_pScan->CreatePointFeatures();



	m_pScan->m_poseScanner.SetPosture(pstAssumed.GetPostureObject());

	// 获取扫描数据成功，返回CScan数据指针
	return m_pScan;
}

CScan* CRangeScanner::GetScanDataLocate(CPostureGauss& pstAssumed, float& ViewAngle)
{
	// 先获取最新一次扫描的极径数据
	if (!UpdateRangeData())
	{
		SetErrorCode(URG_ERROR_CODE_LADAR_DATA_ERROR);

		return NULL;
	}
	ClearErrorCode();

    m_fStartAng = /*pstAssumed.fThita*/ - ViewAngle / 2 /*+ PI / 2*/;
    m_fEndAng = /*pstAssumed.fThita +*/ ViewAngle / 2 /*+ PI / 2*/;

    //cout<<"startangle: "<<m_fStartAng<<"endangle: "<<m_fEndAng<<endl;

#ifdef SAFE_KUICK_SUING
     m_CritSection.Lock();
     memset(m_nDistTemp,0,m_nPointCount*sizeof(int));
     memset(m_nIntensityDataTemp,0,m_nPointCount*sizeof(int));
     memcpy(m_nDistTemp,m_nDist,m_nPointCount*sizeof(int));
     memcpy(m_nIntensityDataTemp,m_nIntensityData,m_nPointCount*sizeof(int));
     m_CritSection.Unlock();
    // 此时采样得到的极坐标数据都已保存在m_nDist缓冲区中，现根据估算姿态求出迪卡尔坐标
     m_pScan->PolarRangesToScanLocate(m_nPointCount, (const int*)m_nDistTemp, &pstAssumed,
        NULL, m_fStartAng, m_fEndAng/*-ViewAngle / 2, ViewAngle/2*//*-PI, PI*/, DEFAULT_SCAN_MAX_RANGE, (const int*)m_nIntensityDataTemp);
#else
    // m_CritSection.Lock();
     // 此时采样得到的极坐标数据都已保存在m_nDist缓冲区中，现根据估算姿态求出迪卡尔坐标
     m_pScan->PolarRangesToScanLocate(m_nPointCount, (const int*)m_nDist, &pstAssumed,
		NULL, m_fStartAng, m_fEndAng/*-ViewAngle / 2, ViewAngle/2*//*-PI, PI*/, DEFAULT_SCAN_MAX_RANGE, (const int*)m_nIntensityData);
    // m_CritSection.Unlock();
#endif
	// 生成直线特征集合
     m_pScan->m_poseScanner.SetPosture(0.0,0.0,0.0);
     m_pScan->CreateLineFeatures();
     //m_pScan->CreateReflectorFeatures();
     m_pScan->CreatePointFeatures();

    //m_pScan->m_poseScanner.SetPosture(pstAssumed.GetPostureObject());

	// 获取扫描数据成功，返回CScan数据指针
	return m_pScan;
}
#endif






int CRangeScanner::GetErrorCode()
{
	return m_nErrorCode;
}

void CRangeScanner::ClearErrorCode()
{
	m_nErrorCode = 0;
}

void CRangeScanner::SetErrorCode( int nErrorCode )
{
	m_nErrorCode = nErrorCode;
}



