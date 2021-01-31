//                           - SLAMPROC.H -
//
//   The interface of class "CLadarNav".
//
//   Author: Zhang Lei
//   Date:   2003. 4. 3
//

#ifndef __CSlamProc
#define __CSlamProc

#include "Tools.h"
#include "Scan.h"
#include "CoxMatcher.h"
#include "RangeScanner.h"
//#include "Totalscanmatcher.h"
#include "GlobalMap.h"
//#include "PerformanceTime.h"
#include "FeatureMap.h"
#include "Localization.h"
#include "LocalizationParam.h"
#include "LeastSquareMethod.h"
#include "PfR2000LaserScanner.h"
#include "HokuyoLaserScanner.h"

#include "KalmanFilter.h"


#define ODOMETER_SIZE 4

namespace sick {
class cSickSafetyLaserScanner;
}
//////////////////////////////////////////////////////////////////////////////
/// \brief The CSlamProc class
class DllExport CSlamProc
{
private:
    slam_vector_velocity m_vel;                //激光头在世界坐标系下的速度
    slam_vector_velocity m_KalmanVel;
    long t1, t2, t3, t4, t5, t6;
    bool bFirstT;
    bool bFirstT1;
    bool bFirstT2;
    int m_nOdmCount;
    float m_fOdm;
    long tProcLine1, tProcLine2;
    int m_nProcLineCount;
    float m_fProcLineOdm;
    int m_nInitCount;
    int m_nPositionMode;

    CKalmanFilter m_KalmanFilter;

    ////////////////mean filter//////////////
    CCriticalSection m_CritSection;
    CCriticalSection m_CriMergeScan;
    std::deque<CSlamPosture> m_PostureData;
    /////////////////////////////////////////
    ///////////PositionMsg///////////////////
    CSlamPosture pstOdm;
    CSlamPosture pstNav;
    CSlamPosture pstDiff;
    CCriticalSection m_CritSectionMsg;
    /////////////////////////////////////////
    ///////////////////reloadfiles///////////
    BOOL m_bReloadFiles;

public:

//#ifndef _MRC_LINUX32
//    cPfR2000LaserScanner*  m_pScanner;          // 指向激光头对象的指针
//#else
    CRangeScanner*  m_pScanner[4];          // 指向激光头对象的指针
   // CRangeScanner*  m_pScanner1;
    cPfR2000LaserScanner* m_pScanner_Pf[4];
    //cPfR2000LaserScanner* m_pScanner1_Pf;
    cHokuyoLaserScanner* m_pScanner_Hky[4];
    //cHokuyoLaserScanner* m_pScanner1_Hky;
   sick::cSickSafetyLaserScanner* m_pScanner_Sick[4];
    // std::shared_ptr<sick::cSickSafetyLaserScanner> m_pScanner_Sick[4];
    int LaserCount;
    LaserParm Laserparm[4];
//#endif
    float m_fOdometer[4];
    float m_fOdmDist;

public:
    CSlamProc();
    ~CSlamProc();


    ////////////////////////////////////
public:

        CScan* pMapScan[4];
        CScan* pCurScan[4];
        CScan* pMergeScan;
        CScan* pMergeScanWorld;
        CScan* pRefScan;
        CSlamPosture* pBadPos;
        CSlamPosture pstScanner;

        CLocalization Localization;
        CLocalizationParam Param;
        CFeatureMap       FeatureMap;

        CSlamPosture pstAt;
        CSlamPosture pstMatch;
        CSlamPosture pstSMatch;
        CSlamPosture pstOrgMatch;
        CSlamPosture pstGyro;

       // CPostureGauss pstLaser[2];

        CPostureGauss m_CurOdmPos;
        CPostureGauss m_LastOdmPos;
        CPostureGauss m_LastStepOdmPos;
        CPostureGauss m_OdmPos;
        float m_vel_norm;                    //里程计记录的路程
        float m_SumOdm;
        float m_SumFailOdm;
        BOOL m_bFullMapAuto;

        float m_SumAngle;
        BOOL m_GyroFlag;
        float m_fLastGyroAngle;
        float m_fCurGyroAngle;

        BOOL m_bGyroStart;
        bool m_bFirstFlagO;
        int m_nWorkModel;


        typedef struct node
        {
            CSlamPoint2d vstart;
            CSlamPoint2d vend;
        }VLines;

        VLines VTemp;

        vector<VLines>VisionLines;


    public:
        BOOL LocalizationProc();
        //void CalculateOdometer(const vector_velocity &vel, const float &fGyroAngle);
        const float* GetOdometer();
        void  MemSetOdometer();

        void LoadFiles();
        BOOL InitLadar();
        void InitPos(float x, float y, float fThita, bool random);
        void SetLadarVel(const float& vel_x, const float& vel_y, const float& vel_angle, const float& fGyroAngle);
        BOOL GetLocatePosture(float& x, float& y, float& fThita, int& PointNum, bool bNewMission, float, float&);
        void SetWorkModel(int nModel);
        int  CheckLadarVel();
        BOOL AddVehiclePosData(const CSlamPosture& pst);
        void MergeDoubleLaserData();
        void LoadLaserParam();
        void GetMergeScanWorld();
        void GetMergeScanWorld(std::vector<std::pair<float,float>>& point, std::vector<std::pair<float,float>>& line);
        void GetFeatureMap(std::vector<std::pair<float,float>>& point, std::vector<std::pair<float,float>>& line);
        void GetPositonMsg(std::map<std::string, float>& Data);
        void SetWorldMapParam(double& fUse1Or2LineDis, double& fOdmDist);
        void NeedReloadFiles(){m_bReloadFiles = true;}
    ////////////////////////////////////
};
#endif
