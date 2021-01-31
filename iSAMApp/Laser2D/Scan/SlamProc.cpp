//                           - SLAMPROC.CPP -
//
//   Implementation of class "CSlamProc".
//
//

#include "stdafx.h"
#include "SlamProc.h"
#include "Localization.h"
#include "SickSafetyLaserScanner.h"

//#include "PfR2000LaserScanner.h"
//#include "HokuyoLaserScanner.h"
#include <fstream>
#include "include/json/json.h"

#include "BlackBox.h"
#include "Project.h"
using namespace sick;
extern CBlackBox EventBox;
long tt1, tt2, tt3;

extern CBlackBox NavBox;
extern CBlackBox LaserBox;
extern bool bLaserEvent;

float m_MaxOdm = 0.0f;

#define SCANNER_MAX_RANGE        30000
float dRefViewAngle = PI * 2;   //PI * 3 / 2



#define  USE_KALMANFILTER
//#define  USE_NAV_BLACK_BOX
#define USE_GYROANGLE

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

void CSlamProc:: LoadLaserParam()
{
    std::ifstream FileLaserParm;//(WORK_PATH"LaserParm.json");
    FileLaserParm.open(WORK_PATH"LaserParm.json");
    cout<<strerror(errno)<<endl;
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(FileLaserParm)
    {
        if(Jreader.parse(FileLaserParm, LaserParmRoot))
        {
            LaserCount = LaserParmRoot["laser"].size();
            cout<<"lasercount1: "<<LaserCount<<endl;
            for(int i = 0; i < LaserCount; i++)
            {
                Laserparm[i].state = LaserParmRoot["laser"][i]["State"].asBool();
                Laserparm[i].LaserId = i;
                Laserparm[i].strIP = LaserParmRoot["laser"][i]["IP"].asString();
                Laserparm[i].strhostIP = LaserParmRoot["laser"][i]["hostIP"].asString();

                Laserparm[i].LaserProductor = LaserParmRoot["laser"][i]["LaserProductor"].asInt();
                Laserparm[i].StartAngle = LaserParmRoot["laser"][i]["StartAngle"].asDouble()/180.0*PI;
                Laserparm[i].EndAngle = LaserParmRoot["laser"][i]["EndAngle"].asDouble()/180.0*PI;
                Laserparm[i].Resolution = LaserParmRoot["laser"][i]["LaserCount"].asInt();
                Laserparm[i].InstallPos.x = LaserParmRoot["laser"][i]["x"].asDouble() * 1000;
                Laserparm[i].InstallPos.y = LaserParmRoot["laser"][i]["y"].asDouble() * 1000;
                Laserparm[i].InstallPos.fThita = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
                Laserparm[i].RefViewAngle = CSlamAngle::NormAngle(Laserparm[i].EndAngle - Laserparm[i].StartAngle);
                Laserparm[i].VisualRangeNum = LaserParmRoot["laser"][i]["VisualRange"].size();
                for(int j = 0; j < Laserparm[i].VisualRangeNum; j++)
                {
                    Laserparm[i].VisualRangeStart[j] = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble() / 180 * PI;
                    Laserparm[i].VisualRangeEnd[j] = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble() / 180 * PI;
                }
            }

        }
        FileLaserParm.close();
    }
}

CSlamProc::CSlamProc()
{
    LoadFiles();//20200430zjc
    for(int i = 0; i < 4; i++)
    {
        m_pScanner[i] = NULL;
        m_pScanner_Hky[i] = NULL;
        m_pScanner_Pf[i] = NULL;
        m_pScanner_Sick[i] = NULL;
    }


    ////////////////////////
    for(int i = 0; i < LaserCount; i++)
    {
        switch (Laserparm[i].LaserProductor)
        {
        case 0:
            m_pScanner_Pf[i] = new cPfR2000LaserScanner(Laserparm[i].Resolution, Laserparm[i].StartAngle, Laserparm[i].EndAngle);
            m_pScanner[i] = m_pScanner_Pf[i];
            break;
        case 1:
            m_pScanner_Hky[i] = new cHokuyoLaserScanner(Laserparm[i].Resolution, Laserparm[i].StartAngle, Laserparm[i].EndAngle);
            m_pScanner[i] = m_pScanner_Hky[i];
            break;
        case 2:
            m_pScanner_Sick[i] = new sick::cSickSafetyLaserScanner(Laserparm[i].Resolution, Laserparm[i].StartAngle, Laserparm[i].EndAngle);
            m_pScanner[i] = m_pScanner_Sick[i];

        default:
            break;
        }
    }

    for(int i = 0; i < 4; i++)
    {
        pMapScan[i] = NULL;
        pCurScan[i] = NULL;
    }
    pMergeScan = NULL;
    pMergeScanWorld = NULL;
    pRefScan = NULL;
    pBadPos = NULL;

    m_GyroFlag = TRUE;
    m_bGyroStart = FALSE;
    bFirstT = true;
    m_bFirstFlagO = true;
    m_nOdmCount = 0;
    m_nInitCount = 0;
    m_nWorkModel = 0;
    m_nPositionMode = 1;
    m_fOdm = 0;
    m_fProcLineOdm = 0.0f;
    m_nProcLineCount = 0;
    m_fOdmDist = 0.5f;
#ifdef USE_KALMANFILTER
    m_KalmanFilter.Initial();
#endif
    m_bFullMapAuto = false;
    m_bReloadFiles = false;//20200430zjc reloadfiles

    // TODO: 在此处添加构造代码


    //LJ  Add m_pRefScan
    pRefScan = new CScan();
    pRefScan->m_pLineFeatures = FeatureMap.GetLineFeatures();
    //////////////////////////
}

CSlamProc::~CSlamProc()
{
    if(pRefScan != NULL)
    {
       // pMergeScan->Clear();
        delete pRefScan;
        pRefScan == NULL;
    }

    if(pMergeScan != NULL)
    {
       // pMergeScan->Clear();
        delete pMergeScan;
        pMergeScan == NULL;
    }

    if(pMergeScanWorld != NULL)
    {
       // pMergeScan->Clear();
        delete pMergeScanWorld;
        pMergeScanWorld == NULL;
    }


    for(int i = 0; i < 4; i++)
    {
        if(pCurScan[i] != NULL)
        {
            delete pCurScan[i];
            pCurScan[i] == NULL;
        }
        if(m_pScanner[i] != NULL)
        {
            delete m_pScanner[i];
            m_pScanner[i] == NULL;
        }

        if(pMapScan[i] != NULL)
        {
            delete pMapScan[i];
            pMapScan[i] == NULL;
        }
    }

}

void CSlamProc::LoadFiles()
{
    FILE* fp2 = fopen(WORK_PATH"FeatureMap.map", "rb");
    //    FILE* fp2 = fopen(WORK_PATH"FeatureMap.txt", "rt");
    if (fp2 != NULL)
    {
        FeatureMap.Load(fp2);
        fclose(fp2);
    }

    // Step 1: 设置激光头参数
    if (!Localization.SetScannerParam(-PI, PI, SCANNER_RESO_RAD, 30000.0f))
    {
        ASSERT(FALSE);
    }

    // Step 2: 装入点特征匹配参数

    FILE* fp = fopen(WORK_PATH"SiasunMatcherParam.txt", "rt");
    if (fp == NULL || !Param.Load(fp))
    {
        ASSERT(FALSE);
    }
 //   float r = 1600.0f;
 //   fscanf(fp,"%f\n", &r);
        //return false;
 //   m_KalmanFilter.SetR(r);

    fclose(fp);

    LoadLaserParam();//20200430 zjc reload files
    Localization.SetParam(Param);

    // Step 3: 装入特征数据
    Localization.SetFeatureMap(&FeatureMap);

    //读取例外表
    fp = fopen(WORK_PATH"SpecialList.txt", "rt");
    if (fp != NULL)
    {
        Localization.LoadSpecialList(fp);
        fclose(fp);
    }
}



BOOL CSlamProc::InitLadar()
{

#ifdef USE_BLACK_BOX
    LaserBox.NewRecord();
    LaserBox <<"NAV 2020-08-24 AM 09:06 BY ZJC";
    LaserBox.EndRecord();
    cout<<"NAV 2020-08-24 AM 09:06 BY ZJC"<<endl;
#endif

    cout<<"Lasercount: "<<LaserCount<<endl;
    for(int i = 0; i < LaserCount; i++)
    {
       // bool b_connected = true;
      //  QString response;
        int j = 0;
      //  CPing ping;

        cout<<"state: "<<Laserparm[i].state<<endl;
        if(Laserparm[i].state)
        {
            for(j = 0; j < 5; j++)
            {
                if(m_pScanner[i]->Start(Laserparm[i].strIP.c_str(), 
				Laserparm[i].strhostIP.c_str(),
				Laserparm[i].LaserId))
                {
                    cout<<"connected "<<i<<endl;
                    break;
                }
                else
                {
                    cout<<"sleep5000:"<<i<<" "<<j<<endl;
                    Sleep(5000);
                }
            }
        }
        if(j > 5)
            return FALSE;
    }
    return TRUE;
}




void CSlamProc::InitPos(float x, float y, float fThita, bool random)
{

//    float thita = CSlamAngle::NormAngle(fThita /*- PI / 2*/);
    CSlamPosture InitPosture(x, y, fThita);
    pstScanner = InitPosture;
    m_CurOdmPos.GetPostureObject() = InitPosture;
    cout<<"initpos: "<<x<<" "<<y<<" "<<fThita<<endl;
    m_LastOdmPos.GetPostureObject() = InitPosture;
    Localization.ResetMatch(InitPosture);
    m_nInitCount = 0;
    m_bFullMapAuto = false;
    if(random)
        m_nOdmCount = 20;
    else
        m_nOdmCount = 15;
#ifdef USE_KALMANFILTER
    m_KalmanFilter.SetInitialPos(InitPosture);
#endif
}



BOOL CSlamProc::GetLocatePosture(float& fLocate_x, float& fLocate_y, float& fLocate_Thita, int& PointNum, bool bNewMission, float fStandardAng, float& fDiffAng)
{
   // cout<<"gogo"<<endl;
    bool noLaserUsed = TRUE;
////reload files 20200430 zjc////////////////////////
    if(m_bReloadFiles)
    {
        LoadFiles();
        m_bReloadFiles = false;
    }
/////////////////////////////////////////////////////
    for(int i = 0; i < LaserCount; i++)
    {
        if (Laserparm[i].state)
            noLaserUsed = FALSE;
    }
    if(noLaserUsed)
        return FALSE;

    // 判断是否有一组完整的轮廓数据到来
    for(int i = 0; i < LaserCount; i++)
    {
        if (Laserparm[i].state && !m_pScanner[i]->DataReady())
        {
#ifdef USE_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox << "FALSE3 ";
            LaserBox.EndRecord();
#endif
//            if(m_nLostLaserCom[i] > 20)
//                bLaserEvent = true;
            return FALSE;
        }
    }


    // 仿真采样时的激光头姿态
    CPostureGauss pstOdometry;
    pstOdometry.GetPostureObject() = m_CurOdmPos/*pstScanner*/;

    //    pMapScan = m_pScanner->GetScanData(pstOdometry, dRefViewAngle);
    if(m_nWorkModel == 1)
    {
        for(int i = 0; i< LaserCount; i++)
        {
            if(Laserparm[i].state)
            {
                CPostureGauss pos;
                pstOdometry.TransformToGlobal(&Laserparm[i].InstallPos, &pos);
                pMapScan[i] = m_pScanner[i]->GetScanData(pos, Laserparm[i].RefViewAngle);
               //改为车体坐标系，20200305
//                pMapScan[i] = m_pScanner[i]->GetScanDataLocate(Laserparm[i].InstallPos , Laserparm[i].RefViewAngle);
            }
        }
    }
    else
    {
        for(int i = 0; i< LaserCount; i++)
        {
             if(Laserparm[i].state)
             {
                 pCurScan[i] = m_pScanner[i]->GetScanDataLocate(Laserparm[i].InstallPos , Laserparm[i].RefViewAngle);
             }
        }
        MergeDoubleLaserData();
    }

    if(bNewMission)
    {
        int count = 0;
        for(int i = 0; i < pMergeScan->m_ptReflectors.size(); i++)
        {
            CSlamPoint2d pt = pMergeScan->m_ptReflectors[i];
            if(fabs(CSlamAngle::NormAngle2(pt.a - fStandardAng)) < 0.17f)
            {
                count++;
                fDiffAng = CSlamAngle::NormAngle2(pt.a - fStandardAng);
                cout << "count:" << count << ", fDiffAng: " << fDiffAng << std::endl;
            }
        }
        if(1 != count)
            fDiffAng = 1.0f;
        cout<<"output diffang: "<<fDiffAng<<endl;
    }

    // 设置里程计姿态
    if(m_nOdmCount == 20)
    {
        m_nPositionMode = FULL_MAP_MODE; //FULL_MAP_MODE
        Localization.SetLocalFullMap(false);//20200430 zjc for localfullmap
    }

    Localization.SetOdometricPosture(pstOdometry.GetPostureObject(), m_nPositionMode);

    ////////////////////////////////////////////
    // 如获取成功，则尝试进行定位
    if (pMergeScan != NULL)
    {
#ifdef USE_BLACK_BOX
#ifdef USE_NAV_BLACK_BOX
        LaserBox.NewRecord();
        LaserBox <<" PositionMode: "<< m_nPositionMode<<" ref.size: "<<(int)pMergeScan->m_ptReflectors.size();
        LaserBox.EndRecord();
#endif
#endif
        // 如定位成功
        if (LocalizationProc())
        {
            ////////////////////////////////////
            m_LastOdmPos.GetPostureObject() =  pstMatch;
            /////////////////////////////////

            m_LastOdmPos.GetPostureObject() =  pstMatch;
            //cout<<"psMatch: "<<pstMatch.x<<" "<<pstMatch.y<<" "<<pstMatch.fThita<<endl;

#ifdef USE_BLACK_BOX
#ifdef USE_NAV_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"psMatch: "<<pstMatch.x<<" "<<pstMatch.y<<" "<<pstMatch.fThita;
            LaserBox.EndRecord();
#endif
#endif

            m_nOdmCount = 0;
            m_fOdm = 0.0f;
            if(m_nInitCount < 10)
                m_nInitCount++;
            fLocate_x = pstMatch.x;
            fLocate_y = pstMatch.y;
            fLocate_Thita = pstMatch.fThita;
            PointNum = Localization.m_pSiasunMatcher->GetFeaturePairCount();
            m_nPositionMode = 1;
            bFirstT2 = true;
            t3 = GetTickCount();

            return TRUE;
        }
        else
        {
            CSlamPosture posOdm;

            float DethaTime2;
           // long t3 = GetTickCount();
            if(bFirstT2)
            {
                t4 = GetTickCount();
                DethaTime2 = ((float)(t4 - t3))/1000;
                bFirstT2 = false;
            }
            else
            {
                t3 = GetTickCount();
                if(/*m_nOdmCount == 0*/0)
                {
                    DethaTime2 = ((float)(t3 - t6))/1000;
                    t4 = t3;
                }
                else
                {
                    DethaTime2 = ((float)(t3 - t4))/1000;
                    t4 = t3;
                }

            }

            float timetest = DethaTime2;
            if(DethaTime2 > 0.3 && m_nPositionMode == 1)
            {
                DethaTime2 = 0.1;
            }
#ifdef USE_KALMANFILTER
            m_KalmanFilter.Step(m_KalmanVel, DethaTime2, m_CurOdmPos, &posOdm);  // LJ 2018.7.8
#endif

           // posOdm.fThita = m_CurOdmPos.fThita;
            m_LastOdmPos =  m_CurOdmPos;

            //cout<<"posOdm: "<<posOdm.x<<" "<<posOdm.y<<" "<<posOdm.fThita<<endl;



#ifdef USE_KALMANFILTER
            m_LastOdmPos.GetPostureObject() = posOdm;  // LJ 2018.7.8
            m_CurOdmPos.GetPostureObject() = posOdm;
#endif

            if(m_nOdmCount == 0)
                m_fOdm = 0;
            else
                m_fOdm += m_vel_norm*DethaTime2;
            if(m_MaxOdm < m_fOdm)
                m_MaxOdm = m_fOdm;
            if(m_MaxOdm > 1.0f)
                m_MaxOdm = 0.0f;

            if(m_nOdmCount < 10)
                m_nOdmCount++;
            //cout<<"m_fOdm: "<<m_fOdm<<endl;
            if(m_fOdm >= m_fOdmDist/*|| m_nOdmCount >= 24*/)
                m_nInitCount = 0;

           // cout<<"m_fOdm: "<< m_fOdm<<"  m_nInitCount: "<<m_nInitCount<<endl;
#ifdef USE_BLACK_BOX
//#ifdef USE_NAV_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"posOdm: "<<posOdm.x<<" "<<posOdm.y<<" "<<posOdm.fThita<<" "<<timetest<<"vel: "<<m_KalmanVel.vel_x<<" "<<m_KalmanVel.vel_y<<" "<<m_KalmanVel.vel_angle<<" ref: "<< (int)(pMergeScan->m_ptReflectors.size())
                       << "m_fOdm: "<< m_fOdm;
            LaserBox.EndRecord();
//#endif
#endif
#ifdef USE_BLACK_BOX
//#ifndef USE_NAV_BLACK_BOX
//            LaserBox.NewRecord();
//            LaserBox << "m_fOdm: "<< m_fOdm<<"  m_nInitCount: "<<m_nInitCount;
//            LaserBox.EndRecord();
//#endif
#endif
            if(/*m_nOdmCount < 150*/m_fOdm < m_fOdmDist && m_nOdmCount != 20 /*&& m_nInitCount > 0*/)
            {
                fLocate_x = m_CurOdmPos.x;
                fLocate_y = m_CurOdmPos.y;
                fLocate_Thita = m_CurOdmPos.fThita;
                //m_nOdmCount++;
                pstScanner = m_CurOdmPos;
                pstMatch = m_CurOdmPos;
                PointNum = Localization.m_pSiasunMatcher->GetFeaturePairCount();

                return TRUE;
            }
            else
            {
                if(m_nOdmCount == 20)
                {
                    m_nPositionMode = 0;
                    Localization.SetLocalFullMap(false);//20200430 zjc for localfullmap
                }
                else
                {
                    if(m_nPositionMode == 1 && m_nInitCount == 0 && m_vel_norm < 0.3f)//20200807加入速度控制
                    {
                        m_nPositionMode = 0;
                        m_nOdmCount = 15;
                        Localization.SetLocalFullMap(true);//20200430 zjc for localfullmap
                    }
                    else
                        m_nPositionMode = 1;
                }


                //cout<<"PositionMode: "<<m_nPositionMode<<endl;
                pstScanner = m_CurOdmPos;
                pstMatch = m_CurOdmPos;
#ifdef USE_BLACK_BOX
//#ifdef USE_NAV_BLACK_BOX
                LaserBox.NewRecord();
                LaserBox << "FALSE1 "<<" PositionMode: "<<m_nPositionMode;
                LaserBox.EndRecord();
#endif
//#endif
                return FALSE;
            }

        }
    }
#ifdef USE_BLACK_BOX
//#ifdef USE_NAV_BLACK_BOX
    LaserBox.NewRecord();
    LaserBox << "FALSE2 ";
    LaserBox.EndRecord();
//#endif
#endif
    return FALSE;
}




BOOL CSlamProc::LocalizationProc()
{
    // 先尝试用点特征进行定位
    int rc = 0;
//    CSlamPosture pstLocalMatch;
//    float dErr = 0;

    CSlamPosture pstNew, pos;
    Localization.SelectMatcher(SCAN_MATCHER_SIASUN);
    CLocalizationResult Result;
    rc = Localization.OnReceiveScan(pMergeScan, Result);
    pstNew = Result.pst;

//    if (0/*rc < 0*/)
//    {
//        // 如果点特征定位不成功，尝试用直线Cox算法
//        Localization.SelectMatcher(SCAN_MATCHER_COX);

//        //rc = Localization.OnReceiveScan(pCurScan, /*pstNew,*/ Result);
//        rc = Localization.OnReceiveScan(pMergeScan, /*pstNew,*/ Result, pRefScan);
//        pstNew = Result.pst;
//    }

    if (rc < 0)
    {
        tProcLine1 = GetTickCount();
        t6 = GetTickCount();
        return FALSE;
    }

    if (rc == 2)
    {
        pstNew.fThita = pstScanner.fThita;
        if(m_nProcLineCount == 0)
        {
            tProcLine1 = GetTickCount();
            m_fProcLineOdm = 0.0f;
            m_nProcLineCount = 1;
        }
        else
        {
            m_nProcLineCount = 1;
            tProcLine2 = GetTickCount();
            float d_time = (float)(tProcLine2 - tProcLine1)/1000.0f;
            m_fProcLineOdm += m_vel_norm * d_time;
           // cout<<"d_time: "<<d_time<<" ProcLineOdm: "<<m_fProcLineOdm<<" m_vel: "<<m_vel_norm<<endl;
#ifdef USE_BLACK_BOX
#ifdef USE_NAV_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"d_time: "<<d_time<<" ProcLineOdm: "<<m_fProcLineOdm<<" m_vel: "<<m_vel_norm;
            LaserBox.EndRecord();
#endif
#endif
            tProcLine1 = tProcLine2;
        }

        if(m_fProcLineOdm > Param.m_fUse1Or2LinesDis /*|| m_nInitCount < 5*/)
        {
            t6 = GetTickCount();
            return FALSE;
        }
    }

    CSlamPosture pstMove = pstNew - pstScanner;
    //cout<<"pstNew: "<< pstNew.x<<" "<<pstNew.y<<" "<<pstNew.fThita<<endl;
#ifdef USE_BLACK_BOX
#ifdef USE_NAV_BLACK_BOX
    LaserBox.NewRecord();
    LaserBox <<"pstNew: "<< pstNew.x<<" "<<pstNew.y<<" "<<pstNew.fThita;
    LaserBox.EndRecord();
#endif
#endif

    // 进行位置跳变检查
    float dx = pstNew.x - m_CurOdmPos.x/*pstScanner.x*/;
    float dy = pstNew.y - m_CurOdmPos.y/*pstScanner.y*/;
    float fAngErr = CSlamAngle::NormAngle2(pstNew.fThita - m_CurOdmPos.fThita);

    ////////////////PostionMsg/////////////////
    m_CritSectionMsg.Lock();
    pstNav = pstNew;
    pstOdm = m_CurOdmPos;
    pstDiff.Create(dx, dy, fAngErr);
    m_CritSectionMsg.Unlock();
    ///////////////////////////////////////////
    // 如果位置发生跳变，在此报错
    if(m_nPositionMode == 1)
    {
        //////////20200804增强局部匹配效果较好情况下的可信度zjc////////////
        if(Localization.GetMatchRatio() >= 85.0f &&(fabs(dx) > 500.0f || fabs(dy) > 500.0f|| fabs(fAngErr) > 0.2f))
        {
#ifdef USE_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"Out Of Range 1000: "<<dx<<" "<<dy<<" "<<fAngErr;
            LaserBox.EndRecord();
#endif
            return FALSE;
        }
        //////////////////////////////////////////////////////////////
        else if (Localization.GetMatchRatio() < 85.0f && (fabs(dx) > 250.0f || fabs(dy) > 250.0f|| fabs(fAngErr) > 0.2f))
        {
            cout<<"Out Of Range 250: "<<dx<<" "<<dy<<endl;
#ifdef USE_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"Out Of Range 250: "<<dx<<" "<<dy<<" "<<fAngErr;
            LaserBox.EndRecord();
#endif
            return FALSE;
        }
    }
    // 如果位置发生跳变，在此报错
    if (m_nPositionMode == 0 && m_nOdmCount == 15)
    {
        //////////20200803增强全局匹配效果较好情况下的可信度zjc////////////
        if(Localization.GetMatchRatio() >= 90.0f &&(fabs(dx) > 1000.0f || fabs(dy) > 1000.0f|| fabs(fAngErr) > 0.2f))
        {
#ifdef USE_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"Out Of Range 1000: "<<dx<<" "<<dy<<" "<<fAngErr;
            LaserBox.EndRecord();
#endif
            return FALSE;
        }
        //////////////////////////////////////////////////////////////
        else if(Localization.GetMatchRatio() < 90.0f && fabs(dx) > 550.0f || fabs(dy) > 550.0f|| fabs(fAngErr) > 0.2f)
        {
            cout<<"Out Of Range 550: "<<dx<<" "<<dy<<endl;
#ifdef USE_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"Out Of Range 550: "<<dx<<" "<<dy<<" "<<fAngErr;
            LaserBox.EndRecord();
#endif
            return FALSE;
        }
        /////////20200804//////////////////////////////////////////
        else if(Localization.GetMatchRatio() < 80.0f)
            return FALSE;
    }

    if (rc == 1)
    {
        if(m_nInitCount > 0)
        {
            m_nProcLineCount = 0;
            m_fProcLineOdm = 0.0f;
            m_nOdmCount = 0;
        }
        if(m_nInitCount == 0)
        {
/////////////////20200430zhaojc///////////////
            float tick = 0.0f;
            t4 = t4 > t6 ? t4 : t6;
            //if(m_bFullMapAuto)
            tick = float(GetTickCount() - t4) / 1000.0f;
//////////////////////////////////////////////
            m_bFullMapAuto = true;

            pstNew.x += m_KalmanVel.vel_x * tick;
            pstNew.y += m_KalmanVel.vel_y * tick;
            pstNew.fThita += m_KalmanVel.vel_angle * tick;
			m_KalmanFilter.SetInitialPos(pstNew);
            m_CurOdmPos.GetPostureObject() = pstNew;//20200430zhaojc
           // Localization.ResetMatch(pstNew);
#ifdef USE_BLACK_BOX
            LaserBox.NewRecord();
            LaserBox <<"setnewpos: "<<pstNew.x<<" "<<pstNew.y<<" "<<pstNew.fThita<<"tick: "<<tick;
            LaserBox.EndRecord();
#endif
            // bFirstT1 = true;
            t6 = GetTickCount();//20200430zhaojc
        }
    }


    float DethaTime1;
    long t5 = GetTickCount();
    if(bFirstT1)
    {
        t6 = GetTickCount();
        DethaTime1 = 0.15;
        bFirstT1 = false;
    }
    else
    {
        if(m_nOdmCount != 0)
        {
            DethaTime1 = ((float)(t5 - t4))/1000;
            t6 = t5;
        }
        else
        {
            DethaTime1 = ((float)(t5 - t6))/1000;
            t6 = t5;
        }

    }
        if(DethaTime1 > 0.3)
        {
            DethaTime1 = 0.1;
        }

#ifdef USE_BLACK_BOX1
        LaserBox.NewRecord();
        LaserBox << "pstResule: " << pstNew.x << ", " << pstNew.y << ", " << pstNew.fThita;
        LaserBox.EndRecord();
#endif
#ifdef USE_KALMANFILTER
        m_KalmanFilter.Step(m_KalmanVel, DethaTime1, pstNew, &pos);
        //pos.fThita = m_CurOdmPos.fThita;
#endif

//#ifdef USE_BLACK_BOX
//    LaserBox.NewRecord();
//    LaserBox << "pstResule: " << pos.x << ", " << pos.y << ", " << pos.fThita;
//    LaserBox.EndRecord();
//#endif

    // 采用刚测得的匹配结果，对CurScan进行坐标变换
    pMergeScan->Move(pstMove.x, pstMove.y);
    pMergeScan->Rotate(pstMove.fThita);
    pMergeScan->CreateLineFeatures();

#ifdef USE_KALMANFILTER
    pstScanner.SetPosture(pos.x, pos.y, pos.fThita);
    pstMatch.SetPosture(pos.x, pos.y, pos.fThita);
    pstOrgMatch.SetPosture(pos.x, pos.y, pos.fThita);
#endif

    return TRUE; // liu jun add 3.19 01:45
}

void CSlamProc::SetLadarVel(const float& vel_x, const float& vel_y, const float& vel_angle, const float& fGyroAngle)
{
    float fVx = vel_x;
    float fVy = vel_y;
    float fVthita = vel_angle;
    float DethaTime = 0.05; //0.125

    long t1 = GetTickCount();
    if(bFirstT)
    {
        t2 = GetTickCount();
        bFirstT = false;
    }
    DethaTime = ((float)(t1 - t2))/1000;
    t2 = t1;

    //m_ulLastVelUpdate = ulSysTimeTick;       // 记录更新时间
    m_vel_norm = (float)hypot(fVy, fVx);//sqrt(fVy * fVy + fVx * fVx);


    float speed_angle;
    if(m_nWorkModel == 1)
        speed_angle = CSlamAngle::NormAngle(atan2(fVy, fVx) + m_OdmPos.fThita + PI / 2);
    else
        speed_angle = CSlamAngle::NormAngle(atan2(fVy, fVx) + pstScanner.fThita + PI / 2);

    m_vel.vel_x = m_vel_norm * cos(speed_angle);                       // 保存速度向量
    m_vel.vel_y = m_vel_norm * sin(speed_angle);
    m_vel.vel_angle = fVthita;

    Localization.m_pSiasunMatcher->UpdateVel(m_vel.vel_x, m_vel.vel_y, m_vel.vel_angle);

   // cout<<"m_vel: "<<m_vel.vel_x<<" "<<m_vel.vel_y<<" "<<m_vel.vel_angle<<endl;


    m_KalmanVel.vel_x = m_vel.vel_x * 1000;
    m_KalmanVel.vel_y = m_vel.vel_y * 1000;
    m_KalmanVel.vel_angle = m_vel.vel_angle;


#ifdef USE_BLACK_BOX
#ifdef USE_NAV_BLACK_BOX
    LaserBox.NewRecord();
  //  LaserBox <<"m_vel: "<<vel_x<<" "<<vel_y;
    LaserBox <<"KlamanVel: "<<m_vel.vel_x<<" "<<m_vel.vel_y<<" "<<m_vel.vel_angle<<" "<<speed_angle;
    LaserBox.EndRecord();
#endif
#endif
    /////////////////////////////////////
    //    std::cout << DethaTime << endl;

    /////////////////////////////////////

    m_CurOdmPos.x = m_LastOdmPos.x + m_vel.vel_x * DethaTime * 1000;
    m_CurOdmPos.y = m_LastOdmPos.y + m_vel.vel_y * DethaTime * 1000;
#ifndef USE_GYROANGLE
    m_CurOdmPos.fThita =m_LastOdmPos.fThita +  m_vel.vel_angle * DethaTime;//20200422由于有陀螺仪，此处thita不应处理
#endif
    m_SumOdm += m_vel_norm * DethaTime * 1000;
    m_SumAngle += m_vel.vel_angle * DethaTime;

    ///////////////////////////////

    if(m_bFirstFlagO == true)
    {
        //        m_OdmPos = m_CurOdmPos;
        m_OdmPos.x = 0;
        m_OdmPos.y = 0;
        m_OdmPos.fThita = 0;
        m_bFirstFlagO = false;
    }
    m_OdmPos.x += m_vel.vel_x * DethaTime * 1000;
    m_OdmPos.y += m_vel.vel_y * DethaTime * 1000;

    ///////////////////////////////
    /************用陀螺仪去更新角度************/
#ifdef USE_GYROANGLE
    if(fVthita > 0 || fVx > 0 || fVy > 0)
        m_bGyroStart = TRUE;
    if(m_bGyroStart)
    {
        float GyroAngle = 0;
        if(m_GyroFlag == TRUE)
        {
            m_fLastGyroAngle = fGyroAngle;
            m_fCurGyroAngle  = m_fLastGyroAngle;
            GyroAngle = m_fCurGyroAngle - m_fLastGyroAngle;
            m_GyroFlag = FALSE;
        }
        else
        {
            m_fCurGyroAngle = fGyroAngle;

            if ((m_fCurGyroAngle - m_fLastGyroAngle) > ANGLE_PI)
            {
                m_fLastGyroAngle += ANGLE_PI * 2;
            }
            else if ((m_fCurGyroAngle - m_fLastGyroAngle) < -ANGLE_PI)
            {
                m_fLastGyroAngle -= ANGLE_PI * 2;
            }
            GyroAngle = CSlamAngle::NormAngle2(m_fCurGyroAngle - m_fLastGyroAngle);
            m_fLastGyroAngle = m_fCurGyroAngle;
        }

        if(m_vel_norm < 1e-3 && fabs(m_vel.vel_angle) < 0.0005f)
            GyroAngle = 0.0f;

        m_CurOdmPos.fThita = m_LastOdmPos.fThita + GyroAngle;
        m_OdmPos.fThita += GyroAngle;


#ifdef USE_KALMANFILTER
        m_KalmanVel.vel_angle = GyroAngle / DethaTime;
        //20200605防止陀螺仪角速度计算有问题
        if(fabs(m_KalmanVel.vel_angle - vel_angle) > 0.5f)
            m_KalmanVel.vel_angle = vel_angle;
        //        std::cout << "Gyro : " << m_KalmanVel.vel_angle << " "  << m_fCurGyroAngle << " " << m_fLastGyroAngle << endl;
#endif
#ifdef USE_MEANFILTER
        m_vel.vel_angle = GyroAngle / DethaTime;
#endif
#ifndef USE_BLACK_BOX
    LaserBox.NewRecord();
    LaserBox <<"fGyro: "<<fGyroAngle<<" Gyro: "<<GyroAngle<<" time: "<<DethaTime;
    LaserBox.EndRecord();
#endif
    }
#endif
    //////////////////////////////////////////////


    /////////////////////////////////////////
    m_fOdometer[0] = m_CurOdmPos.x;
    m_fOdometer[1] = m_CurOdmPos.y;
    m_fOdometer[2] = m_CurOdmPos.fThita;
    m_fOdometer[3] += m_vel_norm * DethaTime;
    /////////////////////////////////////////
    //    std::cout << m_OdmPos.x << ", " << m_OdmPos.y << ", "  << m_OdmPos.fThita << endl;

}


const float* CSlamProc::GetOdometer()
{
    //    pthread_mutex_lock(&OdometerLock);

    return m_fOdometer;

    //    pthread_mutex_unlock(&OdometerLock);
}
void  CSlamProc::MemSetOdometer()
{
    //      pthread_mutex_lock(&OdometerLock);

    m_fOdometer[3] = 0.0f;

    //        std::cout<<"Memset"<<std::endl;

    //    pthread_mutex_unlock(&OdometerLock);
}


void  CSlamProc::SetWorkModel(int nModel)
{
    m_nWorkModel = nModel;
}


int CSlamProc::CheckLadarVel()
{
    if(fabs(CSlamAngle(pstOrgMatch.fThita - pstMatch.fThita).Degree2()) > (5/180)*PI)
        return 1;
    return 0;
}

///////////////////////////////////////////////////
/// \brief CSlamProc::MergeDoubleLaserData
///由于这两个激光头扫描范围没有重合部分，所以当前直接将两个激光头的特征合在了一起
void CSlamProc::MergeDoubleLaserData()
{
    int mark = -1;

    if(pMergeScan != NULL)
    {
       // pMergeScan->Clear();
        delete pMergeScan;
        pMergeScan == NULL;
    }

    for(int i = 0; i < LaserCount; i++)
    {
        if(Laserparm[i].state && pCurScan[i] != NULL && mark < 0)
        {
            if(pCurScan[i] == NULL)
                return;
            pMergeScan = pCurScan[i]->Duplicate();
           // pMergeScan->m_ptReflectors.clear();
            pMergeScan->m_poseScanner.SetPosture(0.0,0.0,0.0);
            mark = i;
        }
    }

    pMergeScan->m_pLineFeatures->m_nCount = 0;

    for(int j = mark; j < LaserCount; j++)
    {
        if(pCurScan[j]==NULL)
            continue;
        if(!Laserparm[j].state)
            continue;
        int count_p = pCurScan[j]->m_ptReflectors.size();
        int count_pm = pMergeScan->m_ptReflectors.size();

        for(int i = 0; i < count_p; i++)
        {
            CSlamPoint2d pt = pCurScan[j]->m_ptReflectors[i];
            bool b_near = false;
            bool b_far = false;
            for(int k = 0; k < count_pm; k++)
            {
                auto dis = pt.DistanceTo(pMergeScan->m_ptReflectors[k]);
                if(dis<=50.0f)
                    b_near = true;
                //20200406激光头安装位置不可信，为了删除干扰，将同时看到的反光板删除
                if(dis> 50.0f && dis < 500.0f)
                {                
                   b_far = true;
                   pMergeScan->m_ptReflectors[k].r = 0.0f;//20200718
                   pMergeScan->m_ptReflectors[k].UpdateCartisian();
                }   
            }
            if(b_far||b_near)
                continue;
            pMergeScan->m_ptReflectors.push_back(pt);
        }

        if(j == mark)
                pMergeScan->CreateLineFeatures();
        else
        {
            int count_l = pCurScan[j]->m_pLineFeatures->m_nCount;
            for(int k = 0; k < count_l; k++)
            {
                CLineFeature line1 = pCurScan[j]->m_pLineFeatures->m_pLines[k];
                pMergeScan->m_pLineFeatures->m_pLines[pMergeScan->m_pLineFeatures->m_nCount+k] = line1;

            }
            pMergeScan->m_pLineFeatures->m_nCount += count_l;

        }
    }

//    for (int i = 0; i < VisionLines.size(); i++)
//    {
//        VLines temp;
//        temp = VisionLines.at(i);

//        pMergeScan->m_pLineFeatures->m_pLines[pMergeScan->m_pLineFeatures->m_nCount + i].m_ptStart = temp.vstart;
//        pMergeScan->m_pLineFeatures->m_pLines[pMergeScan->m_pLineFeatures->m_nCount + i].m_ptEnd = temp.vend;
//        // pMergeScan->m_pLineFeatures->m_pLines[pMergeScan->m_pLineFeatures->m_nCount + i].m_nWhichSideToUse = 6;

//    }
//    pMergeScan->m_pLineFeatures->m_nCount = pMergeScan->m_pLineFeatures->m_nCount + VisionLines.size();
//    VisionLines.clear();
    // cout << pMergeScan->m_pLineFeatures->m_nCount << endl;
}

void CSlamProc::GetMergeScanWorld()
{
   // m_CritSection.Lock();
    if(pMergeScanWorld != NULL)
    {
        //pMergeScanWorld->Clear();
        delete pMergeScanWorld;
        pMergeScanWorld = NULL;
    }
     //   pMergeScanWorld->Clear();
    if(pMergeScan)
    {
        m_CriMergeScan.Lock();
       // cout<<"duplicate mergescan"<<endl;
        pMergeScanWorld = pMergeScan->Duplicate();
        m_CriMergeScan.Unlock();
    }
    else
        return;
    int count_p = pMergeScanWorld->m_ptReflectors.size();

    //cout<<"count_p: "<<count_p<<endl;
    for(int i = 0; i < count_p; i++)
    {
        CSlamPoint2d pt = pMergeScanWorld->m_ptReflectors[i];
       // cout<<"pt "<<i<<" "<<pt.x<<" "<<pt.y<<endl;
        pt.Move(m_CurOdmPos);
        pMergeScanWorld->m_ptReflectors[i] = pt;
    }
  //  m_CritSection.Unlock();
}

void CSlamProc::GetMergeScanWorld(std::vector<std::pair<float,float>>& point, std::vector<std::pair<float,float>>& line)
{
    int count_p = pMergeScanWorld->m_ptReflectors.size();
    for(int i = 0; i < count_p; i++)
    {
        CSlamPoint2d pt = pMergeScanWorld->m_ptReflectors[i];
        pt.Move(m_CurOdmPos);
        point.push_back(make_pair(pt.x, pt.y));
    }

}

void CSlamProc::GetFeatureMap(std::vector<std::pair<float,float>>& point, std::vector<std::pair<float,float>>& line)
{
    int count = FeatureMap.GetPointFeatures()->GetCount();
    for(int i = 0; i < count; i++)
    {
        point.push_back(make_pair(FeatureMap.GetPointFeatures()->GetPoint(i).x, FeatureMap.GetPointFeatures()->GetPoint(i).y));
    }
   // cout<<"FeatureCount:"<<FeatureMap.GetPointFeatures()->GetCount()<<endl;
    //cout<<"layercount:"<<Layer->GetCount()<<endl;
    //LineFS = FeatureMap.GetLineFeatures();
}

void CSlamProc::SetWorldMapParam(double& fUse1Or2LineDis, double& fOdmDist)
{
    Param.m_fUse1Or2LinesDis = fUse1Or2LineDis;
    m_fOdmDist = fOdmDist;
}

void CSlamProc::GetPositonMsg(std::map<std::string, float>& Data)
{
#ifdef AGV_LINUX_DEBUG
    Data["pstNew.x"] = 0.1f;
    Data["pstNew.y"] = 0.2f;
    Data["pstNew.fThita"] = 0.3f;
    Data["pstOdm.x"] = 0.4f;
    Data["pstOdm.y"] = 0.5f;
    Data["pstOdm.fThita"] = 0.6f;
    Data["Diff.x"] = 0.7f;
    Data["Diff.y"] = 0.8f;
    Data["Diff.thita"] = 0.9f;
#else
    m_CritSectionMsg.Lock();
//    Data["pstNew.x"] = pstNav.x;
//    Data["pstNew.y"] = pstNav.y;
//    Data["pstNew.fThita"] = pstNav.fThita;
//    Data["pstOdm.x"] = pstOdm.x;
//    Data["pstOdm.y"] = pstOdm.y;
//    Data["pstOdm.fThita"] = pstOdm.fThita;
    Data["Diff.x"] = pstDiff.x;
    Data["Diff.y"] = pstDiff.y;
    Data["Diff.thita"] = pstDiff.fThita;
    Data["ratio"] = Localization.GetMatchRatio();
    Data["FullMapCount"] = Localization.GetFullMapCount();
    m_CritSectionMsg.Unlock();
#endif
}
