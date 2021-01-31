// 这段 MFC 示例源代码演示如何使用 MFC Microsoft Office Fluent 用户界面 
// (“Fluent UI”)。该示例仅供参考，
// 用以补充《Microsoft 基础类参考》和 
// MFC C++ 库软件随附的相关电子文档。  
// 复制、使用或分发 Fluent UI 的许可条款是单独提供的。  
// 若要了解有关 Fluent UI 许可计划的详细信息，请访问  
// http://go.microsoft.com/fwlink/?LinkId=238214。
//
// 版权所有(C) Microsoft Corporation
// 保留所有权利。

// iSAMAppView.cpp : CiSAMAppView 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "iSAMApp.h"
#endif

#include "MainFrm.h"
#include "iSAMAppDoc.h"
#include "iSAMAppView.h"
#include <Afxmt.h>

#include "SickSafetyLaserScanner.h"
#include "CsmMatcher.h"
#include "AffinePosture.h"

#include "vector.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define WORLD_WIDTH				 10000
#define WORLD_HEIGHT			 10000

#define  LASER_MIN_RANGE    0.7f
#define  LASER_MAX_RANGE    20.0f
#define  SCAN_POINT_NUM     1650
#define  START_ANGLE        -137.5f/180*PI
#define  END_ANGLE          137.5f/180*PI

extern CMapFuser *pMapFuser;



HANDLE CiSAMAppView::m_hKillThread = NULL;
HANDLE CiSAMAppView::m_hThreadDead = NULL;

HANDLE CiSAMAppView::m_hKillThreadpc = NULL;
HANDLE CiSAMAppView::m_hThreadDeadpc = NULL;

BOOL g_bDataIsReady = FALSE;
extern bool loop_closed;
//class NDTMap;
#if 0
perception_oru::NDTMap ndtlocal(new perception_oru::LazyGrid(0.2));
#endif
//CNdtKeyFrame NDTKeyFrame;
//NDTKeyFrame.ndtmap = perception_oru::NDTMap(new perception_oru::LazyGrid(0.2));

extern Eigen::Affine3d getAsAffine(float x, float y, float yaw);

Eigen::Affine3d RobotPoseAffine;

using namespace gtsam;
using namespace std;

struct sParam
{
	char* chBuf = new char[16];
	float fScanMin;
	float fScanMax;
	int   nLocationDistance;
	float fAngleDifference;
	int   nPairDistance;
	int   nScanThresholdNum;
	int   nScanThresholdIntensity;
	int   nGlobalMapMinNum;
	float fInstallAngle;
};
sParam   SlamParam;

extern int Step;

UINT CiSAMAppView::ProcessNDTProc(LPVOID pParam)
{
	CiSAMAppView* Obj = (CiSAMAppView*)pParam;

	while (WaitForSingleObject(m_hKillThread, 0) != WAIT_OBJECT_0)
	{
		if (Step == SIMU_END_STEP)
			break;

		Obj->SupportNDT();
		
		Sleep(10);
	}
	SetEvent(m_hThreadDead);
	return 0;
}

UINT CiSAMAppView::ProcessPointCloudProc(LPVOID pParam)
{
	CiSAMAppView* Obj = (CiSAMAppView*)pParam;

	while (WaitForSingleObject(m_hKillThreadpc, 0) != WAIT_OBJECT_0)
	{
		Obj->SupportPointCloudMap();
		Sleep(10);
	}
	SetEvent(m_hThreadDeadpc);

	return 0;
}

// CiSAMAppView

IMPLEMENT_DYNCREATE(CiSAMAppView, CScrollView)

BEGIN_MESSAGE_MAP(CiSAMAppView, CScrollView)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEWHEEL()
	ON_WM_TIMER()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_CHAR()
	ON_WM_LBUTTONDBLCLK()
	ON_COMMAND(ID_CONNECTLASER, &CiSAMAppView::OnConnectlaser)
	ON_UPDATE_COMMAND_UI(ID_LASERIP, &CiSAMAppView::OnUpdateLaserip)
	ON_COMMAND(ID_LASERIP, &CiSAMAppView::OnLaserip)
	ON_COMMAND(ID_LASERLIMITNEAR, &CiSAMAppView::OnLaserlimitnear)
	ON_COMMAND(ID_LASERLIMITFAR, &CiSAMAppView::OnLaserlimitfar)
	ON_COMMAND(ID_INSTALLANGLE, &CiSAMAppView::OnInstallangle)
	ON_COMMAND(ID_SCANPOINTSNUM, &CiSAMAppView::OnScanpointsnum)
	ON_COMMAND(ID_SCANFREQUENCY, &CiSAMAppView::OnScanfrequency)
	
	ON_COMMAND(ID_STOPLASERSCANNER, &CiSAMAppView::OnStopLaserScanner)
	ON_COMMAND(ID_ROTATEMAP, &CiSAMAppView::OnRotateMap)
	ON_COMMAND(ID_BUTTONROTATEMAP, &CiSAMAppView::OnButtonRotateMap)
	ON_COMMAND(ID_MOVE_GRAPH, &CiSAMAppView::OnMoveGraph)
	ON_COMMAND(ID_ROTATE_GRAPH, &CiSAMAppView::OnRotateGraph)
	ON_COMMAND(ID_OPCONFIRM, &CiSAMAppView::OnOpConfirm)
END_MESSAGE_MAP()

// CiSAMAppView 构造/析构

CiSAMAppView::CiSAMAppView()
{
	// TODO:  在此处添加构造代码
	//m_nGetDataCount = 0;
	m_bLButtonDown = FALSE;
	m_bRButtonDown = FALSE;
	m_bSetWindowViewPort = FALSE;
	m_bLaserScan = FALSE;
	m_bFirstFrame = TRUE;
	m_bConnectLaserDelay = FALSE;
	m_bStopReflesh = FALSE;
	m_bRotateMapMode = FALSE;
	m_nRotateStage = 0;
	m_nCountGood = 0;
	m_nSourceNDT = 0;

	ISAM2Params params;
	params.relinearizeThreshold = 0.01;
	params.relinearizeSkip = 1;
	isam = new ISAM2(params);
	/*gtsam::Vector vector3(3);
	vector3 << 1e-8, 1e-8, 2e-5;
	gtsam::Vector vector3_50(3);
	vector3 << 1e-8, 1e-8, 1e-3;
	prior_noise_ = noiseModel::Diagonal::Variances(vector3);
	odom_noise_ = noiseModel::Diagonal::Variances(vector3_50);
	constraint_noise_ = noiseModel::Diagonal::Variances(vector3_50);*/
	prior_noise_ = noiseModel::Diagonal::Sigmas(/*sigmas1*/gtsam::Vector3(0, 0, 0.00002));
	odom_noise_ = noiseModel::Diagonal::Sigmas(/*sigmas1*/gtsam::Vector3(0, 0, 0.001));
	constraint_noise_ = noiseModel::Diagonal::Sigmas(/*sigmas1*/gtsam::Vector3(0, 0, 0.001));
	
	cloudMap = new CPointCloudMap;

	ndtcloudMap = new CPointCloudMap;
	keyFrameCount = 0;
	FrameCount = 0;

	m_bMoveCurFrameMode = FALSE;
	m_bRotateCurFrameMode = FALSE;
	m_nRotateCurFrameStage = 0;

	m_bOptimizeConfirm = FALSE;

	TransCurFrame.setIdentity();

	m_bDisPointCloud = FALSE;
}

CiSAMAppView::~CiSAMAppView()
{
	SetEvent(m_hKillThread);
	SetEvent(m_hKillThreadpc);

	WaitForSingleObject(m_hThreadDead, 5000);
	WaitForSingleObject(m_hThreadDeadpc, 5000);

	CloseHandle(m_hKillThread);
	CloseHandle(m_hThreadDead);
	CloseHandle(m_hKillThreadpc);
	CloseHandle(m_hThreadDeadpc);
}

BOOL CiSAMAppView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO:  在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CScrollView::PreCreateWindow(cs);
}

// CiSAMAppView 绘制

void CiSAMAppView::OnDraw(CDC* pDC)
{
	CiSAMAppDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO:  在此处为本机数据添加绘制代码

	if (pMapFuser->map != NULL /*&& !pDoc->m_bPause*/)
	{
		// 显示地图
		pMapFuser->PlotModelMap(pDC/*&memoryDC*/, pDoc->m_ScrnRef, 255 << 16, 0);
	}

	if (!cloudMap->clouds.empty())
	{
		cloudMap->Plot(pDoc->m_ScrnRef, pDC, _RGB(255, 0, 0), 1);
	}

	if (!ndtcloudMap->clouds.empty() && m_bDisPointCloud/*pDoc->m_bPause*/)
	{
		ndtcloudMap->Plot(pDoc->m_ScrnRef, pDC, _RGB(0, 0, 255), 1);
	}

	if (NdtKeyFrames.size() > 0)
	{
		for (int i = 0; i < NdtKeyFrames.size(); i++)
		{
			NdtKeyFrames[i].ndtmap->Plot(pDC, pDoc->m_ScrnRef, 255 << 8, 0);
		}
	}
	/*for (int i = 0; i < keyFrameCount; i++)
	{
		NdtKeyFrames[i].ndtmap->Plot(pDC, pDoc->m_ScrnRef, 255 << 8, 0);
	}
*/
#if 0
	if(!pDoc->m_bPause)
		ndtlocal.Plot(pDC, pDoc->m_ScrnRef, 255, 0);
#endif
}


void CiSAMAppView::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == 1)
	{
		/*Invalidate(true);
		UpdateWindow();*/
	}
}
CScan *pLastScan;
CScan* pScan;
void CiSAMAppView::SupportNDT()
{
	CiSAMAppDoc* pDoc = GetDocument();
	
	//pDoc->m_crit.Lock();
	
	if (/*loop_closed && */m_bOptimizeConfirm)
	{
		m_bOptimizeConfirm = FALSE;
		m_bDisPointCloud = TRUE;
		Eigen::Affine3d correct_pose;
		correct_pose = TransCurFrame * RobotPoseAffine;
		CPosture correct_robopose = AffineToPosture(correct_pose);

		gtsam::Pose2 pose_from = Pose2(correct_robopose.x, correct_robopose.y,
			correct_robopose.fThita);

		gtsam::Pose2 pose_to = Pose2(NdtKeyFrames[0].m_RobotPose.x,
			NdtKeyFrames[0].m_RobotPose.y,
			NdtKeyFrames[0].m_RobotPose.fThita);

		graph.emplace_shared<BetweenFactor<Pose2> >(ndtcloudMap->poses.size() - 1, 0,
			/*Pose2(0, 0, 0)*/pose_from.between(pose_to), constraint_noise_);

		GaussNewtonParams parameters;
		// Stop iterating once the change in error between steps is less than this value
		parameters.relativeErrorTol = 1e-5;
		// Do not perform more than N iteration steps
		parameters.maxIterations = 100;
		// Create the optimizer ...
		GaussNewtonOptimizer optimizer(graph, initial_estimate, parameters);
		// ... and optimize
		Values result = optimizer.optimize();

		ndtcloudMap->CorrectPoses(result);

		Invalidate(true);


	}

	if (pDoc->m_pScanDataSet.size() > 0 /*&& m_nGetDataCount < pDoc->m_pScanDataSet.size() */&& !m_bLaserScan)
	{
		
		Eigen::Affine3d t;
		t.setIdentity();
		int size = pDoc->m_pScanDataSet.size();
		CPclPointCloud pcloud;
		pcloud = pDoc->m_pScanDataSet.at(size - 1)->cloud;

		if (1/*pDoc->m_bUseCSM*/)
		{
			RobotPoseAffine = pMapFuser->processFrame(pDoc->m_pScanDataSet.at(size - 1)->cloud,
				pDoc->m_pScanDataSet.at(size - 1)->m_pstOdometry, m_bLocateState, m_nCountGood,
				m_nSourceNDT);
		}
		else
		{
			RobotPoseAffine = pMapFuser->laserOdomCallback(pDoc->m_pScanDataSet.at(size - 1)->m_pstOdometry,
				pDoc->m_pScanDataSet.at(size - 1)->cloud, m_bLocateState, m_nCountGood,
				m_nSourceNDT);
		}

		m_RobotPose = AffineToPosture(RobotPoseAffine);

		ndtcloudMap->AddCloudandAbsPose(pcloud, m_RobotPose, false);
#if 0
		//perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(0.2));
		ndtlocal.guessSize(0, 0, 0, 40, 40, 0.2);
		ndtlocal.loadPointCloud(pDoc->m_pScanDataSet.at(size - 1)->cloud/*pcloud*/, 40);
		ndtlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
#endif
		//ndtKeyFrame.ndtmap = new perception_oru::NDTMap(ndtlocal);

		if (FrameCount % 50 == 0)
		{
#if 0
			CNdtKeyFrame tempKeyFrame;
			tempKeyFrame.ndtmap = new perception_oru::NDTMap(ndtlocal);

			tempKeyFrame.ndtmap->guessSize(0, 0, 0, 40, 40, 0.2);
			tempKeyFrame.ndtmap->loadPointCloud(pDoc->m_pScanDataSet.at(size - 1)->cloud/*pcloud*/, 40);
			tempKeyFrame.ndtmap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

			tempKeyFrame.m_FrameID = FrameCount;
			tempKeyFrame.m_RobotPose = m_RobotPose;

			NdtKeyFrames.push_back(tempKeyFrame);
#endif
			//NdtKeyFrames[keyFrameCount].ndtmap = new perception_oru::NDTMap(ndtlocal);

			//NdtKeyFrames[keyFrameCount].ndtmap->guessSize(0, 0, 0, 40, 40, 0.2);
			//NdtKeyFrames[keyFrameCount].ndtmap->loadPointCloud(pDoc->m_pScanDataSet.at(size - 1)->cloud/*pcloud*/, 40);
			//NdtKeyFrames[keyFrameCount].ndtmap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

			//NdtKeyFrames[keyFrameCount].m_FrameID = FrameCount;
			//NdtKeyFrames[keyFrameCount].m_RobotPose = m_RobotPose;

			keyFrameCount++;
		}
		FrameCount++;

		if (ndtcloudMap->input_ndtcloud_count == 1)
		{
			ndtcloudMap->input_ndtcloud_count++;
			graph.emplace_shared<PriorFactor<Pose2> >(0,
				Pose2(ndtcloudMap->poses[0].x, ndtcloudMap->poses[0].y,
					ndtcloudMap->poses[0].fThita), prior_noise_);
			initial_estimate.insert(0, Pose2(ndtcloudMap->poses[0].x, ndtcloudMap->poses[0].y,
				ndtcloudMap->poses[0].fThita));
		}
		else
		{
			gtsam::Pose2 pose_from = Pose2(ndtcloudMap->poses[ndtcloudMap->poses.size() - 2].x,
				ndtcloudMap->poses[ndtcloudMap->poses.size() - 2].y,
				ndtcloudMap->poses[ndtcloudMap->poses.size() - 2].fThita);
			gtsam::Pose2 pose_to = Pose2(ndtcloudMap->poses[ndtcloudMap->poses.size() - 1].x,
				ndtcloudMap->poses[ndtcloudMap->poses.size() - 1].y,
				ndtcloudMap->poses[ndtcloudMap->poses.size() - 1].fThita);
			graph.emplace_shared<BetweenFactor<Pose2>>(ndtcloudMap->poses.size() - 2,
				ndtcloudMap->poses.size() - 1, pose_from.between(pose_to), odom_noise_);
			initial_estimate.insert(ndtcloudMap->poses.size() - 1, gtsam::Pose2(ndtcloudMap->poses[ndtcloudMap->poses.size() - 1].x,
				ndtcloudMap->poses[ndtcloudMap->poses.size() - 1].y,
				ndtcloudMap->poses[ndtcloudMap->poses.size() - 1].fThita));
		}

		if (loop_closed)
		{
			CiSAMAppDoc* pDoc = GetDocument();
			pDoc->m_bPause = TRUE;
		}
			
		
		//CScanData* pdata = pDoc->m_pScanDataSet.at(size - 1);
		//delete pdata;
		pDoc->m_pScanDataSet.pop_back();
		m_nGetDataCount++;
		g_bDataIsReady = FALSE;
		/*perception_oru::NDTMap ndtlocal(::new perception_oru::LazyGrid(0.2));
		ndtlocal.guessSize(0, 0, 0, 30, 30, 0.2);
		ndtlocal.loadPointCloud(pDoc->m_pScanDataSet[0]->cloud, 30);
		ndtlocal.computeNDTCells(1);*/
		//ndtlocal.Plot()
//		MapFuser.map = new perception_oru::NDTMap;
//		MapFuser.map->loadPointCloud(pDoc->m_pScanDataSet[0]->cloud,200);
		//MapFuser.processFrame(pDoc->m_pScanDataSet[0]->cloud, pDoc->m_pScanDataSet[0]->m_pstOdometry);
		//pDoc->m_pScanDataSet.clear();

		Invalidate();
	}
	else if (m_bLaserScan && !m_bConnectLaserDelay)
	{

		std::shared_ptr<sensor::CRawPointCloud> pRawPointCloud;
		if (!m_RealScanner->GetRawPointCloud(pRawPointCloud))
			return;
		if (pScan != NULL)
			delete pScan;
		//CScan *pScanCloud = TransformCloud(pRawPointCloud/*, pScanCloud*/);
		pScan = new CScan(SCAN_POINT_NUM);
		if (pScan == NULL)
			return;

		//USHORT *dis = new USHORT[SCAN_POINT_NUM];
		USHORT dis[SCAN_POINT_NUM];
		for (int i = 0; i < SCAN_POINT_NUM; i++)
		{
			dis[i] = pRawPointCloud->distance[i];
		}
			
		
		//vector<USHORT>::iterator it = pRawPointCloud->distance.begin();
		PoseGauss pos;
		CPosture realpos;
		pScan->PolarRangesToScan(SCAN_POINT_NUM, dis, &pos, &realpos, START_ANGLE, END_ANGLE, LASER_MAX_RANGE*1000, 0);
		pScan->m_poseScanner.m_pstMean = realpos;

		Eigen::Affine3d odometry;
		if (m_bFirstFrame)
		{
			odometry = getAsAffine(0, 0, 0);
			pLastScan = pScan->Duplicate();
			m_bFirstFrame = FALSE;
		}
		else
		{
			CCsmScan scan1(*pLastScan);
			CCsmScan scan2(*pScan);
			CCsmMatcher	matcher;

			sm_result sr;
			CPosture result;
			if (matcher.Match(&scan1, &scan2, sr))
			{
				result.x = sr.x[0];
				result.y = sr.x[1];
				result.fThita = sr.x[2];

				//return true;*/
			}
			else
			{
				result.x = 0;
				result.y = 0;
				result.fThita = 0;
			}

			delete pLastScan;
			pLastScan = pScan->Duplicate();
			odometry = getAsAffine(result.x, result.y, result.fThita);
		}

		CPclPointCloud pCloud = Scan2PclCloud(*pScan);

		for (int i = 0; i < pCloud.size(); i++)
		{
			pCloud.at(i).x /= 1000;
			pCloud.at(i).y /= 1000;
			pCloud.at(i).z = (double)(rand()/*%10*/) / INT_MAX;
		}

		CScanData* pScanData = new CScanData();
		pScanData->cloud = pCloud;
		pScanData->m_pstOdometry = odometry;

		RobotPoseAffine = pMapFuser->processFrame(pScanData->cloud, pScanData->m_pstOdometry, m_bLocateState, m_nCountGood,
			m_nSourceNDT);
		m_RobotPose = AffineToPosture(RobotPoseAffine);
		delete pScanData;
		pScanData = NULL;
		Invalidate();
	}
	else if (m_bConnectLaserDelay)
	{
		
		//if (GetTickCount() - m_dwTicRec > 10000)
			m_bConnectLaserDelay = FALSE;
	}
	
	//UpdateWindow();
	//pDoc->m_crit.Unlock();
}


void CiSAMAppView::SupportPointCloudMap()
{
	CiSAMAppDoc* pDoc = GetDocument();

	if (pDoc->m_pScanDataSet.size() > 0)
	{
		int size = pDoc->m_pScanDataSet.size();

		if (cloudMap->AddCloudandPose(pDoc->m_pScanDataSet.at(size - 1)->cloud,
			pDoc->m_pScanDataSet.at(size - 1)->m_pstOdometry, loop_closed))
		{
			if (1/*!loop_closed*/)
			{
				if (cloudMap->input_cloud_count == 1)
				{
					graph.emplace_shared<PriorFactor<Pose2> >(0,
						Pose2(cloudMap->poses[0].x, cloudMap->poses[0].y,
							cloudMap->poses[0].fThita), prior_noise_);
					initial_estimate.insert(0, Pose2(cloudMap->poses[0].x, cloudMap->poses[0].y,
						cloudMap->poses[0].fThita));
				}
				else
				{
					gtsam::Pose2 pose_from = Pose2(cloudMap->poses[cloudMap->poses.size() - 2].x,
						cloudMap->poses[cloudMap->poses.size() - 2].y,
						cloudMap->poses[cloudMap->poses.size() - 2].fThita);
					gtsam::Pose2 pose_to = Pose2(cloudMap->poses[cloudMap->poses.size() - 1].x,
						cloudMap->poses[cloudMap->poses.size() - 1].y,
						cloudMap->poses[cloudMap->poses.size() - 1].fThita);
					graph.emplace_shared<BetweenFactor<Pose2>> (cloudMap->poses.size() - 2,
						cloudMap->poses.size() - 1, pose_from.between(pose_to), odom_noise_);
					initial_estimate.insert(cloudMap->poses.size() - 1, gtsam::Pose2(cloudMap->poses[cloudMap->poses.size() - 1].x,
						cloudMap->poses[cloudMap->poses.size() - 1].y,
						cloudMap->poses[cloudMap->poses.size() - 1].fThita));
				}

				/*isam->update(graph, initial_estimate);
				isam->update();

				graph.resize(0);
				initial_estimate.clear();

				isam_current_estimate = isam->calculateEstimate();*/
			}
			
		}

		if (loop_closed)
		{
			graph.emplace_shared<BetweenFactor<Pose2> >(cloudMap->poses.size() - 1, 0,
				Pose2(0, 0, 0), constraint_noise_);
			//gtsam::Pose2 pose_from = Pose2(0.0001,0.0001,0.0001);
			//gtsam::Pose2 pose_to = Pose2(0.000006,0.000004,0.0000003);
			//graph.add(BetweenFactor<Pose2> (cloudMap->poses.size() - 1, 0,
			//	pose_from.between(pose_to), constraint_noise_));
			//isam->update(graph);

			//isam->update();
			//graph.resize(0);
			GaussNewtonParams parameters;
			// Stop iterating once the change in error between steps is less than this value
			parameters.relativeErrorTol = 1e-5;
			// Do not perform more than N iteration steps
			parameters.maxIterations = 100;
			// Create the optimizer ...
			GaussNewtonOptimizer optimizer(graph, initial_estimate, parameters);
			// ... and optimize
			Values result = optimizer.optimize();

			//cloudMap->CorrectPoses(isam_current_estimate);
			cloudMap->CorrectPoses(result);

			CiSAMAppDoc* pDoc = GetDocument();
			pDoc->m_bPause = TRUE;
			/*pDoc->OnUpdatePauseAndStart()*/

			loop_closed = false;

			Invalidate(true);
		}
		//m_nGetDataCount++;

		pDoc->m_pScanDataSet.pop_back();

		Invalidate(FALSE);
		UpdateWindow();
	}

}

//
// 对输入的原始点云转换成笛卡尔坐标系.
//

#if 1
CScan* pscan;
CScan* CiSAMAppView::TransformCloud(const std::shared_ptr<sensor::CRawPointCloud>& pRawCloud/*, CScan* pScan*/)
{
	if (!pRawCloud)
	{
		return false;
	}
	
	if (pscan != NULL)
		delete pscan;

	pscan = new CScan(SCAN_POINT_NUM);
	

	float first_angle = pRawCloud->start_angle;
	unsigned int num_pts = pRawCloud->num_points;
	short laser_id = pRawCloud->laser_id;
	float angular_increment = 0.0;

	if (num_pts > 0) 
	{
		angular_increment = (pRawCloud->end_angle - pRawCloud->start_angle) / num_pts;
	}

	if (num_pts > pRawCloud->distance.size())
	{
		return false;
	}

	double min_range = LASER_MIN_RANGE;
	double max_range = LASER_MAX_RANGE/*m_ScannerGroupParam[laser_id].m_fMaxRange*/;

	Eigen::Vector3d pt;
	float r = 0.0;
	float a = 0.0;
	//cloud_trans.m_vPointCloud.reserve(num_pts);
	// 引入Z方向的噪声
	double varz = 0.05 / (double)INT_MAX;

	USHORT *dis = new USHORT(SCAN_POINT_NUM);
	std::shared_ptr<USHORT> scandis;
	for (int i = 0; i < num_pts; i++)
	{
		*dis = pRawCloud->distance[i];
		dis++;
		r = (float)pRawCloud->distance[i] / 1000.0;  //原始数据距离以mm为单位，需要转换为m
		a = first_angle + i*angular_increment;

		// 过滤掉不满足距离的点
		if (r < min_range || r > max_range)
		{
			r = 0.0;
		}

		// 过虑掉不满足可视角度的点
		/*if (!m_ScannerGroupParam[laser_id].m_AppAngleRange.Contain(a)){
			r = 0.0;
			}*/

		// 极径大于0，表示数据有效
		if (r >= 0.0)
		{
			pt(0) = r * cos(a);
			pt(1) = r * sin(a);
			pt(2) = varz*rand();
			//            if (std::isnan(pt(0)) || std::isnan(pt(1)) || std::isnan(pt(2)))
			//                continue;
			//cloud_trans.m_vPointCloud.push_back(pt);
			//pScan
		}
		//cout<<"trans_size: "<<cloud_trans.m_vPointCloud.size()<<endl;
	}
	PoseGauss pos;
	CPosture realpos;
	pscan->PolarRangesToScan(SCAN_POINT_NUM, dis, &pos, &realpos, START_ANGLE, END_ANGLE, LASER_MAX_RANGE, 0);
	pscan->m_poseScanner.m_pstMean = realpos;
	//pScan = pscan;
	delete dis;
	return pscan;
	//return true;
}
#endif

void CiSAMAppView::OnInitialUpdate()
{
	LoadParam();//加载设置参数
	InitDisplay();

	((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView = this;

	CScrollView::OnInitialUpdate();

	GetParent()->SetWindowText(_T("优化"));
	EnableToolTips(TRUE);

	SetTimer(1, 500, NULL);

	CRect r;
	GetClientRect(r);

	CSize sizeTotal;
	// TODO:  计算此视图的合计大小
	sizeTotal.cx = sizeTotal.cy = 100;
	SetScrollSizes(MM_TEXT, sizeTotal);

	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	float fWidthRatio = (float)r.Width() / 10;
	float fHeightRatio = (float)r.Height() / 10;
	ScrnRef.SetRatio(min(fWidthRatio, fHeightRatio));
	ScrnRef.SetCenterPoint(CPnt(0, 0));
	ScrnRef.SetCenterPoint(CPnt(0, 0));
	//CScreenReference s1(1024, 800, 28.2f, CPnt(0, 0));

	//ScrnRef = s1;
	m_nGetDataCount = 0;

	m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hKillThread != NULL);

	m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hThreadDead != NULL);

	m_hKillThreadpc = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hKillThreadpc != NULL);

	m_hThreadDeadpc = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hThreadDeadpc != NULL);

	/*if (pDoc->m_bMapNdt)
		AfxBeginThread(ProcessNDTProc, this, THREAD_PRIORITY_NORMAL);
	else if (pDoc->m_bMapPointCloud)
		AfxBeginThread(ProcessPointCloudProc, this, THREAD_PRIORITY_NORMAL);*/
}

void CiSAMAppView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	m_bRButtonDown = FALSE;
	m_bRotateMapMode = FALSE;
	ClientToScreen(&point);
	/*OnContextMenu(this, point);*/
	/*CMenu menu;
	menu.CreatePopupMenu();
	if (1)
	{
	menu.AppendMenu(0, ID_ROTATEMAP, _T("旋转视图"));
	}

	menu.TrackPopupMenu(TPM_LEFTALIGN | TPM_RIGHTBUTTON, point.x, point.y, this);*/
}

BOOL CiSAMAppView::OnMouseWheel(UINT nFlags, short zDelta, CPoint point)
{
	if (zDelta > 0)
		OnMagnify(point);
	else if (zDelta < 0)
		OnReduce(point);


	return CScrollView::OnMouseWheel(nFlags, zDelta, point);
}

void CiSAMAppView::OnMagnify(CPoint point)
{
	//	point.x = 681;
	//	point.y = 319 + 10;

	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;
	//	point += GetScrollPosition();
	CPnt pt = ScrnRef.GetWorldPoint(point);
	m_bBestFitMode = false;

	CRect r;
	GetClientRect(r);

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	ScrnRef.m_fRatio *= 1.1f;
	//m_nMouseWheelMagnifyCount++;
	ScrnRef.SetPointMappping(point.x, point.y, pt);
	CPnt ptLeftTop = ScrnRef.GetLeftTopPoint();
	CPoint pointLeftTop = ScrnRef.GetWindowPoint(ptLeftTop);

	/*m_sizeTotalScroll.cx = (LONG)(WORLD_WIDTH * ScrnRef.m_fRatio * 4);
	m_sizeTotalScroll.cy = (LONG)(WORLD_HEIGHT * ScrnRef.m_fRatio * 4);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);*/

	//	ScrollToPosition(pointLeftTop);
	Invalidate(true);
	//UpdateWindow();
}

void CiSAMAppView::OnReduce(CPoint point)
{
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;
	point += GetScrollPosition();
	CPnt pt = ScrnRef.GetWorldPoint(point);
	m_bBestFitMode = false;

	CRect r;
	GetClientRect(r);

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	ScrnRef.m_fRatio /= 1.1f;
	//m_nMouseWheelReduceCount++;
	ScrnRef.SetPointMappping(point.x, point.y, pt);
	CPnt ptLeftTop = ScrnRef.GetLeftTopPoint();
	CPoint pointLeftTop = ScrnRef.GetWindowPoint(ptLeftTop);

	/*m_sizeTotalScroll.cx = (LONG)(WORLD_WIDTH * ScrnRef.m_fRatio * 4);
	m_sizeTotalScroll.cy = (LONG)(WORLD_HEIGHT * ScrnRef.m_fRatio * 4);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);*/
	//	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);

	//	ScrollToPosition(pointLeftTop);
	Invalidate(true);
	//UpdateWindow();
}


void CiSAMAppView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CiSAMAppView 诊断

#ifdef _DEBUG
void CiSAMAppView::AssertValid() const
{
	CScrollView::AssertValid();
}

void CiSAMAppView::Dump(CDumpContext& dc) const
{
	CScrollView::Dump(dc);
}

CiSAMAppDoc* CiSAMAppView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CiSAMAppDoc)));
	return (CiSAMAppDoc*)m_pDocument;
}
#endif //_DEBUG


// CiSAMAppView 消息处理程序

void CiSAMAppView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	m_bLButtonDown = TRUE;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	if (m_bRotateMapMode)
		DealWithRotateMap(point);

	if (m_bMoveCurFrameMode)
		DealWithMoveCurFrame(point);
	else
		m_nMoveCurFrameStage = 0;

	if (m_bRotateCurFrameMode)
		DealWithRotateCurFrame(point);
	else
		m_nRotateCurFrameStage = 0;

	CScrollView::OnLButtonDown(nFlags, point);
}


void CiSAMAppView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	m_bLButtonDown = FALSE;

	CScrollView::OnLButtonUp(nFlags, point);
}


void CiSAMAppView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	m_bRButtonDown = TRUE;

	CScrollView::OnRButtonDown(nFlags, point);
}


void CiSAMAppView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 计算当前鼠标位置
	CPnt pt = ScrnRef.GetWorldPoint(point);
	CString str, str1 = _T("");
	str.Format(_T("当前:(%.3f, %.3f)"), pt.x, pt.y);
	str += _T("         ");

	CMFCRibbonBaseElement * pElement = (CMFCRibbonStatusBarPane*)pMain->m_wndStatusBar.FindElement(ID_STATUSBAR_PANE1);
	pElement->SetText(str);
	pElement->Redraw();

	if (m_bLButtonDown && m_bRButtonDown && !m_bSetWindowViewPort)
	{
		m_pntRecordMouseMove = point;
		m_bSetWindowViewPort = TRUE;
		::SetCursor(LoadCursor(NULL, IDC_CROSS));
		//Invalidate();
	}



	if (m_bSetWindowViewPort && (!m_bLButtonDown || !m_bRButtonDown))
	{
		pDoc->m_ScrnRef.SetViewPortDiff(point.x - m_pntRecordMouseMove.x, point.y - m_pntRecordMouseMove.y);
		Invalidate();
	}

	if (!m_bLButtonDown || !m_bRButtonDown)
	{
		m_bSetWindowViewPort = FALSE;
	}

	CScrollView::OnMouseMove(nFlags, point);
}


void CiSAMAppView::OnConnectlaser()
{
	// TODO:  在此添加命令处理程序代码
	sick::cSickSafetyLaserScanner* m_Scanner = new sick::cSickSafetyLaserScanner(SCAN_POINT_NUM,
		START_ANGLE,
		END_ANGLE);
	m_RealScanner = m_Scanner;
	std::string str_ip = "192.168.3.112";
	std::string host_ip = "192.168.3.28";
	char* s_ip = "192.168.3.112";
	char * h_ip = "192.168.3.254";
	//m_RealScanner->Start(str_ip.c_str(), host_ip.c_str(), 0);
	m_RealScanner->Start(s_ip, h_ip, 0);

	//AfxBeginThread(ProcessNDTProc, this, THREAD_PRIORITY_NORMAL);//210109

	m_bLaserScan = TRUE;
	m_bFirstFrame = TRUE;
	m_bConnectLaserDelay = TRUE;
	m_dwTicRec = GetTickCount();
}

void CiSAMAppView::LoadParam()
{
	FILE* file = fopen("SlamParam.txt", "r");
	fscanf(file, "%s\n", SlamParam.chBuf);
	fscanf(file, "%f\t%f\n", &SlamParam.fScanMin, &SlamParam.fScanMax);
	fscanf(file, "%d\t%f\n", &SlamParam.nLocationDistance, &SlamParam.fAngleDifference);
	fscanf(file, "%d\n%d\t%d\n%d\n", &SlamParam.nPairDistance, &SlamParam.nScanThresholdNum,
		&SlamParam.nScanThresholdIntensity, &SlamParam.nGlobalMapMinNum);
	fscanf(file, "%f\n", &SlamParam.fInstallAngle);
	fclose(file);
	//fInstallAngle = SlamParam.fInstallAngle;

}

void CiSAMAppView::InitDisplay()
{
	OnDisplayLaserIP();
	OnDisplayLaserLimitNear();
	OnDisplayLaserLimitFar();
	OnDisplayLaserInstallAngle();
	OnDisplayLaserScanPointNum();
	OnDisplayLaserScanFrequency();
}

void CiSAMAppView::OnDisplayLaserIP()
{
	// TODO:  在此添加命令处理程序代码
	/*CMFCRibbonBar* pRibbon = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
	CMFCRibbonEdit *edit = dynamic_cast<CMFCRibbonEdit*>(pRibbon->FindByID(ID_LASERIP));*/
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_LASERIP));
	ASSERT_VALID(edit);
	CString strx(SlamParam.chBuf);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppView::OnDisplayLaserLimitNear()
{
	// TODO:  在此添加命令处理程序代码
	/*CMFCRibbonBar* pRibbon = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
	CMFCRibbonEdit *edit = dynamic_cast<CMFCRibbonEdit*>(pRibbon->FindByID(ID_LASERIP));*/
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_LASERLIMITNEAR));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%.2f"), SlamParam.fScanMin);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppView::OnDisplayLaserLimitFar()
{
	// TODO:  在此添加命令处理程序代码
	/*CMFCRibbonBar* pRibbon = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
	CMFCRibbonEdit *edit = dynamic_cast<CMFCRibbonEdit*>(pRibbon->FindByID(ID_LASERIP));*/
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_LASERLIMITFAR));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%.2f"), SlamParam.fScanMax);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppView::OnDisplayLaserInstallAngle()
{
	// TODO:  在此添加命令处理程序代码
	/*CMFCRibbonBar* pRibbon = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
	CMFCRibbonEdit *edit = dynamic_cast<CMFCRibbonEdit*>(pRibbon->FindByID(ID_LASERIP));*/
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_INSTALLANGLE));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%.2f"), SlamParam.fInstallAngle);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppView::OnDisplayLaserScanPointNum()
{
	// TODO:  在此添加命令处理程序代码
	/*CMFCRibbonBar* pRibbon = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
	CMFCRibbonEdit *edit = dynamic_cast<CMFCRibbonEdit*>(pRibbon->FindByID(ID_LASERIP));*/
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SCANPOINTSNUM));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%d"), 1650);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppView::OnDisplayLaserScanFrequency()
{
	// TODO:  在此添加命令处理程序代码
	/*CMFCRibbonBar* pRibbon = ((CFrameWndEx*)AfxGetMainWnd())->GetRibbonBar();
	CMFCRibbonEdit *edit = dynamic_cast<CMFCRibbonEdit*>(pRibbon->FindByID(ID_LASERIP));*/
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SCANFREQUENCY));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%d"), 20);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppView::OnUpdateLaserip(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
}


void CiSAMAppView::OnLaserip()
{
	// TODO:  在此添加命令处理程序代码
}


void CiSAMAppView::OnLaserlimitnear()
{
	// TODO:  在此添加命令处理程序代码
}


void CiSAMAppView::OnLaserlimitfar()
{
	// TODO:  在此添加命令处理程序代码
}


void CiSAMAppView::OnInstallangle()
{
	// TODO:  在此添加命令处理程序代码
}


void CiSAMAppView::OnScanpointsnum()
{
	// TODO:  在此添加命令处理程序代码
}


void CiSAMAppView::OnScanfrequency()
{
	// TODO:  在此添加命令处理程序代码
}


void CiSAMAppView::OnStopLaserScanner()
{
	// TODO:  在此添加命令处理程序代码
	//m_bStopReflesh = TRUE;
	m_bLaserScan = FALSE;
}


void CiSAMAppView::OnRotateMap()
{
	// TODO:  在此添加命令处理程序代码
	m_bRotateMapMode = TRUE;
}
void CiSAMAppView::DealWithRotateMap(CPoint point)
{
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	switch (m_nRotateStage)
	{
	case 0:
	{
		m_ptRotateCenter = pt;                // 定义旋转中心
		m_nRotateStage = 1;
	}
		break;
	case 1:                                  // 定义拼接块第二个对应点
	{
		m_ptFrom = pt;
		m_nRotateStage = 2;
	}
		break;
	case 2:
	{
		// 如果距离过短无法处理，退出
		if (m_ptRotateCenter.DistanceTo(pt) < 0.01f)
			return;

		m_ptTo = pt;
		CLine ln1(m_ptRotateCenter, m_ptFrom);
		float fLen = ln1.Length();

		CLine ln2(m_ptRotateCenter, m_ptTo);
		CAngle ang = ln1.AngleToLine(ln2);

		float x0 = m_ptRotateCenter.x;
		float y0 = m_ptRotateCenter.y;
		double dx = x0 * cos(ang) - y0 * sin(ang) - x0;
		double dy = y0 * cos(ang) + x0 * sin(ang) - y0;

		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
		tr = Eigen::Translation<double, 3>(-dx, -dy, 0) *
			Eigen::AngleAxis<double>(ang.m_fRad, Eigen::Vector3d::UnitZ());

		pMapFuser->map->Transform(tr);
		//m_trans = tr * m_trans;

		Invalidate();
		//m_nWorkSubType = 0;
		m_nRotateStage = 0;
	}
		break;
	}
}




void CiSAMAppView::OnButtonRotateMap()
{
	// TODO:  在此添加命令处理程序代码
	m_bRotateMapMode = TRUE;
}


void CiSAMAppView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// 1. Create a factor graph container and add factors to it
	NonlinearFactorGraph graph;

	// 2a. Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
	//Vector sigmas1(0.3,0.3,0.1);
	
	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(/*sigmas1*/gtsam::Vector3(0.3, 0.3, 0.1));
	graph.emplace_shared<PriorFactor<Pose2> >(1, Pose2(0, 0, 0), priorNoise);

	// For simplicity, we will use the same noise model for odometry and loop closures
	//Vector sigmas2(0.2, 0.2, 0.1);
	noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(/*sigmas2*/gtsam::Vector3(0.2, 0.2, 0.1));

	// 2b. Add odometry factors
	// Create odometry (Between) factors between consecutive poses
	graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0), model);
	graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), model);
	graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), model);
	graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2, 0, M_PI_2), model);

	// 2c. Add the loop closure constraint
	// This factor encodes the fact that we have returned to the same pose. In real systems,
	// these constraints may be identified in many ways, such as appearance-based techniques
	// with camera images. We will use another Between Factor to enforce this constraint:
	graph.emplace_shared<BetweenFactor<Pose2> >(5, 2, Pose2(2, 0, M_PI_2), model);
	graph.print("\nFactor Graph:\n"); // print

	// 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
	Values initialEstimate;
	initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
	initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
	initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
	initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
	initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
	initialEstimate.print("\nInitial Estimate:\n"); // print

	// 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
	// The optimizer accepts an optional set of configuration parameters,
	// controlling things like convergence criteria, the type of linear
	// system solver to use, and the amount of information displayed during
	// optimization. We will set a few parameters as a demonstration.
	GaussNewtonParams parameters;
	// Stop iterating once the change in error between steps is less than this value
	parameters.relativeErrorTol = 1e-5;
	// Do not perform more than N iteration steps
	parameters.maxIterations = 100;
	// Create the optimizer ...
	GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
	// ... and optimize
	Values result = optimizer.optimize();
	result.print("Final Result:\n");

	// 5. Calculate and print marginal covariances for all variables
	cout.precision(3);
	Marginals marginals(graph, result);
	cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
	cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
	cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
	cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
	cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;
	ISAM2 *isam;
	ISAM2Params params;
	params.relinearizeThreshold = 0.01;
	params.relinearizeSkip = 1;
	isam = new ISAM2(params);
	isam->update(graph, initialEstimate);
	isam->update();
	Values isam_estimate = isam->calculateEstimate();
	Pose2 lastest_estimate = isam_estimate.at<Pose2>(isam_estimate.size() - 1);

	// 1. Create a factor graph container and add factors to it
	//NonlinearFactorGraph graph;

	//// 2a. Add a prior on the first pose, setting it to the origin
	//// A prior factor consists of a mean and a noise model (covariance matrix)
	//noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
	//graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

	//// For simplicity, we will use the same noise model for odometry and loop closures
	//noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

	//// 2b. Add odometry factors
	//// Create odometry (Between) factors between consecutive poses
	//graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0), model));
	//graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
	//graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
	//graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

	//// 2c. Add the loop closure constraint
	//// This factor encodes the fact that we have returned to the same pose. In real systems,
	//// these constraints may be identified in many ways, such as appearance-based techniques
	//// with camera images. We will use another Between Factor to enforce this constraint:
	//graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));
	//graph.print("\nFactor Graph:\n"); // print

	//// 3. Create the data structure to hold the initialEstimate estimate to the solution
	//// For illustrative purposes, these have been deliberately set to incorrect values
	//Values initialEstimate;
	//initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
	//initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
	//initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
	//initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
	//initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
	//initialEstimate.print("\nInitial Estimate:\n"); // print

	//// 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
	//// The optimizer accepts an optional set of configuration parameters,
	//// controlling things like convergence criteria, the type of linear
	//// system solver to use, and the amount of information displayed during
	//// optimization. We will set a few parameters as a demonstration.
	//GaussNewtonParams parameters;
	//// Stop iterating once the change in error between steps is less than this value
	//parameters.relativeErrorTol = 1e-5;
	//// Do not perform more than N iteration steps
	//parameters.maxIterations = 100;
	//// Create the optimizer ...
	//GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
	//// ... and optimize
	//Values result = optimizer.optimize();
	//result.print("Final Result:\n");

	//// 5. Calculate and print marginal covariances for all variables
	//cout.precision(3);
	//Marginals marginals(graph, result);
	//cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
	//cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
	//cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
	//cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
	//cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;
}

void CiSAMAppView::OnMoveGraph()
{
	// TODO: 在此添加命令处理程序代码
	

	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	//CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	CMFCRibbonButton *button = DYNAMIC_DOWNCAST(CMFCRibbonButton, pMain->m_wndRibbonBar.FindByID(ID_MOVE_GRAPH));
	ASSERT_VALID(button);
	CString str;
	m_bMoveCurFrameMode = ~m_bMoveCurFrameMode;
	if (m_bMoveCurFrameMode)
		str.Format(_T("停止平移"));
	else
		str.Format(_T("开启平移"));

	button->SetText(str);
}


void CiSAMAppView::OnRotateGraph()
{
	// TODO: 在此添加命令处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	//CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	CMFCRibbonButton *button = DYNAMIC_DOWNCAST(CMFCRibbonButton, pMain->m_wndRibbonBar.FindByID(ID_ROTATE_GRAPH));
	ASSERT_VALID(button);
	CString str;
	m_bRotateCurFrameMode = ~m_bRotateCurFrameMode;
	if (m_bRotateCurFrameMode)
		str.Format(_T("停止旋转"));
	else
		str.Format(_T("开启旋转"));

	button->SetText(str);
}

void CiSAMAppView::DealWithRotateCurFrame(CPoint point)
{
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	switch (m_nRotateCurFrameStage)
	{
	case 0:
	{
		m_ptRotateCurFrameCenter = pt;                // 定义旋转中心
		m_nRotateCurFrameStage = 1;
	}
	break;
	case 1:                                  // 定义拼接块第二个对应点
	{
		m_ptCurFrameFrom = pt;
		m_nRotateCurFrameStage = 2;
	}
	break;
	case 2:
	{
		// 如果距离过短无法处理，退出
		if (m_ptRotateCurFrameCenter.DistanceTo(pt) < 0.01f)
			return;

		m_ptCurFrameTo = pt;
		CLine ln1(m_ptRotateCurFrameCenter, m_ptCurFrameFrom);
		float fLen = ln1.Length();

		CLine ln2(m_ptRotateCurFrameCenter, m_ptCurFrameTo);
		CAngle ang = ln1.AngleToLine(ln2);

		float x0 = m_ptRotateCurFrameCenter.x;
		float y0 = m_ptRotateCurFrameCenter.y;
		double dx = x0 * cos(ang) - y0 * sin(ang) - x0;
		double dy = y0 * cos(ang) + x0 * sin(ang) - y0;

		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
		tr = Eigen::Translation<double, 3>(-dx, -dy, 0) *
			Eigen::AngleAxis<double>(ang.m_fRad, Eigen::Vector3d::UnitZ());

		TransCurFrame = tr * TransCurFrame;
		/*pMapFuser->map->Transform(tr);*/
		//m_trans = tr * m_trans;
#if 0
		ndtlocal.Transform(tr);
#endif

		Invalidate();
		//m_nWorkSubType = 0;
		m_nRotateCurFrameStage = 0;
	}
	break;
	}
}


void CiSAMAppView::DealWithMoveCurFrame(CPoint point)
{
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	switch (m_nMoveCurFrameStage)
	{
	case 0:
	{
		m_ptCurFrameFrom = pt;
		m_nMoveCurFrameStage = 1;
	}
	break;
	case 1:
	{
		m_ptCurFrameTo = pt;
		
		CPosture pst;
		pst.x = -m_ptCurFrameTo.x + m_ptCurFrameFrom.x;
		pst.y = -m_ptCurFrameTo.y + m_ptCurFrameFrom.y;
		pst.fThita = 0;

		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
		tr = Eigen::Translation<double, 3>(-pst.x, -pst.y, 0) *
			Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ());

		TransCurFrame = tr * TransCurFrame;
		/*pMapFuser->map->Transform(tr);*/
		//m_trans = tr * m_trans;
#if 0
		ndtlocal.Transform(tr);
#endif
		Invalidate();
		//m_nWorkSubType = 0;
		m_nMoveCurFrameStage = 0;
	}
	break;
	}
}

void CiSAMAppView::OnOpConfirm()
{
	// TODO: 在此添加命令处理程序代码
	m_bOptimizeConfirm = TRUE;
}
