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

#include "iSAMAppDoc.h"
#include "SimuScanView.h"
#include <Afxmt.h>
#include "scan.h"
#include "SimuScanner.h"
#include "PclPointCloud.h"
#include "ndt_fuser\ndt_fuser_node.h"
#include "ndt_map\ndt_map.h"
#include "ndt_registration\ndt_matcher_d2d_2d.h"
#include "CsmMatcher.h"
#include <deque>
#include "MainFrm.h"
#include <WinGDI.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define WORLD_WIDTH				 10000
#define WORLD_HEIGHT			 10000

#define _RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

#define RANGE_LIMIT              30



extern CSimuScanner SimuScanner;
//extern CMapFuser MapFuser;

CScan* pCurScan = NULL;

HANDLE CSimuScanView::m_hKillThread = NULL;
HANDLE CSimuScanView::m_hThreadDead = NULL;

perception_oru::NDTMap *ndtmap = NULL;

extern BOOL g_bDataIsReady;

int Step = 0;
int m_nScanCount;
#define  STEP_LENGTH   200

bool loop_closed = false;

// CiSAMAppView

IMPLEMENT_DYNCREATE(CSimuScanView, CScrollView)

BEGIN_MESSAGE_MAP(CSimuScanView, CScrollView)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_WM_LBUTTONDBLCLK()
	ON_WM_MOUSEWHEEL()
	ON_WM_TIMER()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_SETCURSOR()
	ON_COMMAND(ID_USECSM, &CSimuScanView::OnUsecsm)
	ON_UPDATE_COMMAND_UI(ID_USECSM, &CSimuScanView::OnUpdateUsecsm)
	//ON_COMMAND(ID_SCANFREQUENCY, &CSimuScanView::OnScanfrequency)
	//ON_UPDATE_COMMAND_UI(ID_SCANFREQUENCY, &CSimuScanView::OnUpdateScanfrequency)
	ON_COMMAND(ID_SIMUSTARTANGLE, &CSimuScanView::OnSimustartangle)
	ON_COMMAND(ID_SIMUENDANGLE, &CSimuScanView::OnSimuendangle)
	ON_COMMAND(ID_SIMUFREQUENCY, &CSimuScanView::OnSimufrequency)
	//ON_UPDATE_COMMAND_UI(ID_SIMUFREQUENCY, &CSimuScanView::OnUpdateSimufrequency)
	
	ON_COMMAND(ID_BESTSHOW, &CSimuScanView::OnBestShow)
	ON_COMMAND(ID_STARTSIMUSCAN, &CSimuScanView::OnStartSimuScan)
	ON_COMMAND(ID_SETTRAJPOINT, &CSimuScanView::OnSetTrajPoint)
END_MESSAGE_MAP()

UINT CSimuScanView::AutoScanProc(LPVOID pParam)
{
	CSimuScanView* Obj = (CSimuScanView*)pParam;
	CiSAMAppDoc* pDoc = Obj->GetDocument();
#ifndef CONTROL_ANGLE
	CPnt pt(0, 0);
#else
	CPosture pt(-21500, 11500, 0);
	//CPosture pt(0, 3500, 0);
#ifdef NEW_TRAJ
	CPnt p11 = /*Obj->m_ScrnRef.GetWindowPoint(*/Obj->m_TrajPoints[Step - 1]/*)*/;
	CPnt p22 = /*Obj->m_ScrnRef.GetWindowPoint(*/Obj->m_TrajPoints[Step]/*)*/;
	float ang0 = atan2(p22.y - p11.y, p22.x - p11.x);
	ang0 = NormAngle(ang0);

	pt.x = Obj->m_TrajPoints[Step - 1].x;
	pt.y = Obj->m_TrajPoints[Step - 1].y;
	pt.fThita = ang0;
#endif
#endif
	/*CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;
	CPoint ptt = ScrnRef.GetWindowPoint(pt);*/

	while (WaitForSingleObject(m_hKillThread, 0) != WAIT_OBJECT_0)
	{
		if (!pDoc->m_bPause)
		{
			if (Step < SIMU_END_STEP)
			{
				//pt.x += 200;
				//pt.y += 50;
				Obj->Explore(pt);

				/*g_bDataIsReady = TRUE;*/
			}
#ifndef NEW_TRAJ
#if 1
			switch (Step)
			{
			case 1:
				/*pt.y += 50;
				if (pt.y > 10000)
				Step = 2;*/
				if (pt.x < 21500)
				{
					pt.x += STEP_LENGTH;
				}
				else
				{
#ifdef CONTROL_ANGLE
					if (pt.fThita > -1.57f)
						pt.fThita -= 0.03;
					else
#endif
						Step = 2;
				}
				break;
			case 2:
				/*pt.x += 50;
				if (pt.x > 8000)
				Step = 3;*/
				if (pt.y > -11500)
				{
					pt.y -= STEP_LENGTH;
				}
				else
				{
#ifdef CONTROL_ANGLE
					if (pt.fThita > -3.14f)
						pt.fThita -= 0.03;
					else
#endif
						Step = 3;
				}
				break;
			case 3:
				/*pt.y -= 50;
				if (pt.y < 7000)
				Step = 4;*/
				if (pt.x > -21500)
				{
					pt.x -= STEP_LENGTH;
				}
				else
				{
#ifdef CONTROL_ANGLE
					if (pt.fThita > -4.71f)
						pt.fThita -= 0.03;
					else
#endif
						Step = 4;
				}
				break;
			case 4:
				/*pt.x += 50;
				if (pt.x > 13000)
				Step = 5;*/
				if (pt.y < 11500)
				{
					pt.y += STEP_LENGTH;
				}
				else
				{
#ifdef CONTROL_ANGLE
					if (pt.fThita < 0)
						pt.fThita += 2 * 3.14f;

					if (pt.fThita > 0.01f)
						pt.fThita -= 0.03;
					else
#endif
					{
						Step = 1;
						loop_closed = true;
					}
						
				}
				break;
			case 5:
				pt.x += STEP_LENGTH;
				pt.y += 40;
				if (pt.x > 23000 && pt.y > 15000)
					Step = SIMU_END_STEP;
				break;
			default:
				Step = SIMU_END_STEP;
				break;

			}
			if (Step == SIMU_END_STEP)
				break;
#endif
#else

			if (Step - 1 < Obj->m_TrajPoints.size())
			{
				CPnt p1 = /*Obj->m_ScrnRef.GetWindowPoint(*/Obj->m_TrajPoints[Step - 1]/*)*/;
				CPnt p2 = /*Obj->m_ScrnRef.GetWindowPoint(*/Obj->m_TrajPoints[Step]/*)*/;
				float ang = atan2(p2.y - p1.y, p2.x - p1.x);
				ang = NormAngle(ang);

				pt.x += STEP_LENGTH * cos(ang);
				pt.y += STEP_LENGTH * sin(ang);

				if (pt.x < Obj->m_TrajPoints[Step].x && pt.y < Obj->m_TrajPoints[Step].y)
				{

				}
				else
				{
					pt.x = Obj->m_TrajPoints[Step].x;
					pt.y = Obj->m_TrajPoints[Step].y;
					if (Step < Obj->m_TrajPoints.size())
					{
						CPnt p3 = /*Obj->m_ScrnRef.GetWindowPoint(*/Obj->m_TrajPoints[Step + 1]/*)*/;
						//CPoint p2 = Obj->m_ScrnRef.GetWindowPoint(Obj->m_TrajPoints[Step]);
						float ang2 = atan2(p3.y - p2.y, p3.x - p2.x);
						ang2 = NormAngle(ang2);
						if (pt.fThita > ang2)
						{
							pt.fThita -= 0.01;
							if (pt.fThita < ang2)
							{
								pt.fThita = ang2;
								Step++;
							}
						}
						else
						{
							pt.fThita += 0.01;
							if (pt.fThita > ang2)
							{
								pt.fThita = ang2;
								Step++;
							}
						}
					}
					else
					{
						Step = SIMU_END_STEP;
						break;
					}
				}
				/*CCircle circle(Obj->m_TrajPoints[Step - 1], 50);
				CLine line(Obj->m_TrajPoints[Step - 1], Obj->m_TrajPoints[Step]);
				CPnt pt1;
				float dist;
				circle.IntersectLineAt(line, pt1, dist);*/
			}
#endif
			
		}
		
		Sleep(1000 / Obj->m_nSimuFrequency);
		
	//Obj->Invalidate();
	}
	SetEvent(m_hThreadDead);
	return 0;
}

Eigen::Affine3d getAsAffine(float x, float y, float yaw)
{
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x, y, 0);
	Eigen::Affine3d T = v*m;

	return T;
}

// CiSAMAppView 构造/析构

CSimuScanView::CSimuScanView()
{
	// TODO:  在此处添加构造代码
	m_bLButtonDown = FALSE;
	m_bRButtonDown = FALSE;
	m_bSetWindowViewPort = FALSE;
	m_bInSetTrajPointMode = FALSE;
	//m_nFrequency = 10;
	m_nSimuFrequency = 10;
	m_fSimuStartAngle = -137.5f/180 * PI;
	m_fSimuEndAngle = 137.5f / 180 * PI;

	m_fWorldSize = 10000;
	m_nScanCount = 0;
	m_bFirstFrame = TRUE;	
}

CSimuScanView::~CSimuScanView()
{
	CiSAMAppDoc* pDoc = GetDocument();
	pDoc->m_nFrameCount--;

	SetEvent(m_hKillThread);
	WaitForSingleObject(m_hThreadDead, 5000);

	CloseHandle(m_hKillThread);
	CloseHandle(m_hThreadDead);
}

BOOL CSimuScanView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO:  在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CScrollView::PreCreateWindow(cs);
}

// CiSAMAppView 绘制

void CSimuScanView::OnDraw(CDC* pDC)
{
	CiSAMAppDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO:  在此处为本机数据添加绘制代码

	SimuScanner.DrawWorldModel(m_ScrnRef, pDC, _RGB(64, 64,64));

	if (pCurScan != NULL)
	{
		pCurScan->Plot(m_ScrnRef, pDC, _RGB(255,0,0), _RGB(0, 255, 0), _RGB(128,255,0), TRUE, FALSE/*TRUE*/, TRUE, 1, true);
	}

	if (m_TrajPoints.size() > 0)
	{
		DrawPoints(m_TrajPoints, m_ScrnRef, pDC, RGB_(0, 0, 255), 3);
	}
	/*if (ndtmap != NULL)
	{
	ndtmap->Plot(pDC, pDoc->m_ScrnRef, _RGB(128, 255, 0), _RGB(0, 255, 128));
	}*/
}

BOOL CSimuScanView::OnMouseWheel(UINT nFlags, short zDelta, CPoint point)
{
	if (zDelta > 0)
		OnMagnify(point);
	else if (zDelta < 0)
		OnReduce(point);


	return CScrollView::OnMouseWheel(nFlags, zDelta, point);
}

void CSimuScanView::OnMagnify(CPoint point)
{
	//	point.x = 681;
	//	point.y = 319 + 10;

	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = m_ScrnRef;

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

	//	point += GetScrollPosition();
	//CPnt pt = ScrnRef.GetWorldPoint(point);
	//m_bBestFitMode = false;

	//CRect r;
	//GetClientRect(r);

	//// Set the view port
	//ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	//ScrnRef.m_fRatio *= 1.1f;
	//m_nMouseWheelMagnifyCount++;
	/*ScrnRef.SetPointMappping(point.x, point.y, pt);
	CPnt ptLeftTop = ScrnRef.GetLeftTopPoint();
	CPoint pointLeftTop = ScrnRef.GetWindowPoint(ptLeftTop);

	m_sizeTotalScroll.cx = (LONG)(WORLD_WIDTH * ScrnRef.m_fRatio * 4);
	m_sizeTotalScroll.cy = (LONG)(WORLD_HEIGHT * ScrnRef.m_fRatio * 4);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);*/

	//	ScrollToPosition(pointLeftTop);
	Invalidate();
	//UpdateWindow();
}

void CSimuScanView::OnReduce(CPoint point)
{
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = m_ScrnRef;

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

	/*point += GetScrollPosition();
	CPnt pt = ScrnRef.GetWorldPoint(point);*/
	//m_bBestFitMode = false;

	//CRect r;
	//GetClientRect(r);

	//// Set the view port
	//ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	//ScrnRef.m_fRatio /= 1.1f;
	//m_nMouseWheelReduceCount++;
	/*ScrnRef.SetPointMappping(point.x, point.y, pt);
	CPnt ptLeftTop = ScrnRef.GetLeftTopPoint();
	CPoint pointLeftTop = ScrnRef.GetWindowPoint(ptLeftTop);

	m_sizeTotalScroll.cx = (LONG)(WORLD_WIDTH * ScrnRef.m_fRatio * 4);
	m_sizeTotalScroll.cy = (LONG)(WORLD_HEIGHT * ScrnRef.m_fRatio * 4);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);*/
	//	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);

	//	ScrollToPosition(pointLeftTop);
	Invalidate();
	//UpdateWindow();
}

void CSimuScanView::ScaleToFitView()
{
	//CiSAMAppDoc* pDoc = GetDocument();
	//CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	//CRect r;
	//GetClientRect(r);
	//if (r.Width() == 0)
	//	return;

	//// Set the view port
	//ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));

	//CRectangle rect;
	//if (pMapFuser != NULL && pMapFuser->map != NULL)
	//	rect = pMapFuser->map->GetCoveringRect();

	//ptCenter = rect.GetCenterPoint();

	//m_fWorldWidth = rect.Width();
	//m_fWorldWidth += 2 * MARGIN_DIST;

	//m_fWorldHeight = rect.Height();
	//m_fWorldHeight += 2 * MARGIN_DIST;

	//// Caculate the display ratio
	//r.InflateRect(-MARGIN_SIZE, -MARGIN_SIZE);
	//float fWidthRatio = r.Width() / m_fWorldHeight;
	//float fHeightRatio = r.Height() / m_fWorldHeight;
	//ScrnRef.SetRatio(min(fWidthRatio, fHeightRatio));

	//// Set the center point
	//ScrnRef.SetCenterPoint(ptCenter);

	//m_sizeTotalScroll.cx = (LONG)(m_fWorldWidth * ScrnRef.m_fRatio);
	//m_sizeTotalScroll.cy = (LONG)(m_fWorldHeight * ScrnRef.m_fRatio);

	//SetScrollSizes(MM_TEXT, m_sizeTotalScroll);
}

void CSimuScanView::OnInitialUpdate()
{
	InitDisplay();
	((CiSAMAppApp*)AfxGetApp())->m_pSimuScanView = this;

	CScrollView::OnInitialUpdate();

	GetParent()->SetWindowText(_T("扫描"));

	CSize sizeTotal;
	// TODO:  计算此视图的合计大小
	sizeTotal.cx = sizeTotal.cy = 100;
	SetScrollSizes(MM_TEXT, sizeTotal);

	m_pstCur.SetPosture(CPnt(0, 0), CAngle(0));
	m_pstLast = m_pstCur;

	CRect r;
	GetClientRect(r);

	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = m_ScrnRef;

	pDoc->m_nFrameCount++;

	ScrnRef.SetViewPort(USHORT(r.Width()-200), USHORT(r.Height() + 750));
	float fWidthRatio = (float)r.Width() / 10;
	float fHeightRatio = (float)r.Height() / 10;
	ScrnRef.SetRatio(0.03);
	ScrnRef.SetCenterPoint(CPnt(-5, -10));

	SetTimer(1, 200, NULL);
	m_nScanCount = 0;
}

void CSimuScanView::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == 1)
	{
	//Invalidate();
	//UpdateWindow();
	}
	CScrollView::OnTimer(nIDEvent);
}

void CSimuScanView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	m_bRButtonDown = FALSE;

	ClientToScreen(&point);
	/*OnContextMenu(this, point);*/
	CMenu menu;
	menu.CreatePopupMenu();
	if (!m_bInSetTrajPointMode)
	{
		menu.AppendMenu(0, ID_BESTSHOW, _T("最佳显示"));
		menu.AppendMenu(0, ID_STARTSIMUSCAN, _T("开始扫描"));
		menu.AppendMenu(0, ID_SETTRAJPOINT, _T("设置轨迹"));
	}
	else
	{
		menu.AppendMenu(0, ID_SETTRAJPOINT, _T("结束设置轨迹"));
	}
	menu.TrackPopupMenu(TPM_LEFTALIGN | TPM_RIGHTBUTTON, point.x, point.y, this);


}

void CSimuScanView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CiSAMAppDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	CScrollView::OnLButtonDblClk(nFlags, point);

	if ((Step == SIMU_READY_STEP || Step == SIMU_END_STEP) && !m_bInSetTrajPointMode)
	{

		Step = SIMU_READY_STEP;//RESET

		m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
		ASSERT(m_hKillThread != NULL);

		m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
		ASSERT(m_hThreadDead != NULL);

		AfxBeginThread(AutoScanProc, this, THREAD_PRIORITY_NORMAL);
		Step = 1;
	}
}

void CSimuScanView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}

#ifndef CONTROL_ANGLE
void CSimuScanView::Explore(CPnt pt, BOOL bMute)
#else
void CSimuScanView::Explore(CPosture pt, BOOL bMute)
#endif
{
	
	CiSAMAppDoc* pDoc = GetDocument();

	if (pCurScan != NULL)
		delete pCurScan;

	// 计算希望达到的目标方向角
	float ang = atan2(pt.y - m_pstCur.y, pt.x - m_pstCur.x);
	ang = NormAngle(ang);

	CPosture ptOrg(pt.x, pt.y, ang);

#if 1
	//pt = AddNoiseToPoint(pt, 100);
	//pt.x += 30;
	//pt +=
	//ang = AddNoiseToAngle(ang, 10);
	//ang += 0.03;
#endif

#if 0
	pt.x = 11338;
	pt.y = 8482;
	ang = -2.138f;
#endif

	// 仿真采样时的激光头姿态
#ifndef CONTROL_ANGLE
	CPosture Pose2(pt.x, pt.y, ang);
	CPosture Pose3 = ptOrg;
#else
	CPosture Pose2 = pt;
	CPosture Pose3 = pt;
#endif


	TRACE(_T("x=%f, y=%f, ang=%f"), pt.x, pt.y, ang);

	// 进行一次仿真扫描采样
	//pCurScan = NewSimulateScan(Pose3, 2 * PI, NULL/*&pRefScan->m_poseScanner.m_pstMean*/);
	pCurScan = NewSimulateScan(Pose3,m_fSimuStartAngle, m_fSimuEndAngle, NULL/*&pRefScan->m_poseScanner.m_pstMean*/);
	m_pstCur = Pose2;
	CPclPointCloud pcloud;
	CScan *scantemp = pCurScan->Duplicate();
	CPosture result;
	Eigen::Affine3d tt;
	if (pDoc->m_bUseCSM)
	{
		if (m_nScanCount == 0)
		{
			m_nScanCount++;
			/*m_ScanData[0] = pCurScan->Duplicate();*/
			m_ScanData.push_front(*scantemp);
		}
		else
		{
			/*m_ScanData[1] = m_ScanData[0]->Duplicate();
			m_ScanData[0] = pCurScan->Duplicate();*/
			if (m_ScanData.size() < 2)
				m_ScanData.push_front(*scantemp);
			else
			{
				//m_ScanData.pop_front();
				m_ScanData.push_front(*scantemp);
			}
			m_nScanCount++;

			CPclPointCloud pCloud1 = Scan2PclCloud(m_ScanData.at(0)/*m_ScanData[m_nScanCount - 2]*/);
			CPclPointCloud pCloud2 = Scan2PclCloud(m_ScanData.at(1)/*m_ScanData[m_nScanCount - 1]*/);
			/*Eigen::Affine3f t;
			
			perception_oru::NDTMatcherD2D_2D ndt_matcher;
			ndt_matcher.ITR_MAX = 30;
			ndt_matcher.n_neighbours = 2;
			ndt_matcher.match(pCloud1, pCloud2, tt, true);*/
//#define csm
#ifdef csm
			CCsmScan scan1(m_ScanData[m_nScanCount - 2]);
			CCsmScan scan2(m_ScanData[m_nScanCount - 1]);
			CCsmMatcher	matcher;
			sm_result sr;
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
#endif
			m_ScanData.pop_back();
		}
	}
	
	/*for (int i = 0; i < pCurScan->m_nCount; i++)
	{
	CPnt pt = pCurScan->m_pPoints[i];
	CPnt pnt = pDoc->m_ScrnRef.GetWorldPoint();
	pcloud.push_back(pnt);
	}*/

	CPclPointCloud pCloud = Scan2PclCloud(*pCurScan);
	
	for (int i = 0; i < pCloud.size(); i++)
	{
		pCloud.at(i).x /= 1000;
		pCloud.at(i).y /= 1000;
		pCloud.at(i).z = (double)(rand()/*%10*/)/INT_MAX;
	}

	CScanData* pScanData = new CScanData();
	pScanData->cloud = pCloud;

	// 计算出相关姿态
	CTransform trans;
	trans.Init(m_pstLast);
	m_pstCur.x /= 1000;
	m_pstCur.y /= 1000;
	CPosture pstRelative = trans.GetLocalPosture(m_pstCur);
	Eigen::Affine3d odometry;
	if (!pDoc->m_bUseCSM)
	{
		if (m_bFirstFrame)
		{
			odometry = getAsAffine(0, 0, 0);
			m_bFirstFrame = FALSE;
		}
		else
		{
			odometry = getAsAffine(pstRelative.x + 0.000, pstRelative.y + 0.000, pstRelative.fThita + 0.00002);
		}
		//odometry = getAsAffine(m_pstCur.x + 0.01, m_pstCur.y + 0.01, m_pstCur.fThita + 0.00);
		//odometry.setIdentity();
	}
	else
	{
		if (m_nScanCount == 1)
		{
			//m_nScanCount++;
			odometry = getAsAffine(0, 0, 0);
		}
		else
		{
#ifdef csm
			odometry = getAsAffine(result.x, result.y, result.fThita);
#else
			odometry = tt;
#endif
		}
			
	}
	
	pScanData->m_pstOdometry = odometry;
	//pDoc->m_crit.Lock();
	
	pDoc->m_pScanDataSet.push_front(pScanData);
	//pDoc->m_crit.Unlock();
	/*perception_oru::NDTMap ndtlocal(::new perception_oru::LazyGrid(0.2));
	ndtlocal.guessSize(0, 0, 0, 30, 30, 0.2);
	ndtlocal.loadPointCloud(pScanData->cloud, RANGE_LIMIT);
	ndtlocal.computeNDTCells(1);
	ndtmap = ::new perception_oru::NDTMap(ndtlocal);*/
	//Eigen::Affine3d Tnow;
	//ndtmap = ::new perception_oru::NDTMap(::new perception_oru::LazyGrid(0.2));
	//ndtmap->initialize(0, 0, 0, 50, 50, 0.4);
	////ndtmap->addPointCloud(Tnow.translation(), pScanData->cloud, 0.1, 100.0, 0.1);
	//ndtmap->loadPointCloud(pScanData->cloud,RANGE_LIMIT);
	//ndtmap->computeNDTCells(1, 1e5, 255, Tnow.translation(), 0.1);
	
	/*delete pScanData;
	pScanData = NULL;*/
	//MapFuser.laserOdomCallback(odometry, pCloud);
	/*MapFuser.processFrame(pCloud, odometry);
	m_pstCur = Pose2;
	pCurScan->m_poseScanner.m_pstMean = m_pstCur;*/

	// 如果已连接
	if (0)
	{
		// 计算出相关姿态
		/*CTransform trans;
		trans.Init(m_pstLast);
		CPosture pstRelative = trans.GetLocalPosture(m_pstCur);
		pScanData->m_pstOdometry = pstRelative;*/
		/*pstRelative.x -= 30;
		pstRelative.y -= 30;
		pstRelative.fThita += 0.001;*/
	}

	m_pstLast = m_pstCur;

	Invalidate();
	//UpdateWindow();
	
}


// CiSAMAppView 诊断

#ifdef _DEBUG
void CSimuScanView::AssertValid() const
{
	CScrollView::AssertValid();
}

void CSimuScanView::Dump(CDumpContext& dc) const
{
	CScrollView::Dump(dc);
}

CiSAMAppDoc* CSimuScanView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CiSAMAppDoc)));
	return (CiSAMAppDoc*)m_pDocument;
}
#endif //_DEBUG


// CiSAMAppView 消息处理程序


void CSimuScanView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	m_bLButtonDown = TRUE;

	CScrollView::OnLButtonDown(nFlags, point);
}


void CSimuScanView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	if (m_bInSetTrajPointMode)
	{
		CScreenReference& ScrnRef = m_ScrnRef;
		CPnt pt = ScrnRef.GetWorldPoint(point);
		m_TrajPoints.push_back(pt);
		Invalidate();
	}
	
	m_bLButtonDown = FALSE;

	CScrollView::OnLButtonUp(nFlags, point);
}


void CSimuScanView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	m_bRButtonDown = TRUE;

	CScrollView::OnRButtonDown(nFlags, point);
}


void CSimuScanView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	CScreenReference& ScrnRef = m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	if (m_bLButtonDown && m_bRButtonDown && !m_bSetWindowViewPort)
	{
		m_pntRecordMouseMove = point;
		m_bSetWindowViewPort = TRUE;
		::SetCursor(LoadCursor(NULL, IDC_CROSS));
		//Invalidate();
	}

	

	if (m_bSetWindowViewPort && (!m_bLButtonDown || !m_bRButtonDown))
	{
		m_ScrnRef.SetViewPortDiff(point.x - m_pntRecordMouseMove.x, point.y - m_pntRecordMouseMove.y);
		Invalidate();
	}

	if (!m_bLButtonDown || !m_bRButtonDown)
	{
		m_bSetWindowViewPort = FALSE;
	}

	CScrollView::OnMouseMove(nFlags, point);
}


BOOL CSimuScanView::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	POINT point;
	::GetCursorPos(&point);
	ScreenToClient(&point);

	//::SetCursor(LoadCursor(NULL, IDC_SIZEALL));

	return CScrollView::OnSetCursor(pWnd, nHitTest, message);
}


void CSimuScanView::OnUsecsm()
{
	// TODO:  在此添加命令处理程序代码
	CiSAMAppDoc* pDoc = GetDocument();
	pDoc->m_bUseCSM = !pDoc->m_bUseCSM;
}


void CSimuScanView::OnUpdateUsecsm(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppDoc* pDoc = GetDocument();
	pCmdUI->SetCheck(pDoc->m_bUseCSM);
}


//void CSimuScanView::OnScanfrequency()
//{
//	// TODO:  在此添加命令处理程序代码
//	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
//	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SCANFREQUENCY));
//	ASSERT_VALID(edit);
//	CString str = edit->GetEditText();
//	m_nFrequency = _wtoi(str);
//	if (m_nFrequency <= 0)
//		m_nFrequency = 5;
//}
//
//
//void CSimuScanView::OnUpdateScanfrequency(CCmdUI *pCmdUI)
//{
//	// TODO:  在此添加命令更新用户界面处理程序代码
//	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
//	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SCANFREQUENCY));
//	ASSERT_VALID(edit);
//	CString str = edit->GetEditText();
//	m_nFrequency = _wtoi(str);
//	if (m_nFrequency <= 0)
//		m_nFrequency = 5;
//}


void CSimuScanView::OnSimustartangle()
{
	// TODO:  在此添加命令处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUSTARTANGLE));
	ASSERT_VALID(edit);
	CString str = edit->GetEditText();
	float angle = _wtof(str);
	m_fSimuStartAngle = angle / 180 * PI;
}


void CSimuScanView::OnSimuendangle()
{
	// TODO:  在此添加命令处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUENDANGLE));
	ASSERT_VALID(edit);
	CString str = edit->GetEditText();
	float angle = _wtof(str);
	m_fSimuEndAngle = angle / 180 * PI;
}


void CSimuScanView::OnSimufrequency()
{
	// TODO:  在此添加命令处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUFREQUENCY));
	ASSERT_VALID(edit);
	CString str = edit->GetEditText();
	m_nSimuFrequency = _wtoi(str);
	if (m_nSimuFrequency <= 0)
		m_nSimuFrequency = 5;
}


//void CSimuScanView::OnUpdateSimufrequency(CCmdUI *pCmdUI)
//{
//	// TODO:  在此添加命令更新用户界面处理程序代码
//	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
//	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUFREQUENCY));
//	ASSERT_VALID(edit);
//	CString str;
//	str.Format(_T("%d"), m_nSimuFrequency);
//	edit->SetEditText(str);
//}

void CSimuScanView::OnDisplaySimufrequency()
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUFREQUENCY));
	ASSERT_VALID(edit);
	CString str;
	str.Format(_T("%d"), m_nSimuFrequency);
	edit->SetEditText(str);
}

void CSimuScanView::OnDisplaySimustartangle()
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUSTARTANGLE));
	ASSERT_VALID(edit);
	CString str;
	str.Format(_T("%.2f"), m_fSimuStartAngle / PI * 180);
	edit->SetEditText(str);
}

void CSimuScanView::OnDisplaySimuendangle()
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_SIMUENDANGLE));
	ASSERT_VALID(edit);
	CString str;
	str.Format(_T("%.2f"), m_fSimuEndAngle / PI * 180);
	edit->SetEditText(str);
}

void CSimuScanView::InitDisplay()
{
	OnDisplaySimustartangle();
	OnDisplaySimuendangle();
	OnDisplaySimufrequency();
}





void CSimuScanView::OnBestShow()
{
	// TODO:  在此添加命令处理程序代码
	CScreenReference& ScrnRef = m_ScrnRef;

	CRect r;
	GetClientRect(r);

	// Set the center point
	//ScrnRef.SetCenterPoint(ptCenter);

	float fSizeX, fSizeY;

	SimuScanner.GetWorldSize(m_pstWindowCenter, fSizeX, fSizeY);

	// Caculate the display ratio
	float fWidthRatio = (float)r.Width() / fSizeX;
	float fHeightRatio = (float)r.Height() / fSizeY;
	ScrnRef.SetRatio(min(fWidthRatio, fHeightRatio) * 0.9);

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));

	ScrnRef.SetCenterPoint(m_pstWindowCenter);
	//ScrnRef.SetCenterPoint(CPnt(11050, 8030));

	Invalidate();
}


void CSimuScanView::OnStartSimuScan()
{
	// TODO:  在此添加命令处理程序代码
	if (Step == SIMU_READY_STEP || Step == SIMU_END_STEP)
	{
		Step = SIMU_READY_STEP;

		m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
		ASSERT(m_hKillThread != NULL);

		m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
		ASSERT(m_hThreadDead != NULL);

		AfxBeginThread(AutoScanProc, this, THREAD_PRIORITY_NORMAL);
		Step = 1;

		/*CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
		CMFCRibbonCheckBox *checkbox = DYNAMIC_DOWNCAST(CMFCRibbonCheckBox, pMain->m_wndRibbonBar.FindByID(ID_MAP_NDT));
		ASSERT_VALID(checkbox);
		checkbox->SetVisible(FALSE);
		checkbox->EnableUpdateTooltipInfo(FALSE);
		checkbox->*/
	}

}


void CSimuScanView::OnSetTrajPoint()
{
	// TODO:  在此添加命令处理程序代码
	m_bInSetTrajPointMode = ~m_bInSetTrajPointMode;
	if (m_bInSetTrajPointMode)
		m_TrajPoints.clear();
	if (!m_bInSetTrajPointMode && m_TrajPoints.size() < 2)
		m_TrajPoints.clear();

	Invalidate();
}

void CSimuScanView::DrawPoints(vector<CPnt>& points, CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, int nPointSize)
{
	CPen pen(PS_SOLID, 1, crColorPoint);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CBrush Brush(RGB_(0, 0, 0));
	CBrush* pOldBrush = pDC->SelectObject(&Brush);

	for (int i = 0; i < points.size(); i++)
	{
		CPoint pnt = ScrnRef.GetWindowPoint(points[i]);

		CRect r(pnt.x - nPointSize, pnt.y - nPointSize, pnt.x + nPointSize, pnt.y + nPointSize);
		pDC->Ellipse(&r);
	}
	

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);
}
