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

// iSAMAppDoc.cpp : CiSAMAppDoc 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "iSAMApp.h"
#endif

#include "iSAMAppDoc.h"
#include "MainFrm.h"
#include "ndt_fuser/ndt_fuser_node.h"
//#include "ndt_registration/ndt_matcher_d2d_2d.h"
#include "iSAMAppView.h"
#include "SimuScanView.h"

#include <propkey.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern int Step;
extern UINT ProcessNDTProc(LPVOID pParam);
extern UINT AutoScanProc(LPVOID pParam);

//extern USHORT nCountGood;
//extern USHORT nSourceNDT;


// CiSAMAppDoc

CMapFuser* pMapFuser = NULL;

IMPLEMENT_DYNCREATE(CiSAMAppDoc, CDocument)

BEGIN_MESSAGE_MAP(CiSAMAppDoc, CDocument)
	ON_COMMAND(ID_SIMUSTART, &CiSAMAppDoc::OnSimuStart)
	ON_COMMAND(ID_RESIMU, &CiSAMAppDoc::OnResimu)
	ON_UPDATE_COMMAND_UI(ID_RESIMU, &CiSAMAppDoc::OnUpdateResimu)
	ON_UPDATE_COMMAND_UI(ID_ROBOTX, &CiSAMAppDoc::OnUpdateRobotx)
	ON_UPDATE_COMMAND_UI(ID_ROBOTY, &CiSAMAppDoc::OnUpdateRoboty)
	ON_UPDATE_COMMAND_UI(ID_ROBOTTHITA, &CiSAMAppDoc::OnUpdateRobotthita)
	ON_COMMAND(ID_PAUSEANDSTART, &CiSAMAppDoc::OnPauseAndStart)
	ON_UPDATE_COMMAND_UI(ID_PAUSEANDSTART, &CiSAMAppDoc::OnUpdatePauseAndStart)
	ON_UPDATE_COMMAND_UI(ID_EDITLOCATESTATE, &CiSAMAppDoc::OnUpdateEditlLcateState)
	ON_UPDATE_COMMAND_UI(ID_EDITCOUNTGOOD, &CiSAMAppDoc::OnUpdateEditCountGood)
	ON_UPDATE_COMMAND_UI(ID_EDITSOURCENDT, &CiSAMAppDoc::OnUpdateEditSourceNDT)
	ON_COMMAND(ID_MAP_NDT, &CiSAMAppDoc::OnMapNdt)
	ON_UPDATE_COMMAND_UI(ID_MAP_NDT, &CiSAMAppDoc::OnUpdateMapNdt)
	ON_COMMAND(ID_MAP_POINT_CLOUD, &CiSAMAppDoc::OnMapPointCloud)
	ON_UPDATE_COMMAND_UI(ID_MAP_POINT_CLOUD, &CiSAMAppDoc::OnUpdateMapPointCloud)
END_MESSAGE_MAP()


// CiSAMAppDoc 构造/析构

CiSAMAppDoc::CiSAMAppDoc()
{
	// TODO:  在此添加一次性构造代码
	m_bUseCSM = FALSE;
	m_nFrameCount = 0;
	m_bPause = FALSE;
	m_bMapNdt = FALSE;
	m_bMapPointCloud = TRUE;
}

CiSAMAppDoc::~CiSAMAppDoc()
{
}

BOOL CiSAMAppDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO:  在此添加重新初始化代码
	pMapFuser = ::new CMapFuser(500, 500, 0.2);
	if (pMapFuser == NULL)
		return false;
	// (SDI 文档将重用该文档)
	//OnSimuStart();

	// 在标题栏上显示名字
	CString strTitle = _T("GTSAM-LOOP");
	//strTitle += SW_VERSION;
	SetTitle(strTitle);
	return TRUE;
}


void CiSAMAppDoc::OnSimuStart()
{
	CiSAMAppApp* pApp = (CiSAMAppApp*)AfxGetApp();
	CMainFrame *pMain = (CMainFrame *)pApp->m_pMainWnd;
	pMain->OnSimuStart();

	if (m_bMapNdt)
		AfxBeginThread(pApp->m_pOptimizeView->ProcessNDTProc, pApp->m_pOptimizeView, THREAD_PRIORITY_NORMAL);
	else if (m_bMapPointCloud)
		AfxBeginThread(pApp->m_pOptimizeView->ProcessPointCloudProc, pApp->m_pOptimizeView, THREAD_PRIORITY_NORMAL);
}

// CiSAMAppDoc 序列化

void CiSAMAppDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO:  在此添加存储代码
	}
	else
	{
		// TODO:  在此添加加载代码
	}
}

#ifdef SHARED_HANDLERS

// 缩略图的支持
void CiSAMAppDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// 修改此代码以绘制文档数据
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// 搜索处理程序的支持
void CiSAMAppDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// 从文档数据设置搜索内容。
	// 内容部分应由“;”分隔

	// 例如:     strSearchContent = _T("point;rectangle;circle;ole object;")；
	SetSearchContent(strSearchContent);
}

void CiSAMAppDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CiSAMAppDoc 诊断

#ifdef _DEBUG
void CiSAMAppDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CiSAMAppDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG




void CiSAMAppDoc::OnResimu()
{
	// TODO:  在此添加命令处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	CSimuScanView* pSimuScanView = ((CiSAMAppApp*)AfxGetApp())->m_pSimuScanView;

	m_pScanDataSet.clear();
	pOptimizeView->m_nGetDataCount = 0;
	/*delete pMapFuser->map->getMyIndex();
	delete pMapFuser->map;
	delete pMapFuser;
	pMapFuser = NULL;*/
	pOptimizeView->Invalidate();
	Step = SIMU_READY_STEP;

	pOptimizeView->m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(pOptimizeView->m_hKillThread != NULL);

	pOptimizeView->m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(pOptimizeView->m_hThreadDead != NULL);

	AfxBeginThread(pOptimizeView->ProcessNDTProc, pOptimizeView, THREAD_PRIORITY_NORMAL);

	pSimuScanView->m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(pSimuScanView->m_hKillThread != NULL);

	pSimuScanView->m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(pSimuScanView->m_hThreadDead != NULL);

	AfxBeginThread(pSimuScanView->AutoScanProc, pSimuScanView, THREAD_PRIORITY_NORMAL);
	Step = 1;
}


void CiSAMAppDoc::OnUpdateResimu(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	pCmdUI->Enable(m_nFrameCount >= 1 && Step == SIMU_END_STEP);
}


void CiSAMAppDoc::OnUpdateRobotx(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	pCmdUI->Enable(TRUE);
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_ROBOTX));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%.2f"), pOptimizeView->m_RobotPose.x);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}


void CiSAMAppDoc::OnUpdateRoboty(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	pCmdUI->Enable(TRUE);
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_ROBOTY));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%.2f"), pOptimizeView->m_RobotPose.y);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}


void CiSAMAppDoc::OnUpdateRobotthita(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	pCmdUI->Enable(TRUE);
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_ROBOTTHITA));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%.2f"), pOptimizeView->m_RobotPose.fThita);
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}


void CiSAMAppDoc::OnPauseAndStart()
{
	// TODO:  在此添加命令处理程序代码
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	CMFCRibbonButton *button = DYNAMIC_DOWNCAST(CMFCRibbonButton, pMain->m_wndRibbonBar.FindByID(ID_PAUSEANDSTART));
	ASSERT_VALID(button);
	CString str;
	m_bPause = ~m_bPause;
	if (m_bPause)
		str.Format(_T("启动仿真"));
	else
		str.Format(_T("暂停仿真"));

	button->SetText(str);
	pOptimizeView->Invalidate(true);
}


void CiSAMAppDoc::OnUpdatePauseAndStart(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	pCmdUI->Enable(m_nFrameCount >= 1 && Step != SIMU_END_STEP && Step != SIMU_READY_STEP);
}


void CiSAMAppDoc::OnUpdateEditlLcateState(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	pCmdUI->Enable(TRUE);
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_EDITLOCATESTATE));
	ASSERT_VALID(edit);
	CString strx;
	if (pOptimizeView->m_bLocateState)
		strx.Format(_T("成功"));
	else
		strx.Format(_T("失败"));
	//strx = SlamParam.chBuf;
	//strx.Format(_T("192.168.1.10"));
	edit->SetEditText(strx);
}

void CiSAMAppDoc::OnUpdateEditCountGood(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	pCmdUI->Enable(TRUE);
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_EDITCOUNTGOOD));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%d"), pOptimizeView->m_nCountGood);
	edit->SetEditText(strx);
}


void CiSAMAppDoc::OnUpdateEditSourceNDT(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	CiSAMAppView* pOptimizeView = ((CiSAMAppApp*)AfxGetApp())->m_pOptimizeView;
	pCmdUI->Enable(TRUE);
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CMFCRibbonEdit *edit = DYNAMIC_DOWNCAST(CMFCRibbonEdit, pMain->m_wndRibbonBar.FindByID(ID_EDITSOURCENDT));
	ASSERT_VALID(edit);
	CString strx;
	strx.Format(_T("%d"), pOptimizeView->m_nSourceNDT);
	edit->SetEditText(strx);
}


void CiSAMAppDoc::OnMapNdt()
{
	// TODO:  在此添加命令处理程序代码
	m_bMapNdt = TRUE;
	m_bMapPointCloud = FALSE;
}


void CiSAMAppDoc::OnUpdateMapNdt(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	pCmdUI->SetCheck(m_bMapNdt);
}


void CiSAMAppDoc::OnMapPointCloud()
{
	// TODO:  在此添加命令处理程序代码
	m_bMapPointCloud = TRUE;
	m_bMapNdt = FALSE;
}


void CiSAMAppDoc::OnUpdateMapPointCloud(CCmdUI *pCmdUI)
{
	// TODO:  在此添加命令更新用户界面处理程序代码
	pCmdUI->SetCheck(m_bMapPointCloud);
}
